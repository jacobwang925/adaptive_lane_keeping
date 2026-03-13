function results = main_single_run(phi_expr, emax, safety_method, mu_value)
%MAIN_SINGLE_RUN  Run closed-loop simulation with configurable parameters
%   results = main_single_run() - Run with default parameters (standalone mode)
%   results = main_single_run(phi_expr, emax, safety_method, mu_value) - Run with custom parameters
%
%   This function runs a closed-loop lane keeping simulation with an MPC
%   controller. It can be run standalone (no arguments) or called as a
%   function with custom parameters.
%
%   SAFETY METHODS:
%   The safety_method parameter determines which safety constraint is used:
%
%   'PSC'  - Probabilistic Safety Certificate (Proposed method)
%            * Uses fun_safety_condition() with phi expression
%            * Monte Carlo simulation calculates p, LfP, LgP, BP
%            * Constraint: -LgP*u - LfP - BP - alpha*(p - (1-epsilon)) >= 0
%            * This is the ONLY method affected by phi expressions!
%
%   'CDBF' - Control-Dependent Barrier Function
%            * Uses vehicle dynamics (yaw rate, steering, velocity)
%            * Does NOT use fun_safety_condition or phi expressions
%
%   'DIRECT' - Direct Lane Keeping Constraint (default for standalone)
%            * Hardcoded: phi = 1 - (e/emax)^2 > 0
%            * Does NOT use fun_safety_condition or phi expressions
%
%   'NONE' - Adaptive MPC without safety constraints
%            * No safety constraints, only adaptive MPC
%            * Does NOT use fun_safety_condition or phi expressions
%
%   Inputs:
%       phi_expr - Barrier function expression (default: '1 - (e/emax)^2')
%                  Only used when safety_method = 'PSC'!
%                  Example: 'cos(pi*e/(2*emax))' for cosine barrier
%       emax     - Lane error tolerance in meters (default: 15)
%       safety_method - 'PSC', 'CDBF', 'DIRECT', or 'NONE' (default: 'DIRECT')
%       mu_value - Friction coefficient (default: 0.3)
%                  0.3 = icy, 0.5 = normal, 0.9 = dry
%
%   Output:
%       results - Struct containing:
%         .prob   - Safety probability timeseries
%         .state  - System state timeseries
%         .input  - Control inputs timeseries
%         .traj   - Vehicle trajectory
%         .LfP, LgP, BP - Lie derivatives and backup probability
%         plus metadata: phi_expr, emax, safety_method, mu_value
%
%   Examples:
%       % Run with defaults (standalone mode)
%       main_single_run
%
%       % PSC with custom phi expression (for phi comparison studies)
%       results = main_single_run('1 - (e/emax)^4', 5, 'PSC', 0.3);
%
%       % CDBF method (phi expression ignored)
%       results = main_single_run('', 10, 'CDBF', 0.5);
%
%   See also: compare_phi_expressions.m, set_phi_expr.m, fun_safety_condition.m

    % Check if running standalone (no arguments) or being called as function
    standalone_mode = (nargin == 0);

    if standalone_mode
        clear; close all;
        % Default parameters for standalone mode
        phi_expr = '1 - (e/emax)^2';
        emax = 15;
        safety_method = 'DIRECT';
        mu_value = 0.3;
    else
        % Validate inputs
        if nargin < 1 || isempty(phi_expr)
            phi_expr = '1 - (e/emax)^2';
        end
        if nargin < 2 || isempty(emax)
            emax = 15;
        end
        if nargin < 3 || isempty(safety_method)
            safety_method = 'DIRECT';
        end
        if nargin < 4 || isempty(mu_value)
            mu_value = 0.3;
        end
    end

    %%%  Path to model %%%%%%%%%%%
    addpath impl_controller
    addpath impl_model
    addpath impl_estimator
    addpath impl_road
    mdl = 'mdl_closed_loop_mpc'; % MPC with code generaion
    load_system(mdl)

    %% Simulation settings (common to all parallel simulations) %%%%%

    % Online or fixed estimation
    ONLINE_ESTIMATION = '1';
    FIXED_ESTIMATION  = '0';
    set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION)
    set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
    set_param([mdl '/prior'],'InitialCondition', '[0.30, 0.01]')
    set_param([mdl '/mes_var'], 'Value', '0.1')

    % Lane error tolerance
    set_param([mdl '/SafeProbabilityMC'],'emax',num2str(emax))

    % Num MC sims for safety probability calculation
    set_param([mdl '/SafeProbabilityMC'],'snum','100')

    % Visualization
    if standalone_mode
        set_param([mdl '/visualization'],'Commented','off')
    else
        set_param([mdl '/visualization'],'Commented','on')
    end

    % Termination condition
    TERM_DIST = '150';
    set_param([mdl '/termination_dist'], 'Value', TERM_DIST)

    TERM_LAT_ERROR  = '100';
    set_param([mdl '/termination_lat'], 'Value', TERM_LAT_ERROR)

    % Initial state
    V0 = 20 * 1000/3600; % longitudinal speed [m/s]
    Re = 0.325; % Wheel radius [m]
    INIT_DYN  = [V0, 0, 0, 0];
    INIT_OMG  = V0/Re*ones(1,4);
    INIT_ROAD = [0,0,0];
    init = [INIT_DYN  INIT_OMG 0 INIT_ROAD];
    set_param([mdl '/initial_vehicle_state'],'Value', ['[' num2str(init) ']'] )

    %%% MPC controller
    disp('--- defining MPC controller ----')
    % dimensions
    nlobj = nlmpc(12,3,2);
    nlobj.Ts = 0.2;
    nlobj.PredictionHorizon = 10;
    nlobj.ControlHorizon = 2;
    % state equation and parameters
    nlobj.Model.StateFcn =  "fun_system_dynamics";
    nlobj.Model.OutputFcn = "fun_output_function";
    nlobj.Model.NumberOfParameters = 2;
    % weights of objective function
    nlobj.Weights.OutputVariables = [0.03 1 1]; % [Vx,e,psi]
    nlobj.Weights.ManipulatedVariablesRate = [1 1];

    %%% constraint - select based on safety_method
    switch upper(safety_method)
        case 'PSC'
            nlobj.Optimization.CustomIneqConFcn = "fun_inequality";
            disp('Using PSC (Proposed) safety method');
            fprintf('  -> phi expressions WILL affect results\n');
        case 'CDBF'
            nlobj.Optimization.CustomIneqConFcn = "fun_inequality_CDBF";
            disp('Using CDBF safety method');
            fprintf('  -> phi expressions will be IGNORED\n');
        case 'DIRECT'
            nlobj.Optimization.CustomIneqConFcn = "fun_inequality_direct_lane_keep";
            disp('Using DIRECT safety method');
            fprintf('  -> phi expressions will be IGNORED\n');
        case 'NONE'
            nlobj.Optimization.CustomIneqConFcn = [];
            disp('Using AMPC (no safety constraints)');
            fprintf('  -> phi expressions will be IGNORED\n');
        otherwise
            error('Unknown safety_method: %s. Use PSC, CDBF, DIRECT, or NONE.', safety_method);
    end

    % validation for codegeneration
    u0 = [0 0];
    ref0 = [40*1000/3600 0 0];
    mu = 0.9;
    probs = [1, 0, 1, 1, 0]; % [P, LfP, LgP, BP]
    validateFcns(nlobj,init,u0,{},{mu,probs},ref0);
    nloptions = nlmpcmoveopt;
    nloptions.Parameters = {mu, probs};
    yref = ref0;
    mv = u0;
    xk = init;
    [mv,nloptions] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);

    % code generation
    code_gen = true;
    [coreData,onlineData] = getCodeGenerationData(nlobj,xk',mv,nloptions.Parameters);
    if code_gen
        disp('--- building MPC controller ----')
        func = 'nlmpcmoveCodeGeneration';
        funcOutput = 'fun_mpc_controller';
        Cfg = coder.config('mex');
        Cfg.EnableDynamicMemoryAllocation = true;
        Cfg.DynamicMemoryAllocationThreshold = 65536;
        codegen('-config',Cfg,func,'-o',funcOutput,'-args',...
            {coder.Constant(coreData), xk', mv, onlineData});
    end

    %% Set phi expression
    set_phi_expr(phi_expr);
    fprintf('Using barrier function: phi = %s\n', phi_expr);
    clear fun_safety_condition;
    rehash;

    %% Closed-loop simulation
    disp('--- start simulation ----')

    % Friction coefficient
    set_param([mdl '/true_friction_coeff'],'Value', num2str(mu_value) )

    res = sim([mdl '.slx']);

    %% Collect results
    results = struct();
    results.prob  = renamevars(res.SafeProb.extractTimetable, 'Data', 'prob');
    results.dist  = renamevars(res.ProbDist.extractTimetable, 'Data', 'dist');
    results.state = renamevars(res.State.extractTimetable, 'Data', 'state');
    results.input = renamevars(res.Input.extractTimetable, 'Data', 'input');
    results.force = renamevars(res.TireForce.extractTimetable, 'Data', 'force');
    results.slipA = renamevars(res.slipA.extractTimetable, 'Data', 'slipA');
    results.slipR = renamevars(res.slipR.extractTimetable, 'Data', 'slipR');
    results.traj  = renamevars(res.Trajectory.extractTimetable, 'Data', 'traj');
    results.LfP   = renamevars(res.LfP.extractTimetable, 'Data', 'LfP');
    results.LgP   = renamevars(res.LgP.extractTimetable, 'Data', 'LgP');
    results.BP    = renamevars(res.BP.extractTimetable,  'Data', 'BP');

    % Additional metadata
    results.phi_expr = phi_expr;
    results.emax = emax;
    results.safety_method = safety_method;
    results.mu_value = mu_value;

    disp('--- completed ----')

end
