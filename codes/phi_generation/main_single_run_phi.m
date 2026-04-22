function results = main_single_run_phi(phi_expr, emax, safety_method, mu_value, opts)
%MAIN_SINGLE_RUN_PHI  Closed-loop sim with custom phi/PSC (lives in codes/phi_generation for merge hygiene)
%   results = main_single_run_phi() - Default parameters (standalone; prefer codes/main_single_run.m script for paper baseline)
%   results = main_single_run_phi(phi_expr, emax, safety_method, mu_value) - Custom parameters
%   results = main_single_run_phi(phi_expr, emax, safety_method, mu_value, opts) - With estimator/velocity overrides
%
%   This function runs a closed-loop lane keeping simulation with an MPC
%   controller. It can be run standalone (no arguments) or called as a
%   function with custom parameters. It patches fun_safety_condition via
%   set_phi_expr and must be paired with compare_phi / run_llm_pipeline workflows.
%
%   SAFETY METHODS: (same as historical main_single_run on function-prompting branch)
%   'PSC'  - uses fun_safety_condition with phi expression
%   'CDBF' / 'DIRECT' / 'NONE' - as documented in branch history
%
%   Examples:
%       results = main_single_run_phi('1 - (e/emax)^4', 5, 'PSC', 0.3);
%       opts.mu_0 = 0.9; opts.sigma_0 = 0.05; opts.bar_sigma = 0.05; opts.init_v = 20;
%       results = main_single_run_phi('1 - (e/emax)^4', 10, 'PSC', 0.3, opts);
%
%   See also: compare_phi_expressions.m, set_phi_expr.m, restore_fun_safety_from_main.m, main_single_run.m (script, main)

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

    if nargin < 5, opts = struct(); end
    if ~isfield(opts, 'mu_0'),      opts.mu_0      = 0.30;          end
    if ~isfield(opts, 'sigma_0'),   opts.sigma_0   = 0.01;          end
    if ~isfield(opts, 'bar_sigma'), opts.bar_sigma = 0.1;           end
    if ~isfield(opts, 'init_v'),    opts.init_v    = 20*1000/3600;  end

    %%%  Path to model (this M-file is under codes/phi_generation)
    thisDir  = fileparts(mfilename('fullpath'));
    codesDir = fileparts(thisDir);
    addpath(thisDir, codesDir, ...
        fullfile(codesDir, 'impl_controller'), ...
        fullfile(codesDir, 'impl_model'), ...
        fullfile(codesDir, 'impl_estimator'), ...
        fullfile(codesDir, 'impl_road'));
    mdl = 'mdl_closed_loop_mpc'; % MPC with code generaion
    load_system(mdl)

    %% Simulation settings (common to all parallel simulations) %%%%%

    % Online or fixed estimation
    ONLINE_ESTIMATION = '1';
    FIXED_ESTIMATION  = '0';
    set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION)
    prior_str = sprintf('[%.4f, %.4f]', opts.mu_0, opts.sigma_0);
    set_param([mdl '/estimate_fixed'], 'Value',  prior_str)
    set_param([mdl '/prior'],'InitialCondition', prior_str)
    set_param([mdl '/mes_var'], 'Value', num2str(opts.bar_sigma))

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
    V0 = opts.init_v; % longitudinal speed [m/s]
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
    nlobj.Weights.OutputVariables = [0.05 1 1]; % [Vx,e,psi]
    nlobj.Weights.ManipulatedVariablesRate = [1 1];

    %% Set phi expression and emax config (must happen before validation & codegen)
    set_phi_expr(phi_expr);
    fprintf('Using barrier function: phi = %s\n', phi_expr);

    fid = fopen(fullfile(codesDir, 'get_emax_config.m'), 'w');
    fprintf(fid, 'function val = get_emax_config()\n    val = %g;\nend\n', emax);
    fclose(fid);

    clear fun_safety_condition get_emax_config;
    rehash;

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
            fprintf('  -> phi expressions WILL affect results\n');
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

    %% Closed-loop simulation
    disp('--- start simulation ----')

    % Friction coefficient
    set_param([mdl '/true_friction_coeff'],'Value', num2str(mu_value) )

    % Simulink MATLAB Function blocks read from the base workspace,
    % but when this runs as a function the variables are local.
    assignin('base', 'coreData',   coreData);
    assignin('base', 'onlineData', onlineData);

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
    results.opts = opts;

    disp('--- completed ----')

end
