function res = run_closed_loop_single(prior_ic, mes_var, emax, mu_gt, v0)
% Main program for closed-loop simulation

%%%  Path to model %%%%%%%%%%%
addpath impl_controller
addpath impl_model
addpath impl_estimator
addpath impl_road
mdl = 'mdl_closed_loop_mpc'; % MPC with code generaion
load_system(mdl)

%% Simulation settings (common to all pallarell simulations) %%%%%

% Online or fixed estimation
ONLINE_ESTIMATION = '1';
FIXED_ESTIMATION  = '0';
set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION)
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')                 % keep as given
set_param([mdl '/prior'],'InitialCondition', mat2str(prior_ic))               % uses input prior_ic
set_param([mdl '/mes_var'], 'Value', num2str(mes_var))                        % uses input mes_var


% Lane error tolerance
set_param([mdl '/SafeProbabilityMC'],'emax',num2str(emax)) % default is 5

% Num MC sims for safety probability calculation
set_param([mdl '/SafeProbabilityMC'],'snum','100') % change to 100 for reproduction

% Visulization
set_param([mdl '/visualization'],'Commented','off') % 'on' to disable

% Termination condition
TERM_DIST = '150'; 
set_param([mdl '/termination_dist'], 'Value', TERM_DIST)

TERM_LAT_ERROR  = '100';
set_param([mdl '/termination_lat'], 'Value', TERM_LAT_ERROR)

% Initial state
V0 = v0 * 1000/3600; % longitudinal speed [m/s]
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
%%% constraint
%nlobj.Optimization.CustomIneqConFcn = [];
nlobj.Optimization.CustomIneqConFcn = "fun_inequality"; % PSC (Proposed)
%nlobj.Optimization.CustomIneqConFcn = "fun_inequality_CDBF"; % CDBF
%nlobj.Optimization.CustomIneqConFcn = @myIneqConFunction; % defined below

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
    cfg = coder.config('mex');                       % fixed var name
    %Cfg.DynamicMemoryAllocation = 'off'; %% Deprecated in Matlab2025a
    cfg.EnableDynamicMemoryAllocation = true;     %% After Matlab2025a
    cfg.DynamicMemoryAllocationThreshold = 65536; %% After Matlab2025a
    codegen('-config',cfg,func,'-o',funcOutput,'-args',...
        {coder.Constant(coreData), xk', mv, onlineData});
end

assignin('base','coreData',coreData);
assignin('base','onlineData',onlineData);

%% Closed-loop simulation
disp('--- start simulation ----')

% Friction coefficient
mu = mu_gt;
set_param([mdl '/true_friction_coeff'],'Value', num2str(mu) )

res_sim = sim([mdl '.slx']);

prob  = renamevars( res_sim.SafeProb.extractTimetable, 'Data', 'prob');
dist  = renamevars( res_sim.ProbDist.extractTimetable, 'Data', 'dist');
state = renamevars( res_sim.State.extractTimetable, 'Data', 'state');
input = renamevars( res_sim.Input.extractTimetable, 'Data', 'input');
force = renamevars( res_sim.TireForce.extractTimetable, 'Data', 'force');
slipA = renamevars( res_sim.slipA.extractTimetable, 'Data', 'slipA');
slipR = renamevars( res_sim.slipR.extractTimetable, 'Data', 'slipR');
traj  = renamevars( res_sim.Trajectory.extractTimetable, 'Data', 'traj');
LfP   = renamevars( res_sim.LfP.extractTimetable, 'Data', 'LfP');
LgP   = renamevars( res_sim.LgP.extractTimetable, 'Data', 'LgP');
BP    = renamevars( res_sim.BP.extractTimetable,  'Data', 'BP');

%save 'data/data_Nominal_mu09' prob dist state input force slipA slipR traj LfP LgP BP

disp('--- completed ----')

% Compute and print average longitudinal speed
% Extract longitudinal velocity (Vx = first column of state)
if istimetable(state)
    Vx = state.state(:,1);
    meanVx = mean(Vx,'omitnan');
    stdVx  = std(Vx,'omitnan');
    fprintf('Average longitudinal speed: %.2f m/s (std: %.2f)\n', meanVx, stdVx);
    fprintf('Equivalent average speed: %.2f km/h\n', meanVx * 3.6);
end

% package outputs
res = struct('prob',prob,'dist',dist,'state',state,'input',input, ...
             'force',force,'slipA',slipA,'slipR',slipR,'traj',traj, ...
             'LfP',LfP,'LgP',LgP,'BP',BP);
end
