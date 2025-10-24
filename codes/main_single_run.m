% Main program for closed-loop simulation
%
clear

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
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
set_param([mdl '/prior'],'InitialCondition', '[0.30, 0.01]')

% Num MC sims for safety probability calculation
set_param([mdl '/SafeProbabilityMC'],'snum','1') % 100 samples

% Visulization
set_param([mdl '/visualization'],'Commented','off') 

% Termination condition
TERM_DIST = '300'; 
set_param([mdl '/termination_dist'], 'Value', TERM_DIST)

TERM_LAT_ERROR  = '300';
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
%%% constraint
%nlobj.Optimization.CustomIneqConFcn = "fun_inequality"; % PSC (Proposed)
nlobj.Optimization.CustomIneqConFcn = "fun_inequality_CDBF"; % CDBF
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
    Cfg = coder.config('mex');
    %Cfg.DynamicMemoryAllocation = 'off'; %% Deprecated in Matlab2025a
    cfg.EnableDynamicMemoryAllocation = true;     %% After Matlab2025a
    cfg.DynamicMemoryAllocationThreshold = 65536; %% After Matlab2025a
    codegen('-config',Cfg,func,'-o',funcOutput,'-args',...
        {coder.Constant(coreData), xk', mv, onlineData});
end


%% Closed-loop simulation
disp('--- start simulation ----')

% Friction coefficient
mu = 0.2;
set_param([mdl '/true_friction_coeff'],'Value', num2str(mu) )

res = sim([mdl '.slx']);

prob  = renamevars( res.SafeProb.extractTimetable, 'Data', 'prob');
dist  = renamevars( res.ProbDist.extractTimetable, 'Data', 'dist');
state = renamevars( res.State.extractTimetable, 'Data', 'state');
input = renamevars( res.Input.extractTimetable, 'Data', 'input');
force = renamevars( res.TireForce.extractTimetable, 'Data', 'force');
slipA = renamevars( res.slipA.extractTimetable, 'Data', 'slipA');
slipR = renamevars( res.slipR.extractTimetable, 'Data', 'slipR');
traj  = renamevars( res.Trajectory.extractTimetable, 'Data', 'traj');
LfP   = renamevars( res.LfP.extractTimetable, 'Data', 'LfP');
LgP   = renamevars( res.LgP.extractTimetable, 'Data', 'LgP');
BP    = renamevars( res.BP.extractTimetable,  'Data', 'BP');

%save 'data/data_Nominal_mu09' prob dist state input force slipA slipR traj LfP LgP BP

disp('--- completed ----')

function cineq = myIneqConFunction(X,U,e,data,mu,probs)
    u = U(1,:)';
    epsilon = 0.1;  alpha   = 1.0; 
    p = probs(1); LfP = probs(2); LgP = probs(3:4); BP = probs(5);
    cineq = -LgP * u - LfP - BP - alpha * ( p - (1-epsilon) );
    %disp([cineq, e])
    %cineq = -1;
end

