% Main program to run simulation
%
clear

%%%  Load simulin model %%%%%%%%%%%
disp('--- loading simulation model ----')
addpath impl_controller
addpath impl_model
addpath impl_estimator
addpath impl_road

mdl = 'mdl_closed_loop_mpc';
load_system([mdl '.slx'])

% Friction coefficient
mu = 0.9;
set_param([mdl '/true_friction_coeff'],'Value', num2str(mu) )

% Safe controller or nominal controller
%SAFE_CTRL    = '1';
%NOMINAL_CTRL = '0';
%set_param([mdl '/control_safe_or_nom'],'sw', NOMINAL_CTRL)

set_param([mdl '/SafeProbabilityMC'],'snum','50')

% Online or fixed estimation
ONLINE_ESTIMATION = '1';
FIXED_ESTIMATION  = '0';
set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION)
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
set_param([mdl '/prior'],'InitialCondition', '[0.30, 0.01]')

% Visulization
set_param([mdl '/visualization'],'Commented','off')

% Termination condition
TERM_DIST_NOM = '127.2';
TERM_DIST_LNG = '300'; 
set_param([mdl '/termination_dist'], 'Value', TERM_DIST_LNG)

TERM_LAT_NOM  = '40';
TERM_LAT_LNG  = '300';
set_param([mdl '/termination_lat'], 'Value', TERM_LAT_LNG)


% Initial state
V0 = 20 * 1000/3600; % longitudinal speed [m/s]
Re = 0.325; % Wheel radius [m]
INIT_DYN  = [V0, 0, 0, 0];
INIT_OMG  = V0/Re*ones(1,4); 
INIT_ROAD = [0,0,0];
init = [INIT_DYN  INIT_OMG 0 INIT_ROAD];
set_param([mdl '/initial_vehicle_state'],'Value', ['[' num2str(init) ']'] )

%%% MPC controller %%%%%%%%%%%%%%%%%%%%%%%%%%
disp('--- defining MPC controller ----')
% dimensions
nlobj = nlmpc(12,3,2);    
nlobj.Ts = 0.1;
nlobj.PredictionHorizon = 25;
nlobj.ControlHorizon = 2;
% state equation and parameters
nlobj.Model.StateFcn =  @(x,u,mu,probs) fun_f(x,mu) + fun_g(x)* u;
nlobj.Model.OutputFcn = @(x,u,mu,probs) [x(1);x(11);x(12)];
nlobj.Model.NumberOfParameters = 2; 
probs = [1, 0, 1, 1, 0]; % [P, LfP, LgP, BP] 
createParameterBus(nlobj,[mdl '/Nonlinear MPC Controller'],'myBusObject',{mu,probs});
% weights of objective function
nlobj.Weights.OutputVariables = [0.05 1 1];
nlobj.Weights.ManipulatedVariablesRate = [1 1];
% constraint
nlobj.Optimization.CustomIneqConFcn = @myIneqConFunction; % defined below
nlobh.Jacobian.CustomIneqConFcn     = @myIneqConJacobian; 

% validation
u0 = [0 0];
ref0 = [40*1000/3600 0 0];
validateFcns(nlobj,init,u0,{},{mu,probs},ref0);
nloptions = nlmpcmoveopt;
nloptions.Parameters = {mu, probs};
yref = ref0;
mv = u0;
xk = init;
[mv,nloptions] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);

%% Closed-loop simulation
disp('--- start simulation ----')

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


function cineq = myIneqConFunction(X,U,e,data,mu,probs)
    u = U(1,:)';
    epsilon = 0.1;  alpha   = 1.0; 
    p = probs(1); LfP = probs(2); LgP = probs(3:4); BP = probs(5);
    cineq = -LgP * u - LfP - BP - alpha * ( p - (1-epsilon) );
    %disp([cineq, e])
    %cineq = -1;
end

function [G,Gmv,Ge] = myIneqConJacobian(X,U,e,data,params)

    LgP = probs(3:4); 
    Nx = data.NumOfStates;
    Nmv = length(data.MVIndex);
    Nc = 1;
    p = data.PredictionHorizon;
    G = zeros(p,Nx,Nc);
    Gmv = zeros(p,Nmv,Nc);
    Gmv(1,:,1) = -LgP;
    Ge = 0;

end