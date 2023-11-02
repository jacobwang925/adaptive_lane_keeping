% Main program to run simulation
%
clear

%%%  Load simulin model %%%%%%%%%%%
addpath impl_controller
addpath impl_model
addpath impl_estimator
addpath impl_road

mdl = 'mdl_closed_loop_mpc';
load_system([mdl '.slx'])

%%% Set parameters  %%%%%%%%%%%%%%

% Safe controller or nominal controller
SAFE_CTRL    = '1';
NOMINAL_CTRL = '0';
%set_param([mdl '/control_safe_or_nom'],'sw', NOMINAL_CTRL)
set_param([mdl '/SafeProbabilityMC'],'snum','100')

% Online or fixed estimation
ONLINE_ESTIMATION = '1';
FIXED_ESTIMATION  = '0';
set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION)
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
set_param([mdl '/prior'],'InitialCondition', '[0.30, 0.01]')

% Visulization
set_param([mdl '/visualization'],'Commented','on')

% Termination condition
TERM_DIST_NOM = '127.2';
TERM_DIST_LNG = '200'; 
set_param([mdl '/termination_dist'], 'Value', TERM_DIST_NOM)

TERM_LAT_NOM  = '40';
TERM_LAT_LNG  = '300';
set_param([mdl '/termination_lat'], 'Value', TERM_LAT_NOM)

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
mu    = 0.9;
probs = [1, 0, 1, 1, 0]; % [P, LfP, LgP, BP] 
createParameterBus(nlobj,[mdl '/Nonlinear MPC Controller'],'myBusObject',{mu,probs});
% weights of objective function
nlobj.Weights.OutputVariables = [0.05 1 1];
nlobj.Weights.ManipulatedVariablesRate = [1 1];
% constraint
%nlobj.Optimization.CustomIneqConFcn = @myIneqConFunction; % defined below
%nlobh.Jacobian.CustomIneqConFcn     = @myIneqConJacobian; 



%%% Run simulation and store data  %%%%%%%%%%
%rng(1)

% Initial state
V0 = 20 * 1000/3600; % longitudinal speed [m/s]
Re = 0.325; % Wheel radius [m]
INIT_DYN  = [V0, 0, 0, 0];
INIT_OMG  = V0/Re*ones(1,4); 
INIT_ROAD = [0,0,0];
init = [INIT_DYN  INIT_OMG 0 INIT_ROAD];
set_param([mdl '/initial_vehicle_state'],'Value', ['[' num2str(init) ']'] )

% Closed-loop simulation
num = 10;

PROB  = NaN(num, 40/0.1);
SPEED = NaN(num, 40/0.1);
TRAJ  = NaN(num, 40/0.1, 3);
for i = 1:num

    mu = 0.2 + rand*0.2;
    set_param([mdl '/true_friction_coeff'],'Value', num2str(mu) )

    disp( [num2str(i) '/' num2str(num) ': mu=' num2str(mu,'%.3f') ' (' char(datetime) ')' ] )
    res = sim([mdl '.slx']);

    prob  = res.SafeProb.extractTimetable;
    dist  = res.ProbDist.extractTimetable;
    state = res.State.extractTimetable;
    traj  = res.Trajectory.extractTimetable;

    len = length(prob.Data);
    PROB( i, 1:len) = prob.Data;
    SPEED(i, 1:len) = state.Data(:,1);
    TRAJ( i, 1:len, :) = traj.Data;

end

P = mean( rmmissing(reshape(PROB, [],1)) );
V = mean( rmmissing(reshape(SPEED, [],1)) )* 3600/1000;

save 'data_mpc/data_Nominal_multi_icy' PROB SPEED TRAJ


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

