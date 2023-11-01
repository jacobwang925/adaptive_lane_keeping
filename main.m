% Main program to run simulation
%
clear

%%%  Load simulin model %%%%%%%%%%%
addpath impl_controller
addpath impl_model
addpath impl_estimator
addpath impl_road

mdl = 'mdl_closed_loop';
load_system([mdl '.slx'])

%%% Set parameters  %%%%%%%%%%%%%%

% Friction coefficient
mu = 0.9;
set_param([mdl '/true_friction_coeff'],'Value', num2str(mu) )

% Safe controller or nominal controller
SAFE_CTRL    = '1';
NOMINAL_CTRL = '0';
set_param([mdl '/control_safe_or_nom'],'sw', NOMINAL_CTRL)

set_param([mdl '/SafeProbabilityMC'],'snum','100')


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


%%% Run simulation and store data  %%%%%%%%%%

% Initial state
V0 = 20 * 1000/3600; % longitudinal speed [m/s]
Re = 0.325; % Wheel radius [m]
INIT_DYN  = [V0, 0, 0, 0];
INIT_OMG  = V0/Re*ones(1,4); 
INIT_ROAD = [0,0,0];
init = [INIT_DYN  INIT_OMG 0 INIT_ROAD];
set_param([mdl '/initial_vehicle_state'],'Value', ['[' num2str(init) ']'] )

% Closed-loop simulation
res = sim('mdl_closed_loop.slx');

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

save 'data/data_Nominal_mu09' prob dist state input force slipA slipR traj LfP LgP BP
