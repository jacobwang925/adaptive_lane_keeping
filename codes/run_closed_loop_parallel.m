function res = run_closed_loop_parallel(prior_ic, mes_var, emax, mu_gt, saveData)

% Main program for closed-loop simulation (multiple parallel runs)

% mu_gt = 0.3; % icy
% mu_gt = 0.5; % normal
% mu_gt = 0.8; % dry

% --- Determine road condition string based on mu_gt ---
if mu_gt < 0.4
    condition_str = 'icy';
elseif mu_gt <= 0.7
    condition_str = 'normal';
else
    condition_str = 'dry';
end

%% AMPC

%%%  Path to model %%%%%%%%%%%
addpath impl_controller
addpath impl_model
addpath impl_estimator
addpath impl_road
mdl = 'mdl_closed_loop_mpc'; % MPC with code generaion
load_system(mdl)

% Online or fixed estimation
ONLINE_ESTIMATION = '1';
FIXED_ESTIMATION  = '0';
set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION) % Online Estimation
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
set_param([mdl '/prior'],'InitialCondition', mat2str(prior_ic))               % uses input prior_ic
set_param([mdl '/mes_var'], 'Value', num2str(mes_var))                        % uses input mes_var

% Lane error tolerance
set_param([mdl '/SafeProbabilityMC'],'emax',num2str(emax)) % default is 5

% Num MC sims for safety probability calculation
set_param([mdl '/SafeProbabilityMC'],'snum','100') % 100 samples

% Visulization
set_param([mdl '/visualization'],'Commented','on') % No visualization

% Termination condition
TERM_DIST = '150'; % '127.2'
set_param([mdl '/termination_dist'], 'Value', TERM_DIST)

TERM_LAT_ERROR  = '100'; %30
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
nlobj.PredictionHorizon = 10;  % Horizon
nlobj.ControlHorizon = 2;
% state equation and parameters
nlobj.Model.StateFcn =  "fun_system_dynamics";
nlobj.Model.OutputFcn = "fun_output_function";
nlobj.Model.NumberOfParameters = 2; 
% weights of objective function
nlobj.Weights.OutputVariables = [0.05 1 1]; % [Vx,e,psi] 
nlobj.Weights.ManipulatedVariablesRate = [1 1];
% constraint
nlobj.Optimization.CustomIneqConFcn = [];
%nlobj.Optimization.CustomIneqConFcn = "fun_inequality";   % PSC (proposed)
%nlobj.Optimization.CustomIneqConFcn = "fun_inequality_CDBF";  % CDBF

% validation for codegeneration
u0 = [0 0];
ref0 = [40*1000/3600 0 0];
mu = 0.9;
probs = [1, 0, 1, 1, 0]; % [P, LfP, LgP, BP] for testing
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
    cfg = coder.config('mex');
    %Cfg.DynamicMemoryAllocation = 'off'; %% Deprecated in Matlab2025a
    cfg.EnableDynamicMemoryAllocation = true;     %% After Matlab2025a
    cfg.DynamicMemoryAllocationThreshold = 65536; %% After Matlab2025a
    codegen('-config',cfg,func,'-o',funcOutput,'-args',...
        {coder.Constant(coreData), xk', mv, onlineData});
end

assignin('base','coreData',coreData);
assignin('base','onlineData',onlineData);

save_system(mdl,[],'OverwriteIfChangedOnDisk',true)


% Simulink objects for parllel computation
num_sims = 20;
MU  = NaN(num_sims,1); 
for i = 1:num_sims
    in(i) = Simulink.SimulationInput(mdl);

    MU(i) = mu_gt + rand*0.1;
    in(i) = setBlockParameter(in(i), [mdl '/true_friction_coeff'], Value=num2str(MU(i)) );
end

% Pallarell simulations
num_pools = 20;
poolobj = gcp("nocreate"); % If no pool, do not create new one.
if isempty(poolobj)
    parpool(num_pools)
end
ress = parsim(in, 'ShowSimulationManager', 'off', 'TransferBaseWorkspaceVariables','on');

% Store data
PROB  = NaN(num_sims, 40/0.1);
SPEED = NaN(num_sims, 40/0.1);
TRAJ  = NaN(num_sims, 40/0.1, 3);
STATE = NaN(num_sims, 40/0.1, 12);
ETIME = NaN(num_sims, 40/0.1);
for i = 1:num_sims
    res   = ress(i);
    prob  = res.SafeProb.extractTimetable;
    dist  = res.ProbDist.extractTimetable;
    state = res.State.extractTimetable;
    traj  = res.Trajectory.extractTimetable;
    etime = res.ElapsedTime.extractTimetable;

    len = length(prob.Data);
    PROB( i, 1:len) = prob.Data;
    SPEED(i, 1:len) = state.Data(:,1);
    TRAJ( i, 1:len, :) = traj.Data;
    STATE( i, 1:len, :) = state.Data;
    ETIME( i, 1:len) = etime.Data;
end

if saveData
    % Format parameters for filename (replace '.' with 'p' to avoid issues)
    % Assuming prior_ic is a 1x2 vector
    p1_str = strrep(num2str(prior_ic(1)), '.', 'p');
    p2_str = strrep(num2str(prior_ic(2)), '.', 'p');
    mes_var_str = strrep(num2str(mes_var), '.', 'p');
    emax_str = strrep(num2str(emax), '.', 'p');
    
    % Create the dynamic parameter string
    param_str = sprintf('_prior_%s_%s_mesvar_%s_emax_%s', p1_str, p2_str, mes_var_str, emax_str);
    
    % 1. AMPC filename
    file_ampc = ['data_mpc/data_AMPC_multi_', condition_str, '_H10', param_str, '.mat'];
    save(file_ampc, 'MU', 'PROB', 'SPEED', 'TRAJ', 'STATE', 'ETIME');
    % fprintf('Data is saved at %s\n', file_ampc);
    fprintf('AMPC = load("%s");\n', file_ampc);

end


save_system(mdl,[],'OverwriteIfChangedOnDisk',true)

%% CDBF

%%%  Path to model %%%%%%%%%%%
addpath impl_controller
addpath impl_model
addpath impl_estimator
addpath impl_road
mdl = 'mdl_closed_loop_mpc'; % MPC with code generaion
load_system(mdl)

% Online or fixed estimation
ONLINE_ESTIMATION = '1';
FIXED_ESTIMATION  = '0';
set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION) % Online Estimation
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
set_param([mdl '/prior'],'InitialCondition', mat2str(prior_ic))               % uses input prior_ic
set_param([mdl '/mes_var'], 'Value', num2str(mes_var))                        % uses input mes_var

% Lane error tolerance
set_param([mdl '/SafeProbabilityMC'],'emax',num2str(emax)) % default is 5

% Num MC sims for safety probability calculation
set_param([mdl '/SafeProbabilityMC'],'snum','100') % 100 samples

% Visulization
set_param([mdl '/visualization'],'Commented','on') % No visualization

% Termination condition
TERM_DIST = '150'; % '127.2'
set_param([mdl '/termination_dist'], 'Value', TERM_DIST)

TERM_LAT_ERROR  = '100'; %30
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
nlobj.PredictionHorizon = 10;  % Horizon
nlobj.ControlHorizon = 2;
% state equation and parameters
nlobj.Model.StateFcn =  "fun_system_dynamics";
nlobj.Model.OutputFcn = "fun_output_function";
nlobj.Model.NumberOfParameters = 2; 
% weights of objective function
nlobj.Weights.OutputVariables = [0.05 1 1]; % [Vx,e,psi] 
nlobj.Weights.ManipulatedVariablesRate = [1 1];
% constraint
%nlobj.Optimization.CustomIneqConFcn = [];
%nlobj.Optimization.CustomIneqConFcn = "fun_inequality";   % PSC (proposed)
nlobj.Optimization.CustomIneqConFcn = "fun_inequality_CDBF";  % CDBF

% validation for codegeneration
u0 = [0 0];
ref0 = [40*1000/3600 0 0];
mu = 0.9;
probs = [1, 0, 1, 1, 0]; % [P, LfP, LgP, BP] for testing
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
    cfg = coder.config('mex');
    %Cfg.DynamicMemoryAllocation = 'off'; %% Deprecated in Matlab2025a
    cfg.EnableDynamicMemoryAllocation = true;     %% After Matlab2025a
    cfg.DynamicMemoryAllocationThreshold = 65536; %% After Matlab2025a
    codegen('-config',cfg,func,'-o',funcOutput,'-args',...
        {coder.Constant(coreData), xk', mv, onlineData});
end

assignin('base','coreData',coreData);
assignin('base','onlineData',onlineData);

save_system(mdl,[],'OverwriteIfChangedOnDisk',true)


% Simulink objects for parllel computation
num_sims = 20;
MU  = NaN(num_sims,1); 
for i = 1:num_sims
    in(i) = Simulink.SimulationInput(mdl);

    MU(i) = mu_gt + rand*0.1;
    in(i) = setBlockParameter(in(i), [mdl '/true_friction_coeff'], Value=num2str(MU(i)) );
end

% Pallarell simulations
num_pools = 20;
poolobj = gcp("nocreate"); % If no pool, do not create new one.
if isempty(poolobj)
    parpool(num_pools)
end
ress = parsim(in, 'ShowSimulationManager', 'off', 'TransferBaseWorkspaceVariables','on');

% Store data
PROB  = NaN(num_sims, 40/0.1);
SPEED = NaN(num_sims, 40/0.1);
TRAJ  = NaN(num_sims, 40/0.1, 3);
STATE = NaN(num_sims, 40/0.1, 12);
ETIME = NaN(num_sims, 40/0.1);
for i = 1:num_sims
    res   = ress(i);
    prob  = res.SafeProb.extractTimetable;
    dist  = res.ProbDist.extractTimetable;
    state = res.State.extractTimetable;
    traj  = res.Trajectory.extractTimetable;
    etime = res.ElapsedTime.extractTimetable;

    len = length(prob.Data);
    PROB( i, 1:len) = prob.Data;
    SPEED(i, 1:len) = state.Data(:,1);
    TRAJ( i, 1:len, :) = traj.Data;
    STATE( i, 1:len, :) = state.Data;
    ETIME( i, 1:len) = etime.Data;
end


if saveData
    % Format parameters for filename (replace '.' with 'p' to avoid issues)
    % Assuming prior_ic is a 1x2 vector
    p1_str = strrep(num2str(prior_ic(1)), '.', 'p');
    p2_str = strrep(num2str(prior_ic(2)), '.', 'p');
    mes_var_str = strrep(num2str(mes_var), '.', 'p');
    emax_str = strrep(num2str(emax), '.', 'p');
    
    % Create the dynamic parameter string
    param_str = sprintf('_prior_%s_%s_mesvar_%s_emax_%s', p1_str, p2_str, mes_var_str, emax_str);

    % 2. CDBF filename
    file_cdbf = ['data_mpc/data_CDBF_multi_', condition_str, '_H10', param_str, '.mat'];
    save(file_cdbf, 'MU', 'PROB', 'SPEED', 'TRAJ', 'STATE', 'ETIME');
    % fprintf('Data is saved at %s\n', file_cdbf);
    fprintf('CDBF = load("%s");\n', file_cdbf);

end


save_system(mdl,[],'OverwriteIfChangedOnDisk',true)

%% APSC

%%%  Path to model %%%%%%%%%%%
addpath impl_controller
addpath impl_model
addpath impl_estimator
addpath impl_road
mdl = 'mdl_closed_loop_mpc'; % MPC with code generaion
load_system(mdl)


% Online or fixed estimation
ONLINE_ESTIMATION = '1';
FIXED_ESTIMATION  = '0';
set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION) % Online Estimation
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
set_param([mdl '/prior'],'InitialCondition', mat2str(prior_ic))               % uses input prior_ic
set_param([mdl '/mes_var'], 'Value', num2str(mes_var))                        % uses input mes_var

% Lane error tolerance
set_param([mdl '/SafeProbabilityMC'],'emax',num2str(emax)) % default is 5

% Num MC sims for safety probability calculation
set_param([mdl '/SafeProbabilityMC'],'snum','100') % 100 samples

% Visulization
set_param([mdl '/visualization'],'Commented','on') % No visualization

% Termination condition
TERM_DIST = '150'; % '127.2'
set_param([mdl '/termination_dist'], 'Value', TERM_DIST)

TERM_LAT_ERROR  = '100'; %30
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
nlobj.PredictionHorizon = 10;  % Horizon
nlobj.ControlHorizon = 2;
% state equation and parameters
nlobj.Model.StateFcn =  "fun_system_dynamics";
nlobj.Model.OutputFcn = "fun_output_function";
nlobj.Model.NumberOfParameters = 2; 
% weights of objective function
nlobj.Weights.OutputVariables = [0.05 1 1]; % [Vx,e,psi] 
nlobj.Weights.ManipulatedVariablesRate = [1 1];
% constraint
%nlobj.Optimization.CustomIneqConFcn = [];
nlobj.Optimization.CustomIneqConFcn = "fun_inequality";   % PSC (proposed)
%nlobj.Optimization.CustomIneqConFcn = "fun_inequality_CDBF";  % CDBF

% validation for codegeneration
u0 = [0 0];
ref0 = [40*1000/3600 0 0];
mu = 0.9;
probs = [1, 0, 1, 1, 0]; % [P, LfP, LgP, BP] for testing
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
    cfg = coder.config('mex');
    %Cfg.DynamicMemoryAllocation = 'off'; %% Deprecated in Matlab2025a
    cfg.EnableDynamicMemoryAllocation = true;     %% After Matlab2025a
    cfg.DynamicMemoryAllocationThreshold = 65536; %% After Matlab2025a
    codegen('-config',cfg,func,'-o',funcOutput,'-args',...
        {coder.Constant(coreData), xk', mv, onlineData});
end

assignin('base','coreData',coreData);
assignin('base','onlineData',onlineData);

save_system(mdl,[],'OverwriteIfChangedOnDisk',true)


% Simulink objects for parllel computation
num_sims = 20;
MU  = NaN(num_sims,1); 
for i = 1:num_sims
    in(i) = Simulink.SimulationInput(mdl);

    MU(i) = mu_gt + rand*0.1;
    in(i) = setBlockParameter(in(i), [mdl '/true_friction_coeff'], Value=num2str(MU(i)) );
end

% Pallarell simulations
num_pools = 20;
poolobj = gcp("nocreate"); % If no pool, do not create new one.
if isempty(poolobj)
    parpool(num_pools)
end
ress = parsim(in, 'ShowSimulationManager', 'off', 'TransferBaseWorkspaceVariables','on');

% Store data
PROB  = NaN(num_sims, 40/0.1);
SPEED = NaN(num_sims, 40/0.1);
TRAJ  = NaN(num_sims, 40/0.1, 3);
STATE = NaN(num_sims, 40/0.1, 12);
ETIME = NaN(num_sims, 40/0.1);
for i = 1:num_sims
    res   = ress(i);
    prob  = res.SafeProb.extractTimetable;
    dist  = res.ProbDist.extractTimetable;
    state = res.State.extractTimetable;
    traj  = res.Trajectory.extractTimetable;
    etime = res.ElapsedTime.extractTimetable;

    len = length(prob.Data);
    PROB( i, 1:len) = prob.Data;
    SPEED(i, 1:len) = state.Data(:,1);
    TRAJ( i, 1:len, :) = traj.Data;
    STATE( i, 1:len, :) = state.Data;
    ETIME( i, 1:len) = etime.Data;
end


if saveData
    % Format parameters for filename (replace '.' with 'p' to avoid issues)
    % Assuming prior_ic is a 1x2 vector
    p1_str = strrep(num2str(prior_ic(1)), '.', 'p');
    p2_str = strrep(num2str(prior_ic(2)), '.', 'p');
    mes_var_str = strrep(num2str(mes_var), '.', 'p');
    emax_str = strrep(num2str(emax), '.', 'p');
    
    % Create the dynamic parameter string
    param_str = sprintf('_prior_%s_%s_mesvar_%s_emax_%s', p1_str, p2_str, mes_var_str, emax_str); 

    % 3. APSC filename
    file_apsc = ['data_mpc/data_APSC_multi_', condition_str, '_H10', param_str, '.mat'];
    save(file_apsc, 'MU', 'PROB', 'SPEED', 'TRAJ', 'STATE', 'ETIME');
    % fprintf('Data is saved at %s\n', file_apsc);
    fprintf('APSC = load("%s");\n', file_apsc);
end


save_system(mdl,[],'OverwriteIfChangedOnDisk',true)
