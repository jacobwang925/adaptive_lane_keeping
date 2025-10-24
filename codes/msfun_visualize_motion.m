function msfun_visualize_motion(block)
% Level-2 MATLAB S-Function for visualizing vehicle motion

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C MEX counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 0;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions        = 2;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;



% Override output port properties
% block.OutputPort(1).Dimensions       = 3;
% block.OutputPort(1).DatatypeID  = 0; % double
% block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0.2 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

block.AllowSignalsWithMoreThan2D = true;

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C MEX counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
    block.NumDworks = 4;

    block.Dwork(1).Name            = 'i';
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;      % double
    block.Dwork(1).Complexity      = 'Real'; % real
    block.Dwork(1).UsedAsDiscState = true;

    td   = block.SampleTimes(1);
    tstr = block.SampleTimes(2);
    tend = str2double( get_param(gcs,"StopTime") );
    tnum = ceil( (tend-tstr)/td );

    block.Dwork(2).Name            = 'trajectory';
    block.Dwork(2).Dimensions      = tnum * 3 ;  % [x, y, psi]
    block.Dwork(2).DatatypeID      = 0;      % double
    block.Dwork(2).Complexity      = 'Real'; % real
    block.Dwork(2).UsedAsDiscState = true;

    block.Dwork(3).Name            = 'road_left';
    block.Dwork(3).Dimensions      = tnum * 2 ;  % [x, y]
    block.Dwork(3).DatatypeID      = 0;      % double
    block.Dwork(3).Complexity      = 'Real'; % real
    block.Dwork(3).UsedAsDiscState = true;

    block.Dwork(4).Name            = 'road_right';
    block.Dwork(4).Dimensions      = tnum * 2 ;  % [x, y]
    block.Dwork(4).DatatypeID      = 0;      % double
    block.Dwork(4).Complexity      = 'Real'; % real
    block.Dwork(4).UsedAsDiscState = true;


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(~)

%end InitializeConditions

%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C MEX counterpart: mdlStart
%%
function Start(block)

    block.Dwork(1).Data = 1;  % time index
    block.Dwork(2).Data = zeros(block.Dwork(2).Dimensions, 1); % trajectory for [x, y, psi]
    block.Dwork(3).Data = zeros(block.Dwork(3).Dimensions, 1); % road left boundary
    block.Dwork(4).Data = zeros(block.Dwork(4).Dimensions, 1); % road right boundary

    LocalInitializeFigure();

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
function Outputs(~)

%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlUpdate
%%
function Update(block)

    % states and inputs
    t     = block.CurrentTime;
    i     = block.Dwork(1).Data;
    tras  = reshape( block.Dwork(2).Data, [block.Dwork(2).Dimensions/3, 3] );
    left  = reshape( block.Dwork(3).Data, [block.Dwork(3).Dimensions/2, 2] );
    right = reshape( block.Dwork(4).Data, [block.Dwork(4).Dimensions/2, 2] );

    % current state and road boundary
    x = block.InputPort(1).Data;
    e = block.InputPort(2).Data;

    Psi_rd = x(3) - e(2);
    XY_rd  = [x(1) x(2)] + [-e(1)*sin(Psi_rd) e(1)*cos(Psi_rd)]; 
    Wrd = 5; % width of the road
    xlim_left  = XY_rd + [-Wrd*sin(Psi_rd) Wrd*cos(Psi_rd)];
    xlim_right = XY_rd + [Wrd*sin(Psi_rd) -Wrd*cos(Psi_rd)];

    % store current x to tras
    tras(i,:) = x;
    block.Dwork(2).Data = reshape(tras, [block.Dwork(2).Dimensions, 1] );  

    left(i,:)  = xlim_left;
    right(i,:) = xlim_right;
    block.Dwork(3).Data = reshape(left,  [block.Dwork(3).Dimensions, 1] );  
    block.Dwork(4).Data = reshape(right, [block.Dwork(4).Dimensions, 1] );  

    fig = get_param(block.BlockHandle,'UserData');
    if ishghandle(fig, 'figure')
      if strcmp(get(fig,'Visible'),'on')
        ud = get(fig,'UserData');
        LocalUpdateFigure(ud,t,x, e);
      end
    end

%end Update

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C MEX counterpart: mdlTerminate
%%
function Terminate(~)

%end Terminate


%
%=============================================================================
% LocalUpdateFigure
% Local function to update the graphics in the animation window.
%=============================================================================
%
function LocalUpdateFigure( ud,time,x, e)

    % plot current position
    posX = x(1);
    posY = x(2);
    psi  = x(3);

    % road boundaries
    lat_err = e(1);
    psi_err = e(2);
    Psi_rd = psi - psi_err;
    XY_rd  = [x(1) x(2)] + [lat_err*sin(Psi_rd) -lat_err*cos(Psi_rd)]; 
    Wrd = 5; % width of the road
    left  = XY_rd + [-Wrd*sin(Psi_rd) Wrd*cos(Psi_rd)];
    right = XY_rd + [Wrd*sin(Psi_rd) -Wrd*cos(Psi_rd)];   

    % vehicel positioning
    set(ud.VehiclePos, 'Shape', LocalCartShape(posX,posY,psi) )
%     set(ud.VehiclePosLane, ...
%             'XData', posX, ...
%             'YData', posY);

    % draw trajectory
    addpoints(ud.VehicleTrs,     posX, posY);
    addpoints(ud.VehicleTrsLane, posX, posY);
    addpoints(ud.RoadLeft,  left(1),  left(2));
    addpoints(ud.RoadRight, right(1), right(2));

    % plot area
    set(ud.AxesH, 'Xlim', [posX-5, posX+5])
    set(ud.AxesH, 'Ylim', [posY-5, posY+5])
    set(ud.AxesH, 'DataAspectRatio', [1, 1, 1])

    % show time
    set(ud.TimeField,...
        'String', [num2str(time,'%.1f') ' s']);
    
    % Force plot to be drawn
    pause(0)
    drawnow

    % end LocalPendSets

%
%=============================================================================
% LocalInitializeFigure
% Local function to initialize the animation.  
% If the animation window already exists, it is brought to the front.  
% Otherwise, a new figure window is created.
%=============================================================================
%
function LocalInitializeFigure()

    TimeClock = 0;
    posX  = 0;
    posY  = 0;
    %
    % The animation figure handle is stored in the pendulum block's UserData.
    % If it exists, initialize the reference mark, time, cart, and pendulum
    % positions/strings/etc.
    %
    Fig = get_param(gcbh,'UserData');
    if ishghandle(Fig ,'figure')

       FigUD = get(Fig,'UserData');

       set(FigUD.VehiclePosLane,...
            'XData',posX,...
            'YData',posY);
        clearpoints(FigUD.VehicleTrs);
        clearpoints(FigUD.VehicleTrsLane);
        clearpoints(FigUD.RoadLeft);
        clearpoints(FigUD.RoadRight);

        figure(Fig);  % bring it to the front

    else
        %
        % the animation figure doesn't exist, create a new one and store its
        % handle in the animation block's UserData
        %
        FigureName = 'Vehicle Motion Visualization';
        Fig = figure(...
            'Units',           'pixel',...
            'Position',        [1000 50 360 640],...
            'Name',            FigureName,...
            'NumberTitle',     'off',...
            'IntegerHandle',   'off',...
            'HandleVisibility','callback',...
            'Resize',          'off',...
            'DeleteFcn',       'pendan([],[],[],''DeleteFigure'')',...
            'CloseRequestFcn', 'pendan([],[],[],''Close'');');
        % Axes for Vehicle Atitude and Tire forces
        AxesVcl = axes(...
            'Parent',  Fig,...
            'Units',   'pixel',...
            'Position',[40 310 300 300],...
            'CLim',    [1 64], ...
            'Xlim',    [-5 5],...
            'Ylim',    [-5 5],...
            'Visible', 'off', ...
            'DataAspectRatio', [1, 1, 1]);
        VehiclePos = plot(...
            LocalCartShape(posX, posY, pi/4), ...
            'Parent',   AxesVcl );
        VehicleTrs = animatedline(...
            'Parent',   AxesVcl, ...
            'Color',    [0 .7 .7]);        
        uicontrol(...
            'Parent',             Fig,...
            'Style',              'text',...
            'Units',              'pixel',...
            'Position',           [80 615 100 20], ...
            'HorizontalAlignment','right',...
            'String',             'Time: ');
        TimeField = uicontrol(...
            'Parent',             Fig,...
            'Style',              'text',...
            'Units',              'pixel', ...
            'Position',           [180 615 100 20],...
            'HorizontalAlignment','left',...
            'String',             [num2str(TimeClock) ' s']);
        % Axes for visualizing trajectory
        AxesLane = axes(...
            'Parent',  Fig,...
            'Units',   'pixel',...
            'Position',[40 20 300 250],...
            'CLim',    [1 64], ...
            'Xlim',    [-5 5],...
            'Ylim',    [-5 5],...
            'Visible', 'on');
        VehiclePosLane = scatter(...
            posX, posY, ...
            'Parent',   AxesLane );
        VehicleTrsLane = animatedline(...
            'Parent',   AxesLane, ...
            'Color',    [0 .7 .7]);        
        RoadLeft = animatedline(...
            'Parent',   AxesLane, ...
            'Color',    [0.1 0.1 0.1]);        
        RoadRight = animatedline(...
            'Parent',   AxesLane, ...
            'Color',    [0.1 0.1 0.1]);       
        
        %
        % all the HG objects are created, store them into the Figure's UserData
        %
        FigUD.AxesH          = AxesVcl;
        FigUD.TimeField      = TimeField;
        FigUD.VehiclePos     = VehiclePos;
        FigUD.VehicleTrs     = VehicleTrs;
        FigUD.VehiclePosLane = VehiclePosLane;
        FigUD.VehicleTrsLane = VehicleTrsLane;
        FigUD.RoadLeft       = RoadLeft;
        FigUD.RoadRight      = RoadRight;
        FigUD.Block        = get_param(gcbh,'Handle');
        set(Fig,'UserData',FigUD);

        drawnow
        set_param(gcbh,'UserData',Fig);

    end

%
%=============================================================================
% LocalCartVertices
% Local function to return polyshape of the cart.  
%=============================================================================
%
function poly = LocalCartShape(x,y,psi)

    Lf = 1.0;   
    Lr = 1.0;
    W  = 1.0; 
    rct  = polyshape([ Lf W/2; Lf -W/2; -Lr -W/2; -Lr W/2 ] + [x y]  );
    poly = rotate(rct, rad2deg(psi), [x,y]);

