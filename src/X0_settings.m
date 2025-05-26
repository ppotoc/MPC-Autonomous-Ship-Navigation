%============================================================================
% MPC-Autonomous-Ship-Navigation
% Primož Potočnik (2025)
%----------------------------------------------------------------------------
% Settings
%============================================================================

%% Files and folders

P.folder_data     = '../data/';           % data folder
P.folder_results  = '../results/';        % results folder
P.file_GSHHG      = 'gshhs_f.b';          % GSHHG data
P.file_map        = 'gshhs_maps.mat';     % maps
P.file_route      = 'route.mat';          % route
P.file_myshipicon = 'ship-icon-blue.ico'; % icon for plotting own ship
P.file_shipicon   = 'ship-icon-red.ico';  % icon for plotting other ships


%% MPC Navigation

%-------- video output ----------------------------
P.save_video     = 1;
P.save_frameRate = 8;
P.save_quality   = 95;

%-------- Simulation parameters --------------------
P.sim_time    = 3000; % max simulation time (if goal not reached earlier)
P.num_ships   = 80;   % number of ships in simulator
P.ship_size   = 12;   % ship icon size (relative)
P.ship_speed  = 3e-3; % ship speed (relative)

%-------- MPC parameters ---------------------------
P.bump_zone         = 1000; % [m]
P.safe_zone         = 5000; % [m]
P.mpc_horizon       = 16;
P.mpc_tirnic        = 45;   % number of trajectories in each MPC iteration
P.max_turn_angle    = 20 * 2*pi/360; % [rad]
P.plot_trajectories = 2;    % 0= no_plot, 1=plot_trajectories

% colors for trajectory plotting
P.colorOK  = [.3 .7 .5]; % dark green
P.colorNOK = [.8 .3 .3]; % dark red


%% Maps and Planner

% Selection of polygon indices when extracting GSHHG data
W.icoast  = 650250 : 653400; % for Adriatic area Split-Vis-Hvar

% Map section for display
W.geolimits  = [43.00  43.56;  15.96  17.33]; % Split-Vis-Hvar

%----- Route start/stop points
W.Split     = [43.49  16.43];
W.Vis       = [43.09  16.21];
W.Sucuraj   = [43.12  17.21];
W.Dubrovnik = [42.63  18.05];

% Compose waypoints into desired route
W.wp = [W.Split; W.Vis; W.Sucuraj; W.Split];

% Map resolution
W.dx_dy_mpc     = 0.002; % for MPC
W.dx_dy_planner = 0.004; % for planner (faster)

% coastline grid shift to cover nearshore zone (number of points to shift)
W.nearcoast = 2;

% colors for land and coast grid visualization
W.color_land  = [ 1 .7 .3];
W.color_coast = [.4 .4 .7];

%----- PLANNER name
W.planner  = 'theta_star'; % BEST
%W.planner = 'a_star';
%W.planner = 'd_star';
%W.planner = 'gbfs';
%W.planner = 'jps';
%W.planner = 'lazy_theta_star'; 