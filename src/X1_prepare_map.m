%============================================================================
% MPC-Autonomous-Ship-Navigation
% Primož Potočnik (2025)
%----------------------------------------------------------------------------
% Prepare map (land, coast, sea)
%============================================================================
clear all, clc, format compact, close all

% settings
X0_settings


%% Load GSHHG data

% Original data 'gshhg-bin-2.3.7.zip' were downloaded from:
%    https://www.soest.hawaii.edu/pwessel/gshhg/
% Processed in MATLAB with:
%    S = gshhs(filename, W.geolimits(1,:), W.geolimits(2,:));
% And saved as:
%    'data/gshhs_f.b.mat'
%---------
% If you want to run simulation in your area, use the commands above to process
% map data accordingly and save 'S' to file 'P.file_GSHHG'

% Import processed GSHHG Data for Adriatic area
filename = [P.folder_data P.file_GSHHG];
load([filename '.mat']); % load GSHHG data


%% Extract coastline

% Extract GSHHS Level 1 (exterior coastlines of continents and oceanic islands)
levels = [S.Level];
L1 = S(levels == 1);

% Cutout of the first polygon (for speedup)
i = W.icoast;
L1(1).Lat = [L1(1).Lat(i) L1(1).Lat(i(end)) L1(1).Lat(i(1)) nan]; % format as polygon
L1(1).Lon = [L1(1).Lon(i) L1(1).Lon(i(1))   L1(1).Lon(i(1)) nan]; % format as polygon


%% Prepare sparse map1 (hi-res)

% Plot base map
plot_geomap;
title('map1')
pause(.1)

% Grid boundaries (adapted to path)
[yy,xx] = geolimits;
dx = W.dx_dy_mpc; % fine grid
dy = W.dx_dy_mpc; % fine grid

% Create coordinate grid for land: LAND
[lonGrid, latGrid] = meshgrid(xx(1):dx:xx(2), yy(1):dy:yy(2));
map_size = size(lonGrid);
grid_map = zeros(map_size); % All points are sea (0)
LAND = grid_map; % Initialization (all 0)

% Draw border (to prevent planners from escaping)
LAND(1,:)   = 2;
LAND(end,:) = 2;
LAND(:,1)   = 2;
LAND(:,end) = 2;

% Check if point is within land
for n = 1:length(L1)
  isLand = inpolygon(lonGrid, latGrid, L1(n).Lon, L1(n).Lat); % check if on land
  LAND = LAND + isLand; % combine land areas
end
LAND(find(LAND >= 1)) = 2; % land=2

% Create coastal belt by shifting land
COAST = 0 * LAND;
for dim = 1:2
  for premik = -W.nearcoast : W.nearcoast % number of steps left and right to form coastal belt
    COAST = COAST + circshift(LAND,premik,dim);
  end
end
COAST(find(COAST)) = 1; % coast=1

% Combine land=2 and coast=1
COAST(find(LAND)) = 2;
% Save final result in LAND
LAND = COAST;

% Separate matrices: land=2 | coast=1
iLand  = find(LAND==2); % land=2
iCoast = find(LAND==1); % coast=1
land  = 0 * LAND; 
coast = 0 * LAND; 
land(iLand)   = 2; % land only
coast(iCoast) = 1; % coast only

%----------- CONVERT TO SPARSE -----------
sland  = sparse(land);
scoast = sparse(coast);
% Save sparse results
map1.land  = sland;  % 2=land
map1.coast = scoast; % 1=coast
map1.lon   = lonGrid;
map1.lat   = latGrid;

% Display grid of land and coast
iLand  = find(map1.land);  % land=2
iCoast = find(map1.coast); % coast=1
plot(map1.lat(iLand),  map1.lon(iLand),  'x','MarkerSize',1,'Color',W.color_land)
plot(map1.lat(iCoast), map1.lon(iCoast), 'x','MarkerSize',1,'Color',W.color_coast)
pause(.1)



%% Prepare sparse map2 (lo-res)

% Plot base map
figure
plot_geomap;
title('map2')
pause(.1)

% Grid boundaries (adapted to path)
[yy,xx] = geolimits;
dx = W.dx_dy_planner; % coarse grid
dy = W.dx_dy_planner; % coarse grid

% Create coordinate grid for land: LAND
[lonGrid, latGrid] = meshgrid(xx(1):dx:xx(2), yy(1):dy:yy(2));
map_size = size(lonGrid);
grid_map = zeros(map_size); % All points are sea (0)
LAND = grid_map; % Initialization (all 0)

% Draw border (to prevent planners from escaping)
LAND(1,:)   = 2;
LAND(end,:) = 2;
LAND(:,1)   = 2;
LAND(:,end) = 2;

% Check if point is within land
for n = 1:length(L1)
  isLand = inpolygon(lonGrid, latGrid, L1(n).Lon, L1(n).Lat); % check if on land
  LAND = LAND + isLand; % combine land areas
end
LAND(find(LAND >= 1)) = 2; % land=2

% Create coastal belt by shifting land
COAST = 0 * LAND;
for dim = 1:2
  for premik = -W.nearcoast : W.nearcoast % number of steps left and right to form coastal belt
    COAST = COAST + circshift(LAND,premik,dim);
  end
end
COAST(find(COAST)) = 1; % coast=1

% Combine land=2 and coast=1
COAST(find(LAND)) = 2;
% Save final result in LAND
LAND = COAST;

% Separate matrices: land=2 | coast=1
iLand  = find(LAND==2); % land=2
iCoast = find(LAND==1); % coast=1
land  = 0 * LAND; 
coast = 0 * LAND; 
land(iLand)   = 2; % land only
coast(iCoast) = 1; % coast only

%----------- CONVERT TO SPARSE -----------
sland  = sparse(land);
scoast = sparse(coast);
% Save sparse results
map2.land  = sland;  % 2=land
map2.coast = scoast; % 1=coast
map2.lon   = lonGrid;
map2.lat   = latGrid;

% Display grid of land and coast
iLand  = find(map2.land);  % land=2
iCoast = find(map2.coast); % coast=1
plot(map2.lat(iLand),  map2.lon(iLand),  'x','MarkerSize',1,'Color',W.color_land)
plot(map2.lat(iCoast), map2.lon(iCoast), 'x','MarkerSize',1,'Color',W.color_coast)
pause(.1)



%% Save results (for further analysis and simulations)

% Save relevant variables
filename = [P.folder_data P.file_map];
save(filename, 'L1','map1','map2')

% Display
whos map1* map2*
map1, map2



return
%% ########################## Load and visualize saved results ############################
clear, clc, close all

% settings
X0_settings

% Load map data
filename = [P.folder_data P.file_map];
load(filename)

% Plot base map1
map = map1;
plot_geomap;
title('map1')
% Plot coast | land
iL = find(map.land);  % land=2
iC = find(map.coast); % coast=1
plot(map.lat(iL), map.lon(iL), 'x','MarkerSize',1,'Color',W.color_land)
plot(map.lat(iC), map.lon(iC), 'x','MarkerSize',1,'Color',W.color_coast)

% Plot base map2
map = map2;
figure
plot_geomap;
title('map2')
% Plot coast | land
iL = find(map.land);  % land=2
iC = find(map.coast); % coast=1
plot(map.lat(iL), map.lon(iL), 'x','MarkerSize',1,'Color',W.color_land)
plot(map.lat(iC), map.lon(iC), 'x','MarkerSize',1,'Color',W.color_coast)