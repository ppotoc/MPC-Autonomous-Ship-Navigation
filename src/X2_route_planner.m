%============================================================================
% MPC-Autonomous-Ship-Navigation
% Primož Potočnik (2025)
%----------------------------------------------------------------------------
% Route planner
%============================================================================
clear all, clc, format compact, close all

% settings
X0_settings

% add planner functions
addpath(genpath('path_planning'));

% load saved maps
load([P.folder_data P.file_map]);

% choose and plot map (which is used by planner)
map  = map2; % map1 | map2
plot_geomap;
title('map2')
% plot coast | land
iL = find(map.land);  % land=2
iC = find(map.coast); % coast=1
plot(map.lat(iL), map.lon(iL), 'x','MarkerSize',1,'Color',W.color_land)
plot(map.lat(iC), map.lon(iC), 'x','MarkerSize',1,'Color',W.color_coast)

% Plot waypoints
for n = 1:size(W.wp,1)
  geoplot(W.wp(n,1),W.wp(n,2),'mo','LineWidth',2); % plot points
end
pause(.1)


%% PATH PLANNER

% choose planner
planner = str2func(W.planner);
disp(planner)

% Create an occupancy map
occupancyMap = 0 * map.coast; % inicialization
occupancyMap(find(map.coast + map.land)) = 2; % planer should avoid coastal area

% Limits of map grid
[yy,xx] = geolimits;
dx = W.dx_dy_planner;
dy = W.dx_dy_planner;

% Initialize the route
path_lat = [];
path_lon = [];

% Compose route segments
for n = 1:size(W.wp,1)-1
  % Translate start & goal into grid index (for planner)
  startPos = [round((W.wp(n,  1) - yy(1)) / dy) + 1, round((W.wp(n,  2) - xx(1)) / dx) + 1];
  goalPos  = [round((W.wp(n+1,1) - yy(1)) / dy) + 1, round((W.wp(n+1,2) - xx(1)) / dx) + 1];

  % run planner
  [bestpath, flag, cost, expand] = planner(occupancyMap, goalPos, startPos); % swap start|goal for correct orientation
  
  % index to geo-coordinates
  path_lon_n = xx(1) + (bestpath(:,2)-1) * dx;
  path_lat_n = yy(1) + (bestpath(:,1)-1) * dy;

  % plot optimal path segment
  geoplot(path_lat_n, path_lon_n, 'ms-'); % PLOT OPTIMAL PATH
  pause(.1)

  % composeroute
  path_lat = [path_lat; path_lat_n];
  path_lon = [path_lon; path_lon_n];
end


%% Simplify route (reduce number of waypoints)

tolerance = 0.004;
path_simple = reducepoly([path_lat,path_lon], tolerance);
path_lat = path_simple(:,1);
path_lon = path_simple(:,2);

% plot reduced route
geoplot(path_lat, path_lon, 'ks-');


%% Save results (to be used in a simulation)

% route
route.lon = path_lon;
route.lat = path_lat;
route.current_target_idx = 2; % initial path index (next waypoint)
route.next_target_x = route.lon(route.current_target_idx);
route.next_target_y = route.lat(route.current_target_idx);

% Save relevant variables
file = [P.folder_data P.file_route];
save(file, 'route')


return
%% ########################## Load and visualize saved results ############################
clear, clc, close all

% settings
X0_settings

% load map, route
load([P.folder_data P.file_map]);
load([P.folder_data P.file_route]);

% plot base map
plot_geomap;

% choose map
map = map2; 
% plot coast
iC = find(map.coast); % obala=1
plot(map.lat(iC), map.lon(iC), 'x','MarkerSize',1,'Color',W.color_coast)
