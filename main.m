% main.m
% Main entry point for autonomous ship navigation simulation

%% MAP
% Simulation map is already prepared in 'data/#X1_prepare_map.mat'
% If you wish to define your own map:
%   1. adjust map settings in 'src/X0_settings.m'
%   2. run 'src/X1_prepare_map.m'

%% ROUTE
% Route is alredy defined in 'data/#X2_route_planner.mat'
% If you wish to define your own route:
%   1. adjust route settings in 'src/X0_settings.m'
%   2. run 'src/X2_route_planner.m'

%% SIMULATION
% Run the simulation
%   1. adjust simulation settings in 'src/X0_settings.m'
%   2. run 'src/X3_mpc_navigation.m'
run src/X3_mpc_navigation 
