%============================================================================
% MPC-Autonomous-Ship-Navigation
% Primož Potočnik (2025)
%----------------------------------------------------------------------------
% A script for plotting the basic topological map in 'uifigure' format (for simulation)
%============================================================================
global gx

% Create a UI figure
fig = uifigure('Name', 'Smart Ship Simulation');
fig.Position = [150 90 1720 961];

% Create a GeographicAxes inside the figure
gx = geoaxes(fig);
gx.FontSize = 8;
gx.Position = [0.04 .04 .95 .93];
hold(gx,'on')

% Plot map frame
geolimits(gx,W.geolimits(1,:),W.geolimits(2,:));
geobasemap(gx, 'streets-light');  % streets-light | topographic

% plot coastline
try
  for n = 1:length(L1)
    geoplot(gx,L1(n).Lat, L1(n).Lon,'Color',[.4 .4 1]);
  end
  pause(.1)
end

% plot route
try
  geoplot(gx,route.lat, route.lon, 'ms-');
  for n = 1:size(W.wp,1)
    geoplot(gx,W.wp(n,1),W.wp(n,2),'mo','LineWidth',2); % plot waypoints
  end
end


%% compass, uitable, buttons 

% Compassplot
pax = polaraxes(fig);  % attach polaraxes to the uifigure
pax.Position = [0.47 0.812 0.13 0.13]; % adjust size within figure
pax.ThetaZeroLocation = 'top'; 
pax.ThetaDir   = 'clockwise';
pax.FontSize   = 7;
pax.RTickLabel = '';

% Status uitable
Mode1 = 'Route following'; 
Mode2 = 'MPC';
status.mode    = Mode1;
status.colreg  = '--';
columnNames = {'Navigation Mode','COLREG'};
status.uitable = uitable('Parent', fig, ...
    'Data', cell(0,length(columnNames)), ... 
    'ColumnName', columnNames, ...
    'FontSize', 10,...
    'Position', [1010 703 620 217], ...
    'ColumnWidth',{115,'auto'} );

% Buttons
pauseBtn = uicontrol(fig,'Style','togglebutton','String','Pause','Position',[1634 858 60 30]); % PAUSE button
stepBtn  = uicontrol(fig,'Style','togglebutton','String','Step', 'Position',[1634 826 60 30]); % STEP button
quitBtn  = uicontrol(fig,'Style','togglebutton','String','Quit', 'Position',[1634 890 60 30]); % QUIT button
quitBtn.ForegroundColor='r';

% Title
pnl = uipanel(fig, 'Position',[1360 620 270 80], 'BackgroundColor',[0.94 0.94 0.94]);
text = ['<html>' ...
        '<b>Autonomous Ship Navigation</b><br>' ...
        '1) Automated Route Planning<br>' ...
        '2) Compliance with COLREGs<br>' ...
        '3) Model Predictive Control for Collision Avoidance<br>' ...
        'MATLAB Simulation by Primož Potočnik (2025)' ...
        '</html>'];
ti = uilabel(pnl, 'Text',text, 'Position',[5 0 255 80], 'FontSize',11, 'Interpreter', 'html');
