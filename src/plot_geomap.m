%============================================================================
% MPC-Autonomous-Ship-Navigation
% Primož Potočnik (2025)
%----------------------------------------------------------------------------
% A script for plotting the basic topological map.
%============================================================================
  
% Plot map frame
geolimits(W.geolimits(1,:),W.geolimits(2,:));
geobasemap streets-light  % streets-light | topographic

fig = gcf;
fig.ToolBar = 'none';
fig.MenuBar = 'none';
fig.Position = [150 90 1720 960];

ax = gca;
ax.Position = [0.04 .04 .95 .93];
ax.FontSize = 8;
hold on
pause(.1)

% plot coastline
try
  for n = 1:length(L1)
    geoplot(L1(n).Lat, L1(n).Lon,'Color',[.2 .2 1]);
  end
  pause(.1)
end

% plot route
try
  geoplot(route.lat, route.lon, 'ms-');
  for n = 1:size(W.wp,1)
    plot(W.wp(n,1),W.wp(n,2),'mo','LineWidth',2); % plot waypoints
  end
end
pause(.1)

clear ax fig n 