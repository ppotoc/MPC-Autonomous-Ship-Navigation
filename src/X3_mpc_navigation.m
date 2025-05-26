%============================================================================
% MPC-Autonomous-Ship-Navigation
% Primož Potočnik (2025)
%----------------------------------------------------------------------------
% MPC simulation
%============================================================================
clc; format compact; clear; 
close(findall(0, 'Type', 'figure')); % close all figures and uifigures

% globals
global P gx map route status 

% nastavitve
X0_settings

% load maps
load([P.folder_data P.file_map]);
% choose map for navigation: map1=hi-res, map2=lo-res
map = map1;

% load route
load([P.folder_data P.file_route]);

% Plot base map
plot_geomap_uifigure;

% Inicializacija ladij na naključnih položajih in hitrostih (prva ladja je naša)
for n = 1:P.num_ships
  % ustvari ladjo
  ships(n) = generate_random_ship(n==1);
  % pozicionira ladje
  ships(n) = move_ship(ships(n));
end



%% ==================================== MAIN LOOP ====================================

% Priprava videa
if P.save_video
  P.video_file = ['../results/#' datestr(now,30) '.mp4'];
  P.videoObj = VideoWriter(P.video_file, 'MPEG-4');
  P.videoObj.FrameRate = P.save_frameRate;
  P.videoObj.Quality   = P.save_quality;
  open(P.videoObj);
end

% določi aktivno ladjo
[ships,i_myship,i_other_ships] = set_my_ship(ships, 1); % ladja '1' je aktivna

% Simualcija
for t = 1:P.sim_time

  % Preveri prisotnost drugih ladij | obale v safe_zone, ali waypointa v bump_zone
  [is_close_max, is_close] = check_collision(ships(i_myship), ships(i_other_ships)); % preveri po vseh ladjah
  ships(i_myship).is_close = is_close_max;

  % ali smo v bump_zone, torej trk?
  if is_close_max == 2
    % trk je neizogiben, zaključi simulacijo
    try 
      writeVideo(P.videoObj, getframe(gx));
      writeVideo(P.videoObj, getframe(gx));
      close(P.videoObj);  % zaključi video, če se je snemal
      delete(P.video_file); % izbriše video, ker shranjujem le ko konca izpeljane poti!!!
    end
    error('COLLISION!')
    return
  end


  % (1)============= MYSHIP : v osnovi sledi planirani ruti
  % get target coordinates
  route.next_target_lon = route.lon(route.current_target_idx);
  route.next_target_lat = route.lat(route.current_target_idx);
  % plot next waypoint
  geoplot(gx,route.next_target_lat,route.next_target_lon,'ks');

  %----- Če ni nobenih ovir, sledi planirani trajektoriji
  if is_close_max==0 % če je varno, sledi trasi
    % status (Route following)
    status.mode   = Mode1;
    status.colreg = '--';
    % izklopi prikaz mpc tirnic
    for i = 1:P.mpc_tirnic
      ships(i_myship).p_traj(i).Visible = 'off';
    end   
    % smer (kot) proti ciljni točki
    theta2target = atan2(route.next_target_lat - ships(i_myship).lat, ...
        (route.next_target_lon - ships(i_myship).lon) * cosd(ships(i_myship).lat));
    % BUG solved: če je bil prej MPC in je smer obratno od WP, bo sprememba smeri prehitra, zato uvedem postopno spremembo
    if abs(wrapToPi(ships(i_myship).theta - theta2target)) <= P.max_turn_angle
      ships(i_myship).theta = theta2target;
    else % sprememba smeri je prevelika, zato postopno zavijam v smer proti WP
      predznak = sign(theta2target - ships(i_myship).theta);
      ships(i_myship).theta = wrapToPi(ships(i_myship).theta + P.max_turn_angle * predznak);
    end
  else
    % status (MPC)
    status.mode   = Mode2;
  end

  %----- Če so v bližini še ladje, vklopi COLREG preverjanje in spremeni smer ladjam, ki se nam morajo izogniti
  if is_close(1)
    % Identify COLREG ships (kdo ima prednost, nato spremeni smer ladjam, ki nimajo prednosti)
    [ships,i_risk_ships] = check_COLREG_rules(ships,i_myship); % po COLREG določi, če se nam morajo kakšne ladje izogniti
  else
	status.colreg = '--'; % ni ladij v bližini, zato ni treba COLREG preverjanja
  end

  %----- Če je v bližini ladja | obala | WP, vklopi MPC navigacijo
  if is_close_max
    % MPC vodenje
    ships(i_myship) = MPC_navigation(ships(i_myship),ships(i_other_ships)); % >>>>>>>>>>>>> MPC NAVIGATION <<<<<<<<<<<<<
  end

  % (3)============= MYSHIP : premik ladje
  % Premik ladje v smeri theta in posodobitev koordinat
  ships(i_myship) = move_ship(ships(i_myship));

  % (4)============= SHIPS : premik ostalih ladij
  % Posodobi pozicije vseh ladij
  for i = i_other_ships
    % posodobi položaj ladje in premakni
    ships(i) = move_ship(ships(i));

    % Preveri, če je ladja ob obali - v tem primeru obrni hitrost, da izpluje na morje
    if is_on_coast(ships(i)) | is_on_land(ships(i)) % NOVO 31.3.2025 dodam 'is_on_land' ker lahko ladje zapeljejo na kopno
      ships(i).theta = wrapToPi(pi + ships(i).theta);
      ships(i) = move_ship(ships(i));
    end
  end

  % Update status table
  update_status;

  % Compass (north orientation) with ship_direction and direction_to_target
  theta2target = atan2(route.next_target_lat - ships(i_myship).lat, ...
      (route.next_target_lon - ships(i_myship).lon) * cosd(ships(i_myship).lat));
  % add pi/2 for north orientation, swap to clockwise
  cp = compassplot(pax, pi/2 - [theta2target ships(i_myship).theta; theta2target ships(i_myship).theta],[1 .94; 1 .94]);
  cp(1).Color = 'm';
  cp(2).Color = 'b';
  pax.ThetaZeroLocation = 'top';
  pax.ThetaDir   = 'clockwise';
  pax.FontSize   = 7;
  pax.RTickLabel = '';

  % Če je ladja blizu trenutne ciljne točke, preklopi na naslednji WAYPOINT
  distance_to_target = haversine(ships(i_myship).lat, ships(i_myship).lon, route.next_target_lat, route.next_target_lon);
  if distance_to_target < P.bump_zone; 
    % nov waypoint
    route.current_target_idx = route.current_target_idx + 1;
    % Ali smo že na cilju?
    if route.current_target_idx > length(route.lon)
      disp('*** Ship arrived to destination ***')
      ships(i_myship).p_safe_circle.Visible = 'off';
      % izklopi prikaz mpc tirnic,
      for i = 1:P.mpc_tirnic
        ships(i_myship).p_traj(i).Visible = 'off';
      end
      % nato zaključi zanko in gre na zajem zadnje slike videa
      break
    end
  end

  % poskrbi za refresh slike
  drawnow, pause(.01)
 
  %----- Buttons -----
  % PAUSE and STEP button
  while get(pauseBtn,'Value')
    set(pauseBtn, 'String', 'Resume');
    pause(.2);
    if get(stepBtn,'Value')
      break
    end
  end
  set(pauseBtn,'String','Pause'); % sprosti gumb
  set(stepBtn, 'String','Step');  % sprosti gumb
  set(stepBtn,'Value',0)
  % QUIT button
  if get(quitBtn,'Value')
    break
  end

  % Zajemi sličico za video
  if P.save_video
    writeVideo(P.videoObj, getframe(gx));
  end
end

% Zapri video datoteko
if P.save_video
  % zajame zadnji frame (2x za prikaz v VLC!!!) in shrani video
  writeVideo(P.videoObj, getframe(gx));
  writeVideo(P.videoObj, getframe(gx));
  close(P.videoObj);
  disp(['Video shranjen kot: ', P.video_file]);
end



%% #################################### Funkcije ########################################

%---------------------- generate_random_ship -----------------------------------------
function ship = generate_random_ship(is_myship)
global P gx map route
    valid = false;
    [rows, cols] = size(map.coast);

    % is myship?
    ship.is_myship = is_myship;
    % default no ships close
    ship.is_close  = false;
    
    while ~valid
      % random koordinate ladje
      randRow = randi(rows);
      randCol = randi(cols);
      % pretvori grid koordinate v latitude/longitude
      ship.lat = map.lat(randRow,randCol);
      ship.lon = map.lon(randRow,randCol);

      % Obdrži, če je naša ladja (ker pozicijo nastavim naknadno), ali če je na morju in izven obalnega pasu
      if is_myship | ~(is_on_land(ship) | is_on_coast(ship))
        valid = true;

        % random smer in hitrost ladje
        ship.theta = rand*pi - pi/2;
        ship.speed = (.25+rand) * P.ship_speed;

        % našo ladjo prestavi na začetno pozicijo
        if is_myship
          ship.lon   = route.lon(1); % začetna lokacija
          ship.lat   = route.lat(1); % začetna lokacija
          ship.theta = deg2rad(210);  % začetni kot (smer)
          ship.speed = P.ship_speed; % hitrost naše ladje
        end

        %---------- ship.shape : oblika ladje za izris -----
        if is_myship
          ic = geoiconchart(gx,43,17,[P.folder_data P.file_myshipicon]); % myship
        else
          ic = geoiconchart(gx,43,17,[P.folder_data P.file_shipicon]);   % other ships
        end
        % move icon
        ic.LongitudeData = ship.lon; % move ship
        ic.LatitudeData  = ship.lat;
        ic.IconRotation  = rad2deg(ship.theta); % rotate ship (degrees)
        ic.SizeData      = P.ship_size; % nastavi velikost ladje
        ship.ic          = ic; % shrani ikono v strukturo ladje

        % inicializacija trajektorij (pretekle in prihodnje), ter bubble za safe_zone in bump_zone
        ship.track_tail  = [];
        ship.track_head  = [];
        ship.tracks_mpc  = nan(P.mpc_tirnic, P.mpc_horizon); % inicializacija tirnic

        %---------- plot head | tail | traj
        % Plot ship head
        ship.p_head = geoplot(gx,[ship.lat ship.lat],[ship.lon ship.lon],'-');
        if is_myship % prikaže smer naše ladje
          ship.p_head.Color = 'b'; % myship
        else
          ship.p_head.Color = [1 .5 .5]; % ships
        end
        % Plot ship tail (only myship)
        if is_myship % prikaže smer naše ladje
          ship.p_tail = geoplot(gx,[ship.lat ship.lat],[ship.lon ship.lon], '.-','color',[.2 .5 .6]);
        else
          ship.p_tail = [];
        end
        % MPC tracks za vse ladje
        ship.p_traj = plot(gx,ship.tracks_mpc','color',P.colorOK);

        %----- plot safe_zone
        % Pretvorba polmera R iz metrov v stopinje
        R_lat = P.safe_zone / 111320;  % 1 stopinja latitude ≈ 111.32 km
        R_lon = P.safe_zone / (111320 * cosd(ship.lat));  % Prilagoditev za longitude
        % Ustvarjanje točk kroga
        alpha = linspace(0, 2*pi, 100);
        lat_circle = ship.lat + R_lat * sin(alpha);
        lon_circle = ship.lon + R_lon * cos(alpha);
        % Prikaz kroga na zemljevidu
        ship.p_safe_circle = geoplot(gx,lat_circle, lon_circle,'Color',[.6 .6 .7]);

        %----- plot bump_zone
        % Pretvorba polmera R iz metrov v stopinje
        R_lat = P.bump_zone / 111320;  % 1 stopinja latitude ≈ 111.32 km
        R_lon = P.bump_zone / (111320 * cosd(ship.lat));  % Prilagoditev za longitude
        % Ustvarjanje točk kroga
        %alpha = linspace(0, 2*pi, 100);
        lat_circle = ship.lat + R_lat * sin(alpha);
        lon_circle = ship.lon + R_lon * cos(alpha);
        % Prikaz kroga na zemljevidu
        ship.p_bump_circle = geoplot(gx,lat_circle, lon_circle,'Color',[.5 .5 .6]);

        % prikaže/skrije safe/bump kroge le za našo ladjo
        ship.p_safe_circle.Visible = is_myship;
        ship.p_bump_circle.Visible = is_myship;

        % ikona ladje prikazana na vrhu (PAZI: zelo počasna operacija, zato enkratni klic, samo naša ladja)
        if is_myship
          uistack(ship.ic, 'top');
        end
      end
    end
end

%---------------------- Funkcija za premik ladje -------------------------------------
% Premakne ladjo iz shranjene pozicije (lat,lon) v smeti (theta) s hitrostjo (speed)
function ship = move_ship(ship)
global P map
   % upgrade position (one step move)
   [ship.lat, ship.lon] = update_ship_position(ship.lat, ship.lon, ship.theta, ship.speed);

   % move ship icon
   ship.ic.LatitudeData  = ship.lat;
   ship.ic.LongitudeData = ship.lon;
   % rotate ship
   ship.ic.IconRotation = rad2deg(ship.theta); % rotate ship (degrees)

   %---------- plot head
   % Predikcija prihodnjih položajev (preprosta linearna predikcija za 'mpc_horizon' korakov naprej)
   lat = ship.lat;
   lon = ship.lon;
   ship.track_head = [lat lon];
   for i = 2:P.mpc_horizon 
     [lat, lon] = update_ship_position(lat,lon,ship.theta,ship.speed);
   end
   ship.track_head(2,:) = [lat lon]; % obdrži le prvo in zadnjo pozicijo
   % Plot ship head
   ship.p_head.LatitudeData  = ship.track_head(:,1);
   ship.p_head.LongitudeData = ship.track_head(:,2);
   if ship.is_myship & ship.is_close==1 % prikaže smer naše ladje samo, če ni vklopljen mpc
     ship.p_head.Visible = 'off';
   else
     ship.p_head.Visible = 'on';
   end

   %--------------- dodatni izrisi za vse ladje (vključujem z 'Visible')
   % tail trajektorija
   ship.track_tail = [ship.track_tail; ship.lat ship.lon];
   ship.p_tail.LatitudeData  = ship.track_tail(:,1);
   ship.p_tail.LongitudeData = ship.track_tail(:,2);
   % move safe_zone
   ship.p_safe_circle.LongitudeData = ship.p_safe_circle.LongitudeData-mean(ship.p_safe_circle.LongitudeData)+ship.lon;
   ship.p_safe_circle.LatitudeData  = ship.p_safe_circle.LatitudeData -mean(ship.p_safe_circle.LatitudeData) +ship.lat;
   % move bump_zone
   ship.p_bump_circle.LongitudeData = ship.p_bump_circle.LongitudeData-mean(ship.p_bump_circle.LongitudeData)+ship.lon;
   ship.p_bump_circle.LatitudeData  = ship.p_bump_circle.LatitudeData -mean(ship.p_bump_circle.LatitudeData) +ship.lat;
   
   % prikaže/skrije safe/bump kroge le za našo ladjo
   ship.p_safe_circle.Visible = ship.is_myship;
   ship.p_bump_circle.Visible = ship.is_myship;
end

%---------------------- Funkcija za premik ladje v geografskih koordinatah --------------
function [lat_next, lon_next] = update_ship_position(lat, lon, theta, speed)
    % Izračun novih koordinat v stopinjah
    lat_next = lat + speed * sin(theta);
    lon_next = lon + speed * cos(theta) / cosd(lat);
end

%---------------------- Funkcija za preverjanje, ali je ladja na kopnem --------------
function on_land = is_on_land(ship)
global map
    % Poiščemo najbližji indeks v matrikah map.lat in map.lon
    [~, row] = min(abs(map.lat(:,1) - ship.lat));
    [~, col] = min(abs(map.lon(1,:) - ship.lon));
    % Preverimo vrednost v map.land
    on_land = (map.land(row, col) > 0);
end

%---------------------- Funkcija za preverjanje, ali je ladja ob obali ----------------
function on_coast = is_on_coast(ship)
global map
    % Poiščemo najbližji indeks v matrikah map.lat in map.lon
    [~, row] = min(abs(map.lat(:,1) - ship.lat));
    [~, col] = min(abs(map.lon(1,:) - ship.lon));
    % Preverimo vrednost v map.land
    on_coast = (map.coast(row, col) > 0);
end

%---------------------- Preverja možnost trkov (ladje, waypoints, obala) -------------
function [is_close_max, is_close] = check_collision(myship, ships)
    global P map route
    is_close = [0 0 0]; % inicializacija: ladje | obala | waypoint

    % 1) Detekcija bližine po vseh ladjah
    for n = 1:length(ships) % po vseh ladjah
        % Izračun Haversinove razdalje
        dist = haversine(myship.lat, myship.lon, ships(n).lat, ships(n).lon);
        if dist <= P.safe_zone, is_close(1) = 1; end % ships in safe_zone
        if dist <= P.bump_zone, is_close(1) = 2; end % ships in bump_zone
    end

    % 2) Detekcija bližine obale in kopnega
    % obala
    iObala = find(map.coast);
    loG = map.lon(iObala);
    laG = map.lat(iObala);
    maG = map.coast(iObala);
    [~, i] = min(abs(myship.lon - loG) + abs(myship.lat - laG));
    dist = haversine(myship.lat, myship.lon, laG(i), loG(i));
    if dist <= P.safe_zone, is_close(2) = 1; end % coast in safe_zone
    
    % 3) Detekcija bližine routing waypointov
    for n = 1:length(route.lon) % po vseh waypointih
      dist = haversine(myship.lat, myship.lon, route.lat(n), route.lon(n));
      if dist <= P.safe_zone, is_close(3) = 1; end % waypoint near
    end

    % vrne največjo možno nevarnost
    is_close_max = max(is_close);
end

% Hitrejša različica za izračun Haversinove razdalje med dvema geografskima točkama
function d = haversine(lat1, lon1, lat2, lon2)
    R = 6371000; % Zemljin polmer v metrih
    % Pretvorba stopinj v radiane
    lat1 = lat1 * (pi / 180);
    lon1 = lon1 * (pi / 180);
    lat2 = lat2 * (pi / 180);
    lon2 = lon2 * (pi / 180);
    % Izračun razlik
    dlat = lat2 - lat1;
    dlon = lon2 - lon1;
    % Haversinova formula
    sin_dlat2 = sin(dlat / 2);
    sin_dlon2 = sin(dlon / 2);
    a = sin_dlat2 .* sin_dlat2 + cos(lat1) .* cos(lat2) .* sin_dlon2 .* sin_dlon2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    % Razdalja v metrih
    d = R * c; 
end

%---------------- Funkcija nastavi, katera ladja je naša (is_myship = true) ----------
function [ships,i_myship,i_other_ships] = set_my_ship(ships, n)
global P
    for i = 1:length(ships)
      ships(i).is_myship = false;
    end
    ships(n).is_myship = true;
    % vrne indekse aktivne ladje in ostalih ladij
    i_myship     = n;
    i_other_ships = find(~[ships.is_myship]);
    
    % Trajectories visible only for myship  
    for i = 1:length(ships)
      for j = 1: P.mpc_tirnic
        if ships(i).is_myship % prikaže mpc tirnice
          ships(i).p_traj(j).Visible = 'on';
        else
          ships(i).p_traj(j).Visible = 'off';
        end
      end
   end
end

%---------------------- Update simulation status -------------------------------------
function [] = update_status
global status
    currentData = status.uitable.Data;
    newData     = {status.mode, status.colreg};
    % izpis podatkov v uitable in scroll na zadnje stanje
    set(status.uitable, 'Data', [currentData; newData]);
    scroll(status.uitable,'bottom');
    %drawnow
end



%% ############################## Funkcija za MPC vodenje ################################

%---------------------- Funkcija za izračun optimalne poti z MPC
function [myship] = MPC_navigation(myship,ships)
    global P gx route map status

    %------------------ kreira mpc tirnice -------------------------
    %
    % možne variacije kota glede na dopustni max_turning_angle
    angles_pahljaca = linspace(-P.max_turn_angle, P.max_turn_angle, P.mpc_tirnic); % pahljača trajektorij

    for iter = 1: P.mpc_tirnic
      % inicializacija tirnice z našo pozicijo
      lat = myship.lat;
      lon = myship.lon;  
      speed = myship.speed;

      % kreira novo tirnico
      theta = myship.theta; % current ship direction
      for h = 1:P.mpc_horizon

        % izmenično dodaja random ali pahljačaste tirnice
        if h==1
          dtheta = angles_pahljaca(iter);
        else
          dtheta = dtheta*.95; % pahljača se zravna proti koncu tirnice
        end
        theta = theta + dtheta;
        
        % sestavlja odseke tirnic
        [lat, lon] = update_ship_position(lat,lon,theta,speed);
        my_lat(iter,h)   = lat;
        my_lon(iter,h)   = lon;
        my_theta(iter,h) = theta;
      end
      % DEBUG plot
      %plot([myship.lat my_lat(iter,:)],[myship.lon my_lon(iter,:)],'k.-');
    end
    % REZULTAT: MCP TIRNICE
    % lon,lat: matrika pozicij MPC tirnic naše ladje (po iteracijah in horizontih)


    %---------- preveri mpc tirnice, če kje trčijo v ladje ---------
    %
    % izračun linearnih trajektorij ladij za horizont predikcije
    for j = 1:length(ships)
      ships_la = ships(j).lat;
      ships_lo = ships(j).lon;
      for h = 1:P.mpc_horizon
        [ships_la, ships_lo] = update_ship_position(ships_la,ships_lo, ships(j).theta, ships(j).speed);
        ships_lat(j,h) = ships_la;
        ships_lon(j,h) = ships_lo;
      end
    end % ships_lat,ships_lon : vektor pozicij drugih ladij

    % Inicializacija matrike za shranjevanje informacij o trkih
    collisions = false(P.mpc_tirnic, P.mpc_horizon);

    % Iteracija skozi vse tirnice naše ladje
    for track = 1:P.mpc_tirnic
      for step = 1:P.mpc_horizon
        % Koordinate naše ladje v trenutnem koraku
        ship_lat = my_lat(track, step);
        ship_lon = my_lon(track, step);
        % Preveri razdaljo do vseh drugih ladij v istem časovnem koraku
        for other_ship = 1:length(ships)
          other_lat = ships_lat(other_ship, step);
          other_lon = ships_lon(other_ship, step);
          % Izračunaj haversine razdaljo
          distance = haversine(ship_lat, ship_lon, other_lat, other_lon); % PAZI, sem imel bug
          % Preveri, ali je razdalja manjša od bump_zone
          if distance < P.bump_zone
            collisions(track, step) = true;
          end
        end
      end
    end
    % zbere trke v skalar za vsako tirnico
    collisions = max(collisions')';
    % REZULTAT: vektor 'collisions', ki označuje tirnice glede na možnost trka

    %---------- preveri mpc tirnice, če kje trčijo v OBALO ---------
    % Inicializacija matrike za shranjevanje informacij o nasedanju
    hit_land = false(P.mpc_tirnic, P.mpc_horizon);
    myship_temp = myship; % inicializacija začasne ladje za preverjanje nasedanja tirnic

    % Iteracija skozi vse tirnice naše ladje
    for track = 1:P.mpc_tirnic
      for step = 1:P.mpc_horizon
        % Koordinate naše ladje v trenutnem koraku
        myship_temp.lon = my_lon(track, step);
        myship_temp.lat = my_lat(track, step);
        % Preveri razdaljo do OBALE in do KOPNEGA
        hit_land(track, step) = is_on_coast(myship_temp) + is_on_land(myship_temp);  % NOVO 26.3.2025
      end
    end
    % zbere trke v skalar za vsako tirnico
    hit_land = max(hit_land')';
    % združi collisions in hit_land
    collisions = collisions | hit_land;
    % REZULTAT: vektor 'collisions', ki označuje tirnice glede na možnost trka in nasedanja

    %------------ Izloči trajektorije z možnostjo trka ali nasedanja -------------
    norisk = find(collisions==0);
    N_norisk = length(norisk); % število tirnic brez rizika trčenja
    % če je trk neizogiben, zaključi simulacijo
    if isempty(norisk) 
      % NOVO 31.3.2025: pred trkom izriše vse tirnice, da se vidi,m da ni nobene prehodne
      for iter = 1: P.mpc_tirnic
        myship.p_traj(iter).LongitudeData = [myship.lon my_lon(iter,:)];
        myship.p_traj(iter).LatitudeData  = [myship.lat my_lat(iter,:)];
        myship.p_traj(iter).Visible = 'on';
        myship.p_traj(iter).Color = P.colorNOK; % barva za NOK tirnice
      end
      try
        writeVideo(P.videoObj, getframe(gx));
        writeVideo(P.videoObj, getframe(gx));
        close(P.videoObj);  % zaključi video, če se je snemal
        delete(P.video_file); % izbriše video, ker shranjujem le ko konca izpeljane poti!!!
      end % zaključi video, če se je snemal
      error('NO FREE TRAJECTORY!')
      return
    end
    % mpc tirnice brez trka
    norisk_lon   = my_lon(norisk,:);
    norisk_lat   = my_lat(norisk,:);
    norisk_theta = my_theta(norisk,:);
    % REZULTAT: mpc tirnice brez trka: norisk_lon, norisk_lat


    %---------------- Izbere najboljšo tirnico ---------------------
    % smer in razdalja proti ciljni točki
    theta2target = atan2(route.next_target_lat - myship.lat, (route.next_target_lon - myship.lon) * cosd(myship.lat));

    % relativni kot naše ladjo proti cilju (next WP)
    relative_theta2target = wrapToPi(myship.theta - theta2target); % zamenjam predznak, NOVO 7.4.2025

    % 1) če je kot ladje do cilja manj kot [90,-90], izbere tirnico s prvim segmentom najbolj usmerjenim proti cilju
    if abs(rad2deg(relative_theta2target)) < 90
      % smer prvega segmenta norisk tirnic
      norisk_angles = norisk_theta(:,1);
      % izbere norisk tirnico, ki ima prvi segment najbolj točno usmerjen proti cilju
      angle_diff = wrapToPi(theta2target - norisk_angles); % 25.3.2025 uredim problem odštevanja kotov
      [~, best_traj_idx] = min(abs(angle_diff));
      
    % 2) če je kot ladje do cilja več kot 90 st., pa izbere tirnico, ki ima zadnjo točko najbližje cilju
    else
      % NOVO 8.4.2025, odpravim bug v naslednjih dveh vrsticah!!!
      % lokacija zadnjega segmenta norisk tirnic
      norisk_lon_end   = norisk_lon(:,end);
      norisk_lat_end   = norisk_lat(:,end);
      % izbere norisk tirnico, ki ima zadnji segment najbližje cilju
      for i = 1:N_norisk
        d(i) = haversine(norisk_lat_end(i), norisk_lon_end(i), route.next_target_lat, route.next_target_lon);
      end
      % izbere tirnico z najbližjo končno točko
      [~, best_traj_idx] = min(d);
    end
    % kot nove (najboljše) tirnice
    best_theta = norisk_theta(best_traj_idx,1);

    %----------- Posodobi smer ladje za naslednji korak ------------
    myship.theta = wrapToPi(best_theta);
    % REZULTAT: nova smer ladje: myship.theta

    %---------------------- Plot tirnic ----------------------------
    % plot all generated mpc trajectories
    if P.plot_trajectories
      for iter = 1: P.mpc_tirnic
        myship.p_traj(iter).LongitudeData = [myship.lon my_lon(iter,:)];
        myship.p_traj(iter).LatitudeData  = [myship.lat my_lat(iter,:)];
        myship.p_traj(iter).Visible = 'on';
        switch collisions(iter)
          case 0, myship.p_traj(iter).Color = P.colorOK;  % barva za OK tirnice
          case 1, myship.p_traj(iter).Color = P.colorNOK; % barva za NOK tirnice
        end
        % plot best trajectory
        if iter==norisk(best_traj_idx)
          myship.p_traj(iter).Color = 'b'; % barva za BEST tirnico
        end
      end
    end
    
end


%% ############################## Funkcija za COLREG ################################

%------------- COLREG : katera ladja se mora izogibati (in zato spremeni smer plovbe)
function [ships,i_risk_ships] = check_COLREG_rules(ships,i_myship)
global P status
    % sestavi uvodni seznam neaktivnih ladij
    i_other_ships = 1:length(ships);
    i_other_ships(i_myship) = [];

    % izračun linearnih trajektorij ladij za horizont predikcije
    for j = 1:length(ships)
      ships_la = ships(j).lat;
      ships_lo = ships(j).lon;
      for h = 1:P.mpc_horizon
        [ships_la, ships_lo] = update_ship_position(ships_la,ships_lo, ships(j).theta, ships(j).speed);
        ships_lat(j,h) = ships_la;
        ships_lon(j,h) = ships_lo;
      end
    end % ships_lat,ships_lon : vektor pozicij ladij

    % identificira nevarne ladje (z možnostjo trka)
    i_risk_ships = [];
    % Iteracija skozi linearno tirnico naše ladje
    for step = 1:P.mpc_horizon
      % Koordinate naše ladje v trenutnem koraku
      myship_lat = ships_lat(i_myship, step);
      myship_lon = ships_lon(i_myship, step);
      % Preveri razdaljo do vseh drugih ladij v istem časovnem koraku
      for other_ship = i_other_ships
        other_lat = ships_lat(other_ship, step);
        other_lon = ships_lon(other_ship, step);
        % Izračunaj haversine razdaljo
        distance = haversine(myship_lat, myship_lon, other_lat, other_lon); % PAZI, sem imel bug
        % Preveri, ali je razdalja manjša od bump_zone
        if distance < P.bump_zone
          i_risk_ships = [i_risk_ships other_ship];
        end
      end
    end
    i_risk_ships = unique(i_risk_ships);
    % če ni rizičnih ladij, prekine COLREG izvajanje
    if isempty(i_risk_ships)
      % update status box
      status.colreg  = 'ON (no risk of collision)';
      return
    end

    % preverja COLREG glede na nevarne ladje
    lat1   = ships(i_myship).lat;
    lon1   = ships(i_myship).lon;
    kot1   = rad2deg(ships(i_myship).theta);
    speed1 = ships(i_myship).speed;

    % po vseh nevarnih ladjah
    for j = i_risk_ships
        lat2   = ships(j).lat;
        lon2   = ships(j).lon;
        kot2   = rad2deg(ships(j).theta);
        speed2 = ships(j).speed;

        % Relativni kot pozicije ladje glede na našo ladjo
        dx = (lon2 - lon1) * cosd(lat1);
        dy = lat2 - lat1;
        % Kot relativnega položaja glede na našo ladjo
        phi = kot1 - atan2d(dy, dx);
        phi = wrapTo180(phi); % normalizacija kota v [-180, 180]
        % razlika smeri obeh ladij
        kot_diff = wrapTo180(kot1 - kot2);

        %--------------------------------------------------------------------------
        % PREVERJANJE COLREG PRAVIL GLEDE NA RELATIVNI KOT POZICIJE IN SMER GIBANJA
        % Situacija 1: Mi prehitevamo drugo ladjo → Mi se moramo umakniti.
        msg = 'OVERTAKING (Rule 13: give way – other vessel has the right of way)';
        % Situacija 2: Druga ladja prehiteva nas → Ona se mora umakniti.
        msg = 'BEING OVERTAKEN (Rule 13: stand on – other vessel must keep clear)';
        % Situacija 3: Obe ladji se srečujeta čelno → Obe morata zaviti desno.
        msg = 'HEAD-ON (Rule 14: both vessels must alter course to starboard)';
        % Situacija 4: Druga ladja prihaja z desne → Mi se moramo umakniti.
        msg = 'CROSSING FROM STARBOARD SIDE (Rule 15: give way – other vessel has the right of way)';
        % Situacija 5: Druga ladja prihaja z leve → Ona se mora umakniti.
        msg = 'CROSSING FROM PORT SIDE (Rule 15: stand on – other vessel must give way)';
        %--------------------------------------------------------------------------
        % 1. Naša ladja prehiteva (mi se umikamo)
        % TODO: pravilno bi bilo 67.5 (in ne 30)
        if abs(phi)          < 30 ... % pozicija ladje spredaj v kotu       +-30 st.  NOVO 4.4.2025
            && abs(kot_diff) < 30     % smer ladje podobna naši v toleranci +-30 st.
          msg = 'OVERTAKING (Rule 13: give way – other vessel has the right of way)';

        % 2. Head-On srečanje (obe ladji se umikata)
        elseif abs(phi)      < 15 ... % pozicija ladje spredaj v kotu +-15 st.
            && abs(kot_diff) > 165    % smer ladje nasprotna naši  NOVO 8.4.2025
          msg = 'HEAD-ON (Rule 14: both vessels must alter course)';
          % Akcija druge ladje: zaenkrat trivialna rešitev, ladjo usmerimo stran od naše
          smer_odklona = sign(kot1 - wrapTo180(180-kot2)); % NOVO 8.4.2025, preveri, če je OK
          ships(j).theta = wrapToPi(ships(j).theta + smer_odklona * P.max_turn_angle); % ladja naj zavije 20 st. stran
          %-------- spremeni barvo ladje, ki se izogiba
          c = ships(j).ic.IconColorData;
          c(find(c(:,:,1)==255)) = 80; % zatemni barvo (iz 255 na 100)
          ships(j).ic.IconColorData = c;

        % 3. Ladja prihaja iz desne (mi se umikamo)
        elseif phi > 0 && phi <= 112.5  % Ladja prihaja iz desne, zato se moramo umakniti
          msg = 'CROSSING FROM STARBOARD SIDE (Rule 15: give way – other vessel has the right of way)';

        % 4. Druga ladja prihaja iz leve (druga ladja se umika)
        elseif phi > -112.5 && phi < 0 % druga ladja (na levi) se mora umakniti
          msg = 'CROSSING FROM PORT SIDE (Rule 15: stand on – other vessel must give way)';
          % Akcija druge ladje: zaenkrat trivialna rešitev, ladjo usmerimo desno, ker nima prednosti
          ships(j).theta = wrapToPi(ships(j).theta - P.max_turn_angle); % ladja naj zavije 20 st. desno
          %-------- spremeni barvo ladje, ki se izogiba
          c = ships(j).ic.IconColorData;
          c(find(c(:,:,1)==255)) = 80; % zatemni barvo (iz 255 na 100)
          ships(j).ic.IconColorData = c;

        % 5. Druga ladja nas prehiteva (druga ladja se umika) (RAZLIKA HITROSTI IMPLICITNO VKLJUČENA PRI VKLOPU MPC)
        elseif abs(phi) > 150 ...       % pozicija ladje zadaj v kotu [-150,150]
            && abs(kot_diff) < 30 ...  % smer ladje podobna naši v toleranci +-30 st.            
          msg = 'BEING OVERTAKEN (Rule 13: stand on – other vessel must keep clear)';
          % Akcija druge ladje: zaenkrat trivialna rešitev, ladjo usmerimo desno, ker nima prednosti
          ships(j).theta = wrapToPi(ships(j).theta - P.max_turn_angle); % ladja naj zavije 20 st. desno
          % TODO: dodaj smer odklona levo/desno
          %-------- spremeni barvo ladje, ki se izogiba
          c = ships(j).ic.IconColorData;
          c(find(c(:,:,1)==255)) = 80; % zatemni barvo (iz 255 na 100)
          ships(j).ic.IconColorData = c;

        else % Ni posebne situacije po COLREG ... TODO: TOLE NE RABIM ?
          msg = 'ON (no action needed)';
        end
        %--------------------------------------------------------------------------
    end

    % update status
    status.colreg  = msg;
end
