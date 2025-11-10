%grid area to map dem file
%==========================================================================
%       UAV LIDAR MAPPING SIMULATION WITH REAL-WORLD TERRAIN (DEM)
%==========================================================================
% This script loads and DOWNSAMPLES a real-world terrain from a GeoTIFF
% DEM file, then simulates a fixed-wing UAV mapping the area with LIDAR.
%
% REQUIRES: Mapping Toolbox (for 'readgeoraster' function)
%==========================================================================

%% 1. SETUP & INITIALIZATION
%--------------------------------------------------------------------------
clear; clc; close all;
fprintf('Setting up simulation...\n');
% --- Simulation Parameters
T_sim = 8000;       
dt = 0.1;           % Simulation time step (seconds)
num_steps = T_sim/dt;
% --- UAV Initial State & Parameters
V = 50;             % UAV cruise speed (m/s)
altitude_agl = 150; % Flight altitude ABOVE GROUND LEVEL (m).
% --- Guidance Controller Parameters
capture_radius = 100; % Increased slightly for larger scale
Kp_psi = 2;

% --- LIDAR Sensor Parameters
lidar_range = 300;
lidar_scan_angle = 70;
lidar_num_beams = 51;

% --- Data Logging and Map Initialization
current_waypoint_idx = 2;
max_points = num_steps * lidar_num_beams;
mapPoints = zeros(3, max_points);
point_counter = 0; % Add a counter

% --- TERRAIN GENERATION FROM DEM FILE ---
fprintf('Generating terrain from DEM file...\n');
dem_filename = 'myDEM.tif';
if ~exist(dem_filename, 'file')
    error('DEM file "%s" not found. Please place it in the same directory as the script.', dem_filename);
end
% 1. Read the full-resolution DEM file
[Z_full, R] = readgeoraster(dem_filename);
Z_full = double(Z_full);
% 2. Downsample the data for performance
downsample_factor = 10;
Z = Z_full(1:downsample_factor:end, 1:downsample_factor:end);
fprintf('DEM downsampled by a factor of %d. Original size: %dx%d, New size: %dx%d\n', ...
        downsample_factor, size(Z_full, 2), size(Z_full, 1), size(Z, 2), size(Z, 1));
% 3. Manually create local X and Y coordinates in meters from geographic limits
num_rows = size(Z, 1);
num_cols = size(Z, 2);
lat_lim = R.Latlim;
lon_lim = R.Lonlim;
m_per_deg_lat = 111132.954 - 559.822 * cos(2 * mean(lat_lim)) + 1.175 * cos(4 * mean(lat_lim));
m_per_deg_lon = 111319.488 * cos(mean(lat_lim));
map_width_m = (lon_lim(2) - lon_lim(1)) * m_per_deg_lon;
map_height_m = (lat_lim(2) - lat_lim(1)) * m_per_deg_lat;
x_vector = linspace(0, map_width_m, num_cols);
y_vector = linspace(0, map_height_m, num_rows);
[X, Y] = meshgrid(x_vector, y_vector);
% 4. Handle NoData values
no_data_value = -32767;
Z(Z == no_data_value) = NaN;

% --- Define the Path (Lawnmower Pattern) IN LOCAL METERS ---

% 1. Define the parameters of the rectangular area you want to map
%    (Using 3% of the total map width/height for a fast test)
x_start = 0.1 * max(x_vector);        % Bottom-left corner X
y_start = 0.1 * max(y_vector);        % Bottom-left corner Y
area_width = 0.15 * max(x_vector);    % How wide to make the rectangle
area_height = 0.15 * max(y_vector);   % How tall to make the rectangle

% 2. Calculate the optimal spacing between flight lines (the "swath")
%    This ensures the LIDAR scans overlap slightly.
swath_width = 2 * altitude_agl * tan(deg2rad(lidar_scan_angle) / 2);
line_spacing = swath_width * 0.75; % 75% spacing for a 25% overlap
num_lines = ceil(area_width / line_spacing);

% 3. Generate the waypoints in a back-and-forth (boustrophedon) pattern
waypoints = []; % Initialize an empty array for the waypoints
current_x = x_start;

for i = 1:num_lines
    if mod(i, 2) == 1
        % Odd line: Fly "up" (North)
        waypoints = [waypoints; current_x, y_start];
        waypoints = [waypoints; current_x, y_start + area_height];
    else
        % Even line: Fly "down" (South)
        waypoints = [waypoints; current_x, y_start + area_height];
        waypoints = [waypoints; current_x, y_start];
    end
    
    % Move "right" (East) to the start of the next line
    current_x = current_x + line_spacing;
end

fprintf('Generated %d waypoints for a %d-line grid pattern.\n', size(waypoints, 1), num_lines);

% --- UAV Initial State
uav_state = zeros(3, 1);
uav_state(1) = waypoints(1,1);
uav_state(2) = waypoints(1,2);
uav_state(3) = atan2(waypoints(2,2) - waypoints(1,2), waypoints(2,1) - waypoints(1,1));

%% 2. ANIMATION VISUALIZATION SETUP
%--------------------------------------------------------------------------
fprintf('Setting up animation visualization...\n');
fig_anim = figure('Name', 'UAV LIDAR Mapping Animation');
ax_anim = axes('Parent', fig_anim);
view(3);
grid on;
hold on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');
zlabel('Altitude (m)');
title('UAV LIDAR Mapping Animation');
surf(X, Y, Z, 'FaceAlpha', 0.8, 'EdgeColor', 'none', 'Parent', ax_anim);
colormap(ax_anim, parula);
% --- *** A robust starting Z *** ---
valid_Z_values_init = Z(~isnan(Z));
z_ground_start = interp2(X, Y, Z, uav_state(1), uav_state(2));
if isnan(z_ground_start)
    z_ground_start = mean(valid_Z_values_init); % Use mean of all valid data
end
flight_altitude_msl = z_ground_start + altitude_agl;
plot3(ax_anim, waypoints(:,1), waypoints(:,2), (min(valid_Z_values_init)+altitude_agl)*ones(size(waypoints,1),1), 'ro--', 'LineWidth', 2);
mapPlot_anim = plot3(ax_anim, NaN, NaN, NaN, '.r', 'MarkerSize', 2);
uav_size = (max(x_vector) - min(x_vector)) / 20;
uav_body_pts = [uav_size, 0, 0; -uav_size/2, uav_size/2, 0; -uav_size/2, -uav_size/2, 0; uav_size, 0, 0]';
h_transform = hgtransform('Parent', ax_anim);
uav_plot = plot3(uav_body_pts(1,:), uav_body_pts(2,:), uav_body_pts(3,:), 'k-', 'LineWidth', 2, 'Parent', h_transform);
legend(ax_anim, 'True Terrain', 'Desired Path', 'Mapped Points (Animation)', 'UAV', 'Location', 'best');

%% 3. SIMULATION LOOP 
%--------------------------------------------------------------------------
fprintf('Initialization complete. Starting simulation loop...\n');

% Define plot update frequency to speed up simulation
plot_update_frequency = 20; % Update plot every 20 steps

for i = 2:num_steps
  
   target_wp = waypoints(current_waypoint_idx, :);
   dist_to_wp = sqrt((target_wp(1) - uav_state(1))^2 + (target_wp(2) - uav_state(2))^2);
  
   if dist_to_wp < capture_radius
       if current_waypoint_idx < size(waypoints, 1)
           current_waypoint_idx = current_waypoint_idx + 1;
       else
           fprintf('Final waypoint reached.\n');
           break;
       end
   end
  
   % --- UAV State Update ---
   psi_desired = atan2(target_wp(2) - uav_state(2), target_wp(1) - uav_state(1));
   error_psi = atan2(sin(psi_desired - uav_state(3)), cos(psi_desired - uav_state(3)));
   psi_dot = Kp_psi * error_psi;
   uav_state(3) = uav_state(3) + psi_dot * dt;
   uav_state(3) = atan2(sin(uav_state(3)), cos(uav_state(3)));
   uav_state(1) = uav_state(1) + V * cos(uav_state(3)) * dt;
   uav_state(2) = uav_state(2) + V * sin(uav_state(3)) * dt;
  
   % --- FIX 1: TERRAIN FOLLOWING ---
   % Get ground height *directly under* the UAV
   current_ground_z = interp2(X, Y, Z, uav_state(1), uav_state(2));
   if isnan(current_ground_z)
       % Handle flying off-map or over NaN data
       valid_Z_values = Z(~isnan(Z));
       current_ground_z = mean(valid_Z_values);
   end
   % This is the UAV's NEW altitude (MSL), updated every step
   uav_current_msl = current_ground_z + altitude_agl;

   % --- LIDAR SENSOR LOGIC ---
   scan_angles_rad = linspace(-deg2rad(lidar_scan_angle)/2, deg2rad(lidar_scan_angle)/2, lidar_num_beams);
   rays_body = [zeros(1, lidar_num_beams); sin(scan_angles_rad); -cos(scan_angles_rad)];
   R_yaw = [cos(uav_state(3)), -sin(uav_state(3)), 0;
            sin(uav_state(3)),  cos(uav_state(3)), 0;
            0,                  0,                 1];
   rays_world = R_yaw * rays_body;
  
   detectedPoints = [];
   for k = 1:lidar_num_beams
       ray_dir = rays_world(:,k);
      
       % RAY INTERSECTION ---
       % The distance to the ground is 'altitude_agl'.
       % We find the intersection 't' along the ray vector.
       t_intersect = altitude_agl / -ray_dir(3);
     

       if t_intersect > 0 && t_intersect < lidar_range
           % Find the (x,y) location of this intersection
           intersect_xy = [uav_state(1); uav_state(2)] + t_intersect * ray_dir(1:2);
           
           % Now find the *actual* Z at that (x,y)
           z_terrain_actual = interp2(X, Y, Z, intersect_xy(1), intersect_xy(2));
          
           if ~isnan(z_terrain_actual)
               detectedPoints = [detectedPoints, [intersect_xy; z_terrain_actual]];
           end
       end
   end
   
   % EFFICIENT ARRAY FILLING ---
   num_new_points = size(detectedPoints, 2);
   if num_new_points > 0
       if (point_counter + num_new_points) <= max_points
           mapPoints(:, point_counter+1 : point_counter+num_new_points) = detectedPoints;
           point_counter = point_counter + num_new_points;
       else
           fprintf('Warning: Max points limit reached. Halting data collection.\n');
           break; % Stop simulation if we run out of pre-allocated space
       end
   end
  
   % EFFICIENT ANIMATION ---
   % Only update the plot every 'plot_update_frequency' steps
   if mod(i, plot_update_frequency) == 0
       % Use the DYNAMIC 'uav_current_msl' for the plot
       translation = makehgtform('translate', [uav_state(1), uav_state(2), uav_current_msl]);
       rotation = makehgtform('zrotate', uav_state(3));
       set(h_transform, 'Matrix', translation * rotation);
       
       % Update the mapped points (only showing the ones collected so far)
       set(mapPlot_anim, 'XData', mapPoints(1, 1:point_counter), ...
                         'YData', mapPoints(2, 1:point_counter), ...
                         'ZData', mapPoints(3, 1:point_counter));
       drawnow limitrate;
   end
   
end
fprintf('Simulation finished.\n');

%% 4. FINAL POINT CLOUD VISUALIZATION (Height-colored)
%--------------------------------------------------------------------------
fprintf('Generating final height-colored point cloud visualization...\n');
if point_counter > 0
   fig_map = figure('Name', 'Height-Colored LIDAR Map');
   ax_map = axes('Parent', fig_map);
  
   % Trim the mapPoints array  ---
   finalMapPoints = mapPoints(:, 1:point_counter);
  
   x_cloud = finalMapPoints(1,:);
   y_cloud = finalMapPoints(2,:);
   z_cloud = finalMapPoints(3,:);
   
   scatter3(ax_map, x_cloud, y_cloud, z_cloud, 10, z_cloud, 'filled');
  
   colormap(ax_map, jet);
   c = colorbar(ax_map);
   c.Label.String = 'Altitude (m)';
  
   % --- THIS IS THE CAXIS LINE ---
   % First, create a vector of all Z values that are not NaN
   valid_Z_values = Z(~isnan(Z));
   % Then, find the min and max of that vector
   caxis(ax_map, [min(valid_Z_values) max(valid_Z_values)]);
   % ----------------------------------------
  
   view(3);
   grid on;
   % axis equal;
   xlabel('East (m)');
   ylabel('North (m)');
   zlabel('Altitude (m)');
   title('Final LIDAR Point Cloud Map (Colored by Height)');
  
   axis(ax_map, 'tight');
  
else
   fprintf('No points were collected for the final map visualization.\n');
end

%% 5. EXPORT POINT CLOUD DATA
%--------------------------------------------------------------------------
fprintf('Exporting point cloud data to CSV...\n');
if point_counter > 0
   % Use the trimmed array for export 
   pointCloudData = mapPoints(:, 1:point_counter)'; 
   filename = 'uav_lidar_map.csv';
   writematrix(pointCloudData, filename);
   fprintf('Successfully saved %d points to %s\n', size(pointCloudData, 1), filename);
else
   fprintf('No points were collected. Nothing to export.\n');
end