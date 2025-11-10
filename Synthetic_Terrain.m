%terrain generated
%==========================================================================
%       UAV LIDAR MAPPING SIMULATION WITH CUSTOM ANIMATION & DATA EXPORT
%==========================================================================
% This script simulates a fixed-wing UAV flying over a generated FOREST
% terrain while using a simplified LIDAR sensor to map the ground below it.
% After the simulation, it exports the final point cloud to a CSV file
% AND generates a height-colored 3D point cloud visualization.
%==========================================================================

%% 1. SETUP & INITIALIZATION
%--------------------------------------------------------------------------
clear; clc; close all;
fprintf('Setting up simulation...\n');
% --- Simulation Parameters
T_sim = 200;       % Max simulation time (seconds)
dt = 0.1;          % Simulation time step (seconds)
num_steps = T_sim/dt;
% --- Define the Path (Waypoints)
waypoints = [  0,   0;
            200,   0;
            200, 300;
            400, 300;
            400, 100;
            600, 100];
% --- UAV Initial State & Parameters
uav_state = zeros(3, 1); % [x(m), y(m), psi(rad)] -> [East, North, Yaw]
uav_state(1) = waypoints(1,1);
uav_state(2) = waypoints(1,2);
uav_state(3) = atan2(waypoints(2,2) - waypoints(1,2), waypoints(2,1) - waypoints(1,1));
V = 15;            % UAV cruise speed (m/s)
altitude = 80;     % Flight altitude ABOVE SEA LEVEL (m).
% --- Guidance Controller Parameters
capture_radius = 15;
Kp_psi = 2;
% --- Data Logging and Map Initialization
current_waypoint_idx = 2;
mapPoints = []; % Initialize an empty array to store the point cloud
% --- TERRAIN GENERATION ---
fprintf('Generating forest terrain...\n');
% 1. Create the base ground using sine waves
x_terrain = -50:2:650; % Increased resolution for better tree shapes
y_terrain = -50:2:350;
[X, Y] = meshgrid(x_terrain, y_terrain);
Z_ground = 5 * sin(X/30) .* cos(Y/50) + 8 * cos(X/80) - 3 * sin(Y/40);
Z_ground = Z_ground + 20; % Raise the terrain base altitude
Z = Z_ground; % Initialize final terrain with ground
% 2. Define Forest Parameters
num_trees = 200;
min_tree_height = 10;
max_tree_height = 25;
min_tree_radius = 3;
max_tree_radius = 8;
% 3. Add Trees to the Terrain
for k = 1:num_trees
   % Pick a random location for the tree center
   x_center = min(x_terrain) + rand() * (max(x_terrain) - min(x_terrain));
   y_center = min(y_terrain) + rand() * (max(y_terrain) - min(y_terrain));
  
   % Pick random properties for the tree
   tree_height = min_tree_height + rand() * (max_tree_height - min_tree_height);
   tree_radius = min_tree_radius + rand() * (max_tree_radius - min_tree_radius);
  
   % Find all grid points within the tree's radius
   distance_from_center = sqrt((X - x_center).^2 + (Y - y_center).^2);
   tree_indices = distance_from_center <= tree_radius;
  
   % Create a cone shape for the tree canopy
   cone_height = tree_height * (1 - distance_from_center(tree_indices) / tree_radius);
  
   % Add the cone height to the ground height at the tree's location
   new_tree_surface_height = Z_ground(tree_indices) + cone_height;
  
   % Ensure the new tree surface is placed on top of whatever is already there
   Z(tree_indices) = max(Z(tree_indices), new_tree_surface_height);
end
% --- LIDAR Sensor Parameters
lidar_range = 100;
lidar_scan_angle = 70;
lidar_num_beams = 51;

%% 2. ANIMATION VISUALIZATION SETUP (Real-time display)
%--------------------------------------------------------------------------
fprintf('Setting up animation visualization...\n');
fig_anim = figure('Name', 'UAV LIDAR Mapping Animation');
ax_anim = axes('XLim',[-50 650], 'YLim',[-50 350], 'ZLim',[0 120], 'Parent', fig_anim);
view(3);
grid on;
hold on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');
zlabel('Altitude (m)');
title('UAV LIDAR Mapping Animation');
surf(X, Y, Z, 'FaceAlpha', 0.6, 'EdgeColor', 'none', 'Parent', ax_anim);
colormap(ax_anim, winter); % colormap to look more "forest-like"
plot3(ax_anim, waypoints(:,1), waypoints(:,2), altitude*ones(size(waypoints,1),1), 'ro--', 'LineWidth', 2);
mapPlot_anim = plot3(ax_anim, NaN, NaN, NaN, '.r', 'MarkerSize', 2);
uav_size = 10;
uav_body_pts = [uav_size, 0, 0; -uav_size/2, uav_size/2, 0; -uav_size/2, -uav_size/2, 0; uav_size, 0, 0]';
h_transform = hgtransform('Parent', ax_anim);
uav_plot = plot3(uav_body_pts(1,:), uav_body_pts(2,:), uav_body_pts(3,:), 'k-', 'LineWidth', 2, 'Parent', h_transform);
legend(ax_anim, 'True Terrain', 'Desired Path', 'Mapped Points (Animation)', 'UAV', 'Location', 'best');

%% 3. SIMULATION LOOP
%--------------------------------------------------------------------------
fprintf('Initialization complete. Starting simulation loop...\n');
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
  
   psi_desired = atan2(target_wp(2) - uav_state(2), target_wp(1) - uav_state(1));
   error_psi = atan2(sin(psi_desired - uav_state(3)), cos(psi_desired - uav_state(3)));
   psi_dot = Kp_psi * error_psi;
   uav_state(3) = uav_state(3) + psi_dot * dt;
   uav_state(3) = atan2(sin(uav_state(3)), cos(uav_state(3)));
   uav_state(1) = uav_state(1) + V * cos(uav_state(3)) * dt;
   uav_state(2) = uav_state(2) + V * sin(uav_state(3)) * dt;
  
   scan_angles_rad = linspace(-deg2rad(lidar_scan_angle)/2, deg2rad(lidar_scan_angle)/2, lidar_num_beams);
   rays_body = [zeros(1, lidar_num_beams); sin(scan_angles_rad); -cos(scan_angles_rad)];
   R_yaw = [cos(uav_state(3)), -sin(uav_state(3)), 0;
            sin(uav_state(3)),  cos(uav_state(3)), 0;
            0,                  0,                 1];
   rays_world = R_yaw * rays_body;
  
   detectedPoints = [];
   for k = 1:lidar_num_beams
       ray_dir = rays_world(:,k);
       t_intersect = altitude / -ray_dir(3);
       if t_intersect > 0 && t_intersect < lidar_range
           intersect_xy = [uav_state(1); uav_state(2)] + t_intersect * ray_dir(1:2);
           z_terrain_actual = interp2(X, Y, Z, intersect_xy(1), intersect_xy(2));
           if ~isnan(z_terrain_actual)
               detectedPoints = [detectedPoints, [intersect_xy; z_terrain_actual]];
           end
       end
   end
   mapPoints = [mapPoints, detectedPoints];
  
   translation = makehgtform('translate', [uav_state(1), uav_state(2), altitude]);
   rotation = makehgtform('zrotate', uav_state(3));
   set(h_transform, 'Matrix', translation * rotation);
   set(mapPlot_anim, 'XData', mapPoints(1,:), 'YData', mapPoints(2,:), 'ZData', mapPoints(3,:));
   drawnow limitrate;
end
fprintf('Simulation finished.\n');

%% 4. FINAL POINT CLOUD VISUALIZATION (Height-colored)
%--------------------------------------------------------------------------
fprintf('Generating final height-colored point cloud visualization...\n');
if ~isempty(mapPoints)
   fig_map = figure('Name', 'Height-Colored LIDAR Map');
   ax_map = axes('Parent', fig_map);
  
   x_cloud = mapPoints(1,:);
   y_cloud = mapPoints(2,:);
   z_cloud = mapPoints(3,:);
  
   scatter3(ax_map, x_cloud, y_cloud, z_cloud, 10, z_cloud, 'filled');
  
   colormap(ax_map, jet);
   c = colorbar(ax_map);
   c.Label.String = 'Altitude (m)';
  
   caxis(ax_map, [min(Z(:)) max(Z(:))]);
  
   view(3);
   grid on;
   axis equal;
   xlabel('East (m)');
   ylabel('North (m)');
   zlabel('Altitude (m)');
   title('Final LIDAR Point Cloud Map (Colored by Height)');
  
   set(ax_map, 'XLim',[-50 650], 'YLim',[-50 350], 'ZLim',[min(Z(:)) max(Z(:)) + 20]);
  
else
   fprintf('No points were collected for the final map visualization.\n');
end

%% 5. EXPORT POINT CLOUD DATA
%--------------------------------------------------------------------------
fprintf('Exporting point cloud data to CSV...\n');
if ~isempty(mapPoints)
   pointCloudData = mapPoints';
   filename = 'uav_lidar_map.csv';
   writematrix(pointCloudData, filename);
   fprintf('Successfully saved %d points to %s\n', size(pointCloudData, 1), filename);
else
   fprintf('No points were collected. Nothing to export.\n');
end
