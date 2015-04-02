clc 
clear all
close all
addpath(genpath('./'))
addpath('../');
addpath('../mex/');

%% load data
data_no = 3;
load lidar3.mat
load joint3.mat

iNeck = get_joint_index('Neck'); % head yaw
iHead = get_joint_index('Head'); % head pitch

if data_no <= 1
    head_angles = [pos(:,iNeck), pos(:,iHead)];
    %figure(1);plot(ts, pos(:, iNeck))
    %figure(2);plot(ts, pos(:, iHead))
else
    head_angles = [pos(iNeck,:)', pos(iHead,:)'];
    %figure(1);plot(ts, pos(iNeck,:))
    %figure(2);plot(ts, pos(iHead,:))
end

%% initialize
map = map_initial( 40, 0.1 );

theta = 0:0.25:270;
theta = theta*pi/180;

n = numel(lidar);
step = 50;

%% pure walking odometry
plot_hist = [];
z_hist = [];

for i = 10:step:n
    
    if data_no<= 1
        t_lidar = lidar{i}.t-t0;
        delta_t = abs(ts - t_lidar);
    else
        delta_t = abs(ts - lidar{i}.t);       
    end
    [tmp, syc_idx] = min(delta_t);
    
    x_robo = lidar{i}.pose(1);
    y_robo = lidar{i}.pose(2);
    yaw_robo = lidar{i}.pose(3);
    
    % remove noisy data out of valid range
    lidar{i}.scan(find(lidar{i}.scan > 30)) = 0;
    x_lidar_tmp = lidar{i}.scan .* cos(theta);
    y_lidar_tmp = lidar{i}.scan .* sin(theta);
    
    %x_lidar = x_robo + x_lidar_tmp * cos(yaw_robo) - y_lidar_tmp * sin(yaw_robo);
    %y_lidar = y_robo + x_lidar_tmp * sin(yaw_robo) + y_lidar_tmp * cos(yaw_robo);
    
    %R = rpy2wrb_xyz( 0, 0, yaw_robo);
    R = rpy2wrb_xyz( 0, head_angles(syc_idx,2), head_angles(syc_idx,1)+yaw_robo); % pitch, roll, yaw
    xyz_lidar = R * [x_lidar_tmp; y_lidar_tmp; zeros(1,1081)];
    x_lidar = x_robo + xyz_lidar(1,:);
    y_lidar = y_robo + xyz_lidar(2,:);
    z_lidar = xyz_lidar(3,:);
    %figure(3); plot3(x_lidar, y_lidar, z_lidar);
    axis([-15,15,-15,15,-15,15]);
    
    x_robo_map = xy2map(map, x_robo);
    y_robo_map = xy2map(map, y_robo);
    x_lidar_map = xy2map(map, x_lidar);
    y_lidar_map = xy2map(map, y_lidar);
    
    % get map cells from ray
    [ x_between, y_between] = getMapCellsFromRay((x_robo_map*ones(1,1081)),(y_robo_map*ones(1,1081)), double(x_lidar_map), double(y_lidar_map));
    
    % update grid map
    map = map_update(map, x_between, y_between, x_lidar_map, y_lidar_map, 100);
    
    % visualize map
    plot_hist = [plot_hist; x_robo_map, y_robo_map];
    figure(4); visual_map(map, plot_hist);
    
    pause(0.015);
end

