function [ x_between, y_between, x_robo_map, y_robo_map, x_lidar_map, y_lidar_map, x_lidar, y_lidar, R ] = lidar_process( lidar_data, pose, head_angles, map)
%LIDAR_PROCESS Summary of this function goes here
%   Detailed explanation goes here

    theta = 0:0.25:270;
    theta = theta*pi/180;
    
    % remove noisy data out of valid range
    lidar_data.scan(lidar_data.scan > 30) = 0;
    x_lidar_tmp = lidar_data.scan .* cos(theta);
    y_lidar_tmp = lidar_data.scan .* sin(theta);

    x_robo = pose(1);
    y_robo = pose(2);
    yaw_robo = pose(3);

    %R = rpy2wrb_xyz( 0, 0, yaw_robo);
    R = rpy2wrb_xyz( -head_angles(2), 0, head_angles(1)+yaw_robo); % pitch, roll, yaw
    xyz_lidar = R * [x_lidar_tmp; y_lidar_tmp; zeros(1,1081)];
    x_lidar = x_robo + xyz_lidar(1,:);
    y_lidar = y_robo + xyz_lidar(2,:);
    %z_lidar = xyz_lidar(3,:);
    %figure(3); plot3(x_lidar, y_lidar, z_lidar);
    %axis([-15,15,-15,15,-15,15]);

    x_robo_map = xy2map(map, x_robo);
    y_robo_map = xy2map(map, y_robo);
    x_lidar_map = xy2map(map, x_lidar);
    y_lidar_map = xy2map(map, y_lidar);

    % get map cells from ray
    [ x_between, y_between] = getMapCellsFromRay((x_robo_map),(y_robo_map), double(x_lidar_map), double(y_lidar_map));

end

