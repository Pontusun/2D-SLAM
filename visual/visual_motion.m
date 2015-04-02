close all
addpath('../');
load lidar3.mat


pose = [];
for i = 10:50:numel(lidar)
    pose = [pose; lidar{i}.pose(1), lidar{i}.pose(2), cos(lidar{i}.pose(3)), sin(lidar{i}.pose(3))];
    plot(pose(:,1),pose(:,2));
    axis([-12 12 -12 12]);
    
    hold on
    %quiver(pose(:,1),pose(:,2),pose(:,3),pose(:,4));
    quiver(lidar{i}.pose(1), lidar{i}.pose(2), cos(lidar{i}.pose(3)), sin(lidar{i}.pose(3)));    
    hold off
    pause(0.025);
    i
end

