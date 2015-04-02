clc 
clear all
close all
addpath(genpath('./'))
addpath('../');
addpath('../mex/');

%% load data
data_no = 3;
load lidar.mat
load joint.mat

vidObj = VideoWriter('map_vedio_test','MPEG-4');
vidObj.FrameRate = 24;
open(vidObj);

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

n = numel(lidar);
step = 10;

%logodds = 5,10;
logodds = 8;
logodds_limit = 127;
map = map_initial( 40, 0.1 );

n_particles = 1000;
[particles, w ] = PF_initial( n_particles, lidar{1}.pose );
%sigma = [ 0.005, 0.005, 0.002] * 10;
sigma = [ 0.005, 0.005, 0.002 ] * 15;
correlation = zeros(n_particles,1);
x_im = map.x_min : map.res : map.x_max;
y_im = map.y_min : map.res : map.y_max;
% x_range = -0.05:0.05:0.05;
% y_range = -0.05:0.05:0.05;
x_range = -1:1:1;
y_range = -1:1:1;

%% SLAM
plot_hist = [];
R_hist = {};
z_hist = [];
counter = 0;

for i = 1000:step:n
    counter = counter + 1;
    
    if data_no<= 1
        t_lidar = lidar{i}.t-t0;
        delta_t = abs(ts - t_lidar);
    else
        delta_t = abs(ts - lidar{i}.t);       
    end
    [tmp, syc_idx] = min(delta_t);
    
    if counter == 1
        % process lidar
        [ x_between, y_between, x_robo_map, y_robo_map, x_lidar_map, y_lidar_map, x_lidar, y_lidar, tmp_R ] = lidar_process( lidar{i}, lidar{i}.pose, head_angles(syc_idx,:), map);
        % update grid map
        map = map_update(map, x_between, y_between, x_lidar_map, y_lidar_map, logodds);         
        % prepare for next loop
        odom_last = lidar{i}.pose;
    else 
        move_robo = lidar{i}.pose - odom_last;
        odom_last = lidar{i}.pose;
        particles_new = PF_dynamic(particles, move_robo, sigma);
        
        map_tmp = int8(map.gridmap);
        map_tmp(map_tmp<0) = 0;
        %map_tmp(map_tmp<-10) = -10;
        %map_tmp(map_tmp>0) = 0;
        for j = 1:n_particles
            % process lidar
            [ x_between_p{j}, y_between_p{j}, x_robo_map_p{j}, y_robo_map_p{j}, x_lidar_map_p{j}, y_lidar_map_p{j}, x_lidar_p{j}, y_lidar_p{j}, R{j} ] = lidar_process( lidar{i}, particles_new(j,:), head_angles(syc_idx,:), map);
            % compute correlation
            %c = map_correlation( map_tmp, x_im, y_im,[x_lidar_p{j}; y_lidar_p{j}; zeros(1,length(x_lidar_p{j}))],x_range,y_range);
            c = map_correlation_new(map_tmp, x_lidar_map_p{j}, y_lidar_map_p{j}, x_range,y_range);
            %correlation(j) = max(max(c));
            correlation(j) = sum(sum(c));
            %correlation(j) = sum((x_lidar_p{j}-x_lidar).^2 + (y_lidar_p{j}-y_lidar).^2);
        end
        %update weights
        w = (correlation.^2)./sum(correlation.^2);
        %update map
        [w_max, p_idx] = max(w);
        map = map_update(map, x_between_p{p_idx}, y_between_p{p_idx}, x_lidar_map_p{p_idx}, y_lidar_map_p{p_idx}, logodds);      
        %resample
        idx_new = PF_resample( w )';
        particles = particles_new(idx_new,:);
        % log data
        R_hist{counter-1} = R{p_idx};
        plot_hist = [plot_hist; x_robo_map_p{p_idx}, y_robo_map_p{p_idx}, lidar{i}.t];
        %pause(0.001);
    end 
    i
   % Write each frame to the file.
   %currFrame = im_replaced{k};
   if mod(counter,4)==0
        % visualize map
        figure(4); I = uint8(map.gridmap + 128);
        imshow( I );
        hold on
        if size(plot_hist,1) == 1
            plot(plot_hist(end,2), plot_hist(end,1), 'o','LineWidth',3,'MarkerSize',5,'MarkerEdgeColor','r');
        else
            plot(plot_hist(:,2), plot_hist(:,1),'r');
            plot(plot_hist(end,2), plot_hist(end,1), 'o','LineWidth',3,'MarkerSize',5,'MarkerEdgeColor','r');
            %scatter(y_lidar_map_p{p_idx}, x_lidar_map_p{p_idx},1,'g');
            plot(y_lidar_map_p{p_idx},x_lidar_map_p{p_idx},'g');
        end
        hold off
       F = getframe(gcf);
       writeVideo(vidObj,imresize(F.cdata,[600, 600]));
       pause(0.001);
   end
end

 close(vidObj);

%% save map & trace
% map_result.trace(:,1) = plot_hist(:,1) * map.res + map.x_min ;
% map_result.trace(:,2) = plot_hist(:,2) * map.res + map.y_min ;
% map_result.trace(:,3) = plot_hist(:,3) ;
% map_result.R = R_hist;
% save map_result_test.mat map_result


