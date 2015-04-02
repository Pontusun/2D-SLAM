clc 
clear all
close all
addpath(genpath('./'))
addpath('../');
addpath('../mex/');

load map_result_test.mat
load depth.mat
load rgb.mat

DEPTH_MAX = 4500; 
DEPTH_MIN = 400;

%%
ground = [];
for k= 500%2:2:numel(DEPTH)
   
   D = DEPTH{k}.depth';
   D = flip(D,2);
   D(D(:) <= DEPTH_MIN) = 0;
   D(D(:) >= DEPTH_MAX) = 0;  
   %figure(1), imagesc(D);    
   R = djpeg(RGB{k}.jpeg);
   R = flip(R,2);
   %figure(2), imagesc(R);
   
   % resize RGB image for depth
   % image FOV = 84.1 x 53.8, depth FOV = 70.6 x 60
   % result FOV = 70.6 * 53.8
   k_img_col = tan(70.6/2/180*pi)/tan(84.1/2/180*pi);
   k_dep_row = tan(53.8/2/180*pi)/tan(60/2/180*pi);
   img_col = round(size(R,2)*k_img_col/4)*4;
   dep_row = round(size(D,1)*k_dep_row/4)*4;
   for i=1:3
       img_cut(:,:,i) = R(:, size(R,2)/2-img_col/2:size(R,2)/2+img_col/2, i );
   end
   D_new = D(size(D,1)/2-dep_row/2:size(D,1)/2+dep_row/2, :);
   R_new = imresize(img_cut, size(D_new));
   
   k_scale = size(img_cut,1)/size(D_new,1);
   
   % x,y,z in kinect frame
   [x_k, y_k] = meshgrid(1:size(D_new,2),1:size(D_new,1));
   x_k = x_k - round(size(D_new,2)/2);
   y_k = y_k - round(size(D_new,1)/2);
   z_k = D_new;  
   xyz_k(:,1) = reshape(x_k.*D_new,[],1)/1050*k_scale;
   xyz_k(:,2) = reshape(z_k,[],1);
   xyz_k(:,3) = reshape(-y_k.*D_new,[],1)/1050*k_scale;
   xyz_k(:,4) = reshape(double(R_new(:,:,1))/255,[],1);
   xyz_k(:,5) = reshape(double(R_new(:,:,2))/255,[],1);
   xyz_k(:,6) = reshape(double(R_new(:,:,3))/255,[],1);
   
   % project
   delta_t =  abs(map_result.trace(:,3) - DEPTH{k}.t);
   [tmp, idx] = min(delta_t);
   R_bw =  map_result.R{idx};
   n_pix = size(xyz_k,1);
   xyz_w(:,1) = sum( xyz_k(:,1:3) .* repmat(R_bw(1,:),n_pix,1) , 2 )/1000;
   xyz_w(:,2) = sum( xyz_k(:,1:3) .* repmat(R_bw(2,:),n_pix,1) , 2 )/1000;
   xyz_w(:,3) = sum( xyz_k(:,1:3) .* repmat(R_bw(3,:),n_pix,1) , 2 )/1000 + 1.41;
   xyz_w(:,4:6) = xyz_k(:,4:6);
   figure(1), scatter3(xyz_w(1:5:end,1),xyz_w(1:5:end,2),xyz_w(1:5:end,3),1,xyz_w(1:5:end,4:6) );
   %figure(2), imagesc(D_new);
   figure(2), imagesc(R_new);
   
   % detect ground
   ground_idx = find(xyz_w(:,3) < 1);
   ground = [  xyz_w(ground_idx,1) + map_result.trace(idx,2), xyz_w(ground_idx,2) - map_result.trace(idx,1), xyz_w(ground_idx,4:6) ];
   %figure(3), plot(ground(:,1), ground(:,2) );
   %axis = [-30, 30, -30, 30];
   
   % viusal ground map
   %figure(4), plot(ground(:,1),ground(:,2),'b' );
   figure(4), scatter(ground(1:40:end,1),ground(1:40:end,2),1,ground(1:40:end,3:5) );
   hold on
   plot(map_result.trace(idx,2), -map_result.trace(idx,1),'o','LineWidth',2,'MarkerSize',2,'MarkerEdgeColor','r');
   axis([-10, 20, -10, 20]);
   %hold off
   pause(0.01);
end