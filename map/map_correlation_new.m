function [ c ] = map_correlation_new( map, x_lidar_map, y_lidar_map, x_range,y_range )
%MAP_CORRELATION_NEW Summary of this function goes here
%   Detailed explanation goes here

n = length(x_range);
c = zeros(n);

k = size(map,1);

for i=1:n
    for j=1:n
        %ind_lidar = sub2ind(size(map), x_lidar_map+x_range(i), y_lidar_map+y_range(j) );
        ind_lidar = x_lidar_map+x_range(i) + (y_lidar_map+y_range(j) -1)*k;
        c(i,j) = sum(map(ind_lidar));
    end
end

% ind_lidar = sub2ind(size(map), x_lidar_map, y_lidar_map );
% c = sum(map(ind_lidar));

end

