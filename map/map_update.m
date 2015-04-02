function [ map_new ] = map_update( map, x_between, y_between, x_lidar_map, y_lidar_map, odds )
%MAP_UPDATE Summary of this function goes here
%   Detailed explanation goes here

map_new = map;

ind_between = sub2ind(size(map.gridmap), x_between, y_between );
ind_lidar = sub2ind(size(map.gridmap), x_lidar_map, y_lidar_map );

map_new.gridmap(ind_between) = map.gridmap(ind_between) - odds;
map_new.gridmap(ind_lidar) = map.gridmap(ind_lidar) + odds;

limit = 127;
map_new.gridmap(map_new.gridmap >limit ) = limit;
map_new.gridmap(map_new.gridmap <-limit) = -limit;

end

