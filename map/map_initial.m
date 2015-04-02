function [ map ] = map_initial( range, resolution )
%MAP_INITIAL Summary of this function goes here
%   Detailed explanation goes here
map.x_max = range;
map.x_min = -range;
map.y_max = range;
map.y_min = -range;
map.res = resolution;

%n = round(2*range/resolution);
n = ceil( (map.x_max-map.x_min)/map.res + 1 );
map.grid_num = n;
%map.grid_mid = round(n/2);

map.gridmap = zeros(n,n,'int16');

end

