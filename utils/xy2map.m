function [ ind ] = xy2map( map, x )
%XY2MAP Summary of this function goes here
%   Detailed explanation goes here

%ind = round( x./ map.res ) + map.grid_mid;

ind = round( (x-map.x_min)./map.res );

end

