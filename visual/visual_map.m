function [ I, F ] = visual_map( map, plot_hist )
%VISUAL_MAP Summary of this function goes here
%   Detailed explanation goes here

I = uint8(map.gridmap + 128);
imshow( I );

hold on
if size(plot_hist,1) == 1
    plot(plot_hist(end,2), plot_hist(end,1), 'o','LineWidth',3,'MarkerSize',5,'MarkerEdgeColor','r');
else
    plot(plot_hist(:,2), plot_hist(:,1),'r');
    plot(plot_hist(end,2), plot_hist(end,1), 'o','LineWidth',3,'MarkerSize',5,'MarkerEdgeColor','r');
end
hold off

%F = getframe(gcf);

end

