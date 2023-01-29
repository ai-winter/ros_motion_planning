function s = plot_square(pts, map_size, G, color)
%%
% @file: plot_square.m
% @breif: plot specific grid
% @author: Winter
% @update: 2023.1.29

%%
    [ptsX, ptsY] = index_to_map(pts(:, 1) + map_size(1) * (pts(:, 2) - 1), map_size, G);
    ptsNum = length(ptsX);
    for i=1:ptsNum
        s = scatter(ptsX, ptsY, 270, 'Marker', 'square', 'MarkerEdgeColor', color, ...
                 "MarkerFaceColor", color);
    end
end