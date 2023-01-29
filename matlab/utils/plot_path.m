function plot_path(path, map_size, G)
%%
% @file: plot_path.m
% @breif: plot planning path
% @author: Winter
% @update: 2023.1.29

%%
    % grid to index
    path = path(:, 1) + map_size(1) * (path(:, 2) - 1);
    % index to map
    [path_x, path_y] = index_to_map(path, map_size, G);
    
    path_len = length(path);
    for i=1:path_len-1
        plot([path_x(i), path_x(i+1)], [path_y(i), path_y(i+1)] , 'Color', '#f00', 'LineStyle','--','LineWidth',1.5); 
    end
end

