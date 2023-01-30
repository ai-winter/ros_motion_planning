function plot_path(path, G)
%%
% @file: plot_path.m
% @breif: plot planning path
% @author: Winter
% @update: 2023.1.29

%%
    delta = G / 2;
    path_len = length(path);
    for i=1:path_len-1
        plot([path(i, 2) + delta, path(i + 1, 2) + delta], ...
        [path(i, 1) + delta, path(i + 1, 1) + delta] , 'Color', '#f00', 'LineStyle','--','LineWidth',1.5); 
    end
end

