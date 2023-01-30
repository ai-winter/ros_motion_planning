function plot_expand(expand, map_size, G, planner_name)
%% 
% @file: plot_expand.m
% @breif: plot expand zone
% @author: Winter
% @update: 2023.1.30

%%
    if strcmp(planner_name, 'a_star') || ...
       strcmp(planner_name, 'gbfs') || ...
       strcmp(planner_name, 'dijkstra') || ...
       strcmp(planner_name, 'jps')
        plot_square(expand, map_size, G, "#ddd");
    end

    if strcmp(planner_name, 'rrt')
        delta = G / 2;
        [expand_len, ~] = size(expand);
        for i=1:expand_len-1
            plot([expand(i, 2) + delta, expand(i, 5) + delta], ...
            [expand(i, 1) + delta, expand(i, 4) + delta] , 'Color', '#ddd', 'LineStyle','--','LineWidth',1.5); 
        end
    end
end

