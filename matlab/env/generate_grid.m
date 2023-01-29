function grid_map = generate_grid(size, obstacle)
%%
% @file: generate_grid.m
% @breif: generate grid map
% @author: Winter
% @update: 2023.1.29

%%
% 1 ------ empty
% 2 ------ obstacle

%%
    grid_map = ones(size(1), size(2));
    grid_map(obstacle) = 2;
end
