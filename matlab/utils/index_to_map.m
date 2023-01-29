function [x, y] = index_to_map(index, map_size, G)
%% 
% @file: index_to_map.m
% @breif: transform from grid index to map coordinate
% @author: Winter
% @update: 2023.1.29

%%
    x = (index - mod(index-1, map_size(1)) + 1) / map_size(1) + G / 2 + 0.9;
    y = mod(index-1, map_size(1)) + G / 2 + 1;
end

