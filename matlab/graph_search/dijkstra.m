function [path, flag, cost, expand] = dijkstra(map, start, goal)
%%
% @file: dijkstra.m
% @breif: Dijkstra motion planning
% @author: Winter
% @update: 2023.1.29

%{ 
    ============ OPEN and CLOSED==============
    [x, y, g, h, px, py]
    =======================================
%}

%%
% initialize
expand= [];                                                   % expand zone
OPEN  = [start, 0, h(start, goal), start];      % OPEN set with priority 
CLOSED = [];                                                % CLOSED set
flag  = false;
neighbor  = [-1, 1, 1.414;...  
                0, 1, 1;...
                1, 1, 1.414;...
              -1, 0, 1;...
                1, 0, 1;...
              -1, -1, 1.414;...
                0, -1, 1;...
                1, -1, 1.414];
neighbor_num = length(neighbor(:, 1));
cost = 0;

while ~isempty(OPEN(:, 1))
    % pop
    [~, index] = min(OPEN(:, 3));
    cur_node = OPEN(index, :);
    OPEN(index, :) = []; 
    
    % exists in CLOSED set
    if loc_list(cur_node, CLOSED, [1, 2])
        continue
    end
    
    % goal found
    if cur_node(1) == goal(1) && cur_node(2) == goal(2)
        CLOSED = [cur_node; CLOSED];
        flag = true;
        cost = cur_node(3);
        break
    end
    
    % explore neighbors
    for i=1:neighbor_num        
        node_n = [cur_node(1) + neighbor(i, 1), ...
                          cur_node(2) + neighbor(i, 2), ...
                          cur_node(3) + neighbor(i, 3), ...
                          0, cur_node(1), cur_node(2)
                          ];
        node_n(4) = h(node_n(1:2), goal);
        
        % exists in CLOSED set
        if loc_list(cur_node, CLOSED, [1, 2])
            continue
        end
        
        % obstacle
        if map(node_n(1), node_n(2)) == 2
            continue;
        end
       
        % goal found
        if cur_node(1) == goal(1) && cur_node(2) == goal(2)
            CLOSED = [cur_node; CLOSED];
            flag = true;
            cost = cur_node(3);
            break
        end
        
        % update expand zone
        expand = [expand; node_n(1:2)];
        
        % update OPEN set
        OPEN = [OPEN; node_n];
    end
    CLOSED = [cur_node; CLOSED];
end

% extract path
path = extract_path(CLOSED, start);
end

%%
function h_val = h(cur_node, goal)
% @breif: heuristic function(Manhattan distance)
    h_val = abs(cur_node(1) - goal(1)) + abs(cur_node(2) - goal(2));
end

function index = loc_list(node, list, range)
% @breif: locate the node in given list
    num = size(list);
    index = 0;
    if ~num(1)
        return
    else  
        for i=1:num(1)
            if isequal(node(range), list(i, range))
                index = i;
                return;
            end
        end
    end
end

function path = extract_path(close, start)
% @breif: Extract the path based on the CLOSED set.
    path  = [];               
    closeNum = length(close(:, 1));
    index = 1;
    while 1
        path = [path; close(index, 1:2)];
        if isequal(close(index, 1:2), start)   
            break;
        end
        for i=1:closeNum
            if isequal(close(i, 1:2), close(index, 5:6))
                index = i;
                break;
            end
        end
    end
end
