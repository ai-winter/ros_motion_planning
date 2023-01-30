function [path, flag, cost, expand] = jps(map, start, goal)
%%
% @file: jps.m
% @breif: Jump Point Search motion planning
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
    f = OPEN(:, 3) + OPEN(:, 4);
    [~, index] = min(f);
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
    
    jp_list = [];
    for i=1:neighbor_num
        [jp, flag] = jump(cur_node, neighbor(i, :), goal, map);
        % exists and not in CLOSED set
        if flag && ~loc_list(jp, CLOSED, [1, 2])
            jp(5:6) = cur_node(1:2);
            jp(4) = h(jp(1:2), goal);
            jp_list = [jp; jp_list];
        end
    end
        
    for j = 1:size(jp_list, 1)
        jp = jp_list(j, :);
        % update OPEN set
        OPEN = [OPEN; jp];
        expand = [expand; jp];

        % goal found
        if jp(1) == goal(1) && jp(2) == goal(2)
            break
        end
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

function [new_node, flag] = jump(cur_node, motion, goal, map)
    flag = false;

    % explore a new node
    new_node = [cur_node(1) + motion(1), ...
                      cur_node(2) + motion(2), ...
                      cur_node(3) + motion(3), ...
                      0, cur_node(1), cur_node(2)
                      ];
    new_node(4) = h(new_node(1:2), goal);

    % obstacle
    if new_node(1) <= 0 || new_node(2) <= 0 || map(new_node(1), new_node(2)) == 2
        return
    end

    % goal found
    if new_node(1) == goal(1) && new_node(2) == goal(2)
        flag = true;
        return
    end

    % diagonal
    if motion(1) && motion(2)
        % if exists jump point at horizontal or vertical
        x_dir = [motion(1), 0, 1];
        y_dir = [0, motion(2), 1];
        [~, flag_x] = jump(new_node, x_dir, goal, map);
        [~, flag_y] = jump(new_node, y_dir, goal, map);
        if  flag_x || flag_y 
            flag = true;
            return
        end
    end
            
    % if exists forced neighbor
    if detect_force(new_node, motion, map)
        flag = true;
        return
    else
        [new_node, flag] = jump(new_node, motion, goal, map);
        return
    end
end

function flag = detect_force(cur_node, motion, map)
    flag = true;
    x = cur_node(1);
    y = cur_node(2);
    x_dir = motion(1);
    y_dir = motion(2);

    % horizontal
    if x_dir && ~y_dir
        if map(x, y + 1) == 2 && map(x + x_dir, y + 1) ~= 2
            return
        end
        if map(x, y - 1) == 2 && map(x + x_dir, y - 1) ~= 2
            return
        end
    end
    
    % vertical
    if ~x_dir && y_dir
        if map(x + 1, y) == 2 && map(x + 1, y + y_dir) ~= 2
            return
        end
        if  map(x - 1, y) == 2 && map(x - 1, y + y_dir) ~= 2
            return
        end
    end

    % diagonal
    if x_dir && y_dir
        if map(x - x_dir, y) == 2 && map(x - x_dir, y + y_dir) ~= 2
            return
        end
        if map(x, y - y_dir) == 2 && map(x + x_dir, y - y_dir) ~= 2
            return
        end
    end

    flag = false;
    return
end