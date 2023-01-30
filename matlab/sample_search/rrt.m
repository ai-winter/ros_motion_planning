function [path, flag, cost, expand] = rrt(map, start, goal)
%%
% @file: rrt.m
% @breif: RRT motion planning
% @paper: Rapidly-Exploring Random Trees: A New Tool for Path Planning
% @author: Winter
% @update: 2023.1.30

%%
    % Maximum expansion distance one step
    max_dist = 0.5;
    % Maximum number of sample points
    sample_num = 10000;
    % heuristic sample
    goal_sample_rate = 0.05;
    % sample list
    sample_list  = [start, 0, start];
    % map size
    [x_range, y_range] = size(map);
    % resolution
    resolution = 0.1;
    
    path = [];
    flag = false;
    cost = 0;
    expand = [];
    
    % main loop
    for i=1: sample_num
        % generate a random node in the map
        node_rand = generate_node(goal_sample_rate, goal, x_range, y_range);

        % visited
        if loc_list(node_rand, sample_list, [1, 2])
            continue
        end

        % generate new node
        [node_new, success] = get_nearest(sample_list, node_rand, map, max_dist, resolution);
        if success
            sample_list = [node_new; sample_list];
            distance = dist(node_new(1:2), goal');
            
            % goal found
            if distance <= max_dist && ~is_collision(node_new(1:2), goal, map, max_dist, resolution)
                goal_ = [goal, node_new(3) + distance, node_new(1:2)];
                sample_list = [goal_; sample_list];
                flag = true;
                cost = goal_(3);
                break
            end
        end
    end
    
    
    if flag
        path = extract_path(sample_list, start);
        expand = sample_list;
    end
end

%%
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

function node = generate_node(goal_sample_rate, goal, x_range, y_range)
%breif: Generate a random node to extend exploring tree.
    if rand() > goal_sample_rate
        x = 0.5 + (x_range - 1) * rand();
        y = 0.5 + (y_range - 1) * rand();
        node = [x, y];
        return
    end
    
    node = goal;
    return
end

function [new_node, flag] = get_nearest(node_list, node, map, max_dist, resolution)
%@breif: Get the node from `node_list` that is nearest to `node`.
    flag = false;
    % find nearest neighbor
    dist_vector = dist(node_list(:, 1:2), node');
    [~, index] = min(dist_vector);
    node_near = node_list(index, :);

    % regular and generate new node
    distance = min(dist(node_near(1:2), node'), max_dist);
    theta = angle(node_near, node);
    new_node = [node_near(1) + distance * cos(theta), ...
                           node_near(2) + distance * sin(theta), ...
                           node_near(3) + distance, ...
                           node_near(1:2)];

    % obstacle check
    if is_collision(new_node(1:2), node_near(1:2), map, max_dist, resolution)
        return
    end
    flag = true;
end

function flag = is_collision(node1, node2, map, max_dist, resolution)
%@breif: Judge collision when moving from node1 to node2.
    flag = true;
    theta = angle(node1, node2);
    distance = dist(node1, node2');

      % distance longer than the threshold
      if (distance > max_dist)
          return
      end
        
      % sample the line between two nodes and check obstacle
      n_step = round(distance / resolution);
      for i=1:n_step
          x = node1(1) + i * resolution * cos(theta);
          y = node1(2) + i * resolution * sin(theta);
          if map(round(x), round(y)) == 2
              return
          end
      end
      
      flag = false;
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
            if isequal(close(i, 1:2), close(index, 4:5))
                index = i;
                break;
            end
        end
    end
end

 function theta = angle(node1, node2)
    theta = atan2(node2(2) - node1(2), node2(1) - node1(1));
 end