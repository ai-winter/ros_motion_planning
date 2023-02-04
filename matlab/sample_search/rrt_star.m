function [path, flag, cost, expand] = rrt_star(map, start, goal)
%%
% @file: rrt_star.m
% @breif: RRT-Star motion planning
% @paper: Sampling-based algorithms for optimal motion planning
% @author: Winter
% @update: 2023.2.2

%%
    % optimal radius
    param.r = 30;
    % Maximum expansion distance one step
    param.max_dist = 0.5;
    % Maximum number of sample points
    param.sample_num = 10000;
    % heuristic sample
    param.goal_sample_rate = 0.05;
    % map size
    [param.x_range, param.y_range] = size(map);
    % resolution
    param.resolution = 0.1;
    
    % sample list
    sample_list  = [start, 0, start];
    
    path = [];
    flag = false;
    cost = 0;
    expand = [];
    
    % main loop
    for i=1: param.sample_num
        % generate a random node in the map
        node_rand = generate_node(goal, param);

        % visited
        if loc_list(node_rand, sample_list, [1, 2])
            continue
        end

        % generate new node
        [node_new, success] = get_nearest(sample_list, node_rand, map, param);
        if success
            sample_list = [node_new; sample_list];
            distance = dist(node_new(1:2), goal');
            
            % goal found
            if distance <= param.max_dist && ~is_collision(node_new(1:2), goal, map, param)
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

function node = generate_node(goal, param)
%breif: Generate a random node to extend exploring tree.
    if rand() > param.goal_sample_rate
        x = 0.5 + (param.x_range - 1) * rand();
        y = 0.5 + (param.y_range - 1) * rand();
        node = [x, y];
        return
    end
    
    node = goal;
    return
end

function [new_node, flag] = get_nearest(node_list, node, map, param)
%@breif: Get the node from `node_list` that is nearest to `node`.
    flag = false;
    % find nearest neighbor
    dist_vector = dist(node_list(:, 1:2), node');
    [~, index] = min(dist_vector);
    node_near = node_list(index, :);

    % regular and generate new node
    distance = min(dist(node_near(1:2), node'), param.max_dist);
    theta = angle(node_near, node);
    new_node = [node_near(1) + distance * cos(theta), ...
                           node_near(2) + distance * sin(theta), ...
                           node_near(3) + distance, ...
                           node_near(1:2)];

    % obstacle check
    if is_collision(new_node(1:2), node_near(1:2), map, param)
        return
    end
    
    %  rewire optimization
    [node_num, ~] = size(node_list);
    for i=1:node_num
        node_n = node_list(i, :);
        %  inside the optimization circle
        new_dist = dist(node_n(1:2), new_node(1:2)');
        if new_dist < param.r
            cost = node_n(3) + new_dist;
            %  update new sample node's cost and parent
            if new_node(3) > cost && ~is_collision(new_node(1:2), node_n(1:2), map, param)
                new_node(4:5) = node_n(1:2);
                new_node(3) = cost;
            else
                %  update nodes' cost inside the radius
                cost = new_node(3) + new_dist;
                if node_n(3) > cost && ~is_collision(new_node(1:2), node_n(1:2), map, param)
                    node_list(i, 4:5) = new_node(1:2);
                    node_list(i, 3) = cost;
                end
            end
        else
            continue;
        end
    end
    
    flag = true;
end

function flag = is_collision(node1, node2, map, param)
%@breif: Judge collision when moving from node1 to node2.
    flag = true;
    theta = angle(node1, node2);
    distance = dist(node1, node2');

      % distance longer than the threshold
      if (distance > param.max_dist)
          return
      end
        
      % sample the line between two nodes and check obstacle
      n_step = round(distance / param.resolution);
      for i=1:n_step
          x = node1(1) + i * param.resolution * cos(theta);
          y = node1(2) + i * param.resolution * sin(theta);
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