function [path, flag, cost, expand] = rrt_connect(map, start, goal)
%%
% @file: rrt_connect.m
% @breif: RRT-Connect motion planning
% @paper: RRT-Connect: An Efficient Approach to Single-Query Path Planning
% @author: Winter
% @update: 2023.2.3

%%
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
    sample_list_f  = [start, 0, start];
    sample_list_b = [goal, 0, goal];
    
    path = [];
    flag = false;
    cost = 0;
    expand = [];
    
    % main loop
    for i=1: param.sample_num
        % generate a random node in the map
        node_rand = generate_node(goal, param);

        % generate new node
        [node_new, success] = get_nearest(sample_list_f, node_rand, map, param);
        if success
            sample_list_f = [node_new; sample_list_f];
            % backward exploring
            [node_new_b, success_b]  = get_nearest(sample_list_b, node_new(1:2), map, param);
            if success_b
                 sample_list_b = [node_new_b; sample_list_b];
                 % greedy extending
                 while true
                     distance = min(param.max_dist, dist(node_new(1:2), node_new_b(1:2)'));
                     theta = angle(node_new_b, node_new);
                     node_new_b2 = [node_new_b(1) + distance * cos(theta), ...
                                                  node_new_b(2) + distance * sin(theta), ...
                                                  node_new_b(3) + distance, ...
                                                  node_new_b(1:2)];
                    if ~is_collision(node_new_b2(1:2), node_new_b(1:2), map, param)
                        sample_list_b = [node_new_b2; sample_list_b];
                        node_new_b = node_new_b2;
                    else
                        break
                    end
                    % goal found
                    if node_new_b(1) == node_new(1) && node_new_b(2) == node_new(2)
                        flag = true;
                        cost = sample_list_f(1, 3) + sample_list_b(1, 3);
                        path = extract_path(sample_list_f, sample_list_b, start, goal);
                        expand = [sample_list_f; sample_list_b];
                        return
                    end
                 end
            end
        end
        
        [len_f, ~] = size(sample_list_f); [len_b, ~] = size(sample_list_b);
        if len_b < len_f
            temp = sample_list_f;
            sample_list_f = sample_list_b;
            sample_list_b = temp;
        end
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

function path = extract_path(node_list_f, node_list_b, start, goal)
% @breif: Extract the path based on the CLOSED set.
    if isequal(node_list_b(end, 1:2), start)
            temp = node_list_f;
            node_list_f = node_list_b;
            node_list_b = temp;
    end
    
    path  = [];
    
    % forward
    [len_f, ~] = size(node_list_f);
    index = 1;
    while 1
        path = [node_list_f(index, 1:2); path];
        if isequal(node_list_f(index, 1:2), start)   
            break;
        end
        for i=1:len_f
            if isequal(node_list_f(i, 1:2), node_list_f(index, 4:5))
                index = i;
                break;
            end
        end
    end
    
    % backward
    [len_b, ~] = size(node_list_b);
    index = 1;
    while 1
        path = [path; node_list_b(index, 1:2)];
        if isequal(node_list_b(index, 1:2), goal)   
            break;
        end
        for i=1:len_b
            if isequal(node_list_b(i, 1:2), node_list_b(index, 4:5))
                index = i;
                break;
            end
        end
    end
end

 function theta = angle(node1, node2)
    theta = atan2(node2(2) - node1(2), node2(1) - node1(1));
 end