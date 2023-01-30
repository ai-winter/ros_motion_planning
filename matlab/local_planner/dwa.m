function [pose, traj, flag] = dwa(map, start, goal, kinematic)
%%
% @file: dwa.m
% @breif: DWA motion planning
% @paper: The Dynamic Window Approach to Collision Avoidance
% @author: Winter
% @update: 2023.1.30

%%
    % return value
    flag = false;
    pose = [];
    traj = [];
    
    % initial robotic state
    robot.x = start(1);
    robot.y = start(2);
    robot.theta = start(3);
    robot.v = 0;
    robot.w = 0;
    
    % evalution parameters    [heading, distance, velocity, predict_time, dt, R]
    eval_param = [0.045, 0.1 ,0.1, 3.0, 0.1, 2.0];
    
    % obstacle
    [m, ~] = size(map);
    obs_index = find(map==2);
    obstacle = [mod(obs_index - 1, m) + 1, fix((obs_index - 1) / m) + 1];
    
    % threshold
    max_iter = 2000;
    max_dist = 1.0;
    
    % main loop
    for i=1:max_iter
        % dynamic configure
        vr = cal_dynamic_win(robot, kinematic, eval_param(5));
        [eval_win, traj_win] = evaluation(robot, vr, goal, obstacle, kinematic, eval_param);
        
        % failed
        if isempty(eval_win)
            return
        end
        
        % update
        value = eval_win(:, 3);
        [~, index] = max(value);
        u = eval_win(index, 1:2);
        robot = f(robot, u, eval_param(5));
        
        pose = [pose; robot.x, robot.y, robot.theta];
        traj_i.info = traj_win;
        traj = [traj; traj_i];
        
        % goal found
        if dist([robot.x, robot.y], goal(1:2)') < max_dist
            flag = true;
            disp("goal arrived!");
            break;
        end
    end
end

%%
function vr = cal_dynamic_win(robot, kinematic, dt)
%@breif: calculate dynamic window
    % hard margin
    vs=[0                               ,     kinematic.V_MAX, ...
           -kinematic.W_MAX,     kinematic.W_MAX];
    % predict margin
    vd = [robot.v - kinematic.V_ACC * dt  ,      robot.v + kinematic.V_ACC * dt, ...
              robot.w - kinematic.W_ACC * dt,      robot.w + kinematic.W_ACC * dt];
    % dynamic window
    v_tmp = [vs; vd];
    vr = [max(v_tmp(:, 1)) min(v_tmp(:, 2)) max(v_tmp(:, 3)) min(v_tmp(:, 4))];
end

function [eval_win, traj_win] = evaluation(robot, vr, goal, obstacle, kinematic, eval_param)
    eval_win = []; traj_win = [];
    for v = vr(1):kinematic.V_RESOLUTION:vr(2)
        for w=vr(3):kinematic.W_RESOLUTION:vr(4)
            % trajectory prediction, consistent of poses
            [robot_star, traj] = generate_traj(robot, v, w, eval_param(4), eval_param(5));
            
            % heading evaluation
            theta = angle([robot_star.x, robot_star.y], goal(1:2));
            heading = pi - abs(robot_star.theta - theta);

            % obstacle evaluation
            dist_vector = dist(obstacle, [robot_star.x; robot_star.y]);
            distance = min(dist_vector);
            if distance > eval_param(6)
                distance = eval_param(6);
            end

            % velocity evaluation
            velocity = abs(v);
            
            % braking evaluation
            dist_stop = v * v / (2 * kinematic.V_ACC);

            % collision check
            if distance > dist_stop && distance >= 1
                eval_win = [eval_win; [v w heading distance velocity]];
                traj_win = [traj_win; traj];
            end
        end
    end
    
    % normalization
    if sum(eval_win(:, 3)) ~= 0
        eval_win(:, 3) = eval_win(:, 3) / sum(eval_win(:, 3));
    end
    if sum(eval_win(:, 4)) ~= 0
        eval_win(:, 4) = eval_win(:, 4) / sum(eval_win(:, 4));
    end
    if sum(eval_win(:, 5)) ~= 0
        eval_win(:, 5) = eval_win(:, 5) / sum(eval_win(:, 5));
    end
    eval_win = [eval_win(:, 1:2), eval_win(:, 3:5) * eval_param(1:3)'];
end

function [robot, traj] = generate_traj(robot, v, w, t, dt)
%@breif: generate trajectory
    time = 0;
    u = [v, w];
    traj = robot;
    while time <= t   
        time = time + dt;
        robot = f(robot, u, dt);
        traj = [traj robot];
    end
end

function robot = f(robot, u, dt)
%@breif: robotic kinematic
    F = [ 1 0 0 0 0
             0 1 0 0 0
             0 0 1 0 0
             0 0 0 0 0
             0 0 0 0 0];
 
    B = [dt * cos(robot.theta) 0
            dt * sin(robot.theta)  0
            0                                dt
            1                                 0
            0                                 1];
 
    x = [robot.x; robot.y; robot.theta; robot.v; robot.w];
    x_star = F * x + B * u';
    robot.x = x_star(1); robot.y = x_star(2); robot.theta = x_star(3);
    robot.v = x_star(4); robot.w = x_star(5);
end

 function theta = angle(node1, node2)
    theta = atan2(node2(2) - node1(2), node2(1) - node1(1));
 end