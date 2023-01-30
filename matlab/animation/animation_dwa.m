function animation_dwa(pose, traj, delta, record_video)
%%
% @file: animation_dwa.m
% @breif: DWA algorithm animation
% @author: Winter
% @update: 2023.1.30

%%
    hold on
    [frames, ~] = size(pose);

    if record_video
        process = VideoWriter('./animation/video/dwa.mp4', 'MPEG-4');
        open(process);
        movie = moviein(frames);
    end

    for i=1:frames
        handler = plot_robot([pose(i, 2) + delta, pose(i, 1) + delta, pose(i, 3)], 0.8, 0.4, 'r');
        handler2 = plot_trajectory(traj(i).info, delta);
        plot(pose(i, 2) + delta, pose(i, 1) + delta, 'Marker', '.', 'color', "#f00");
        drawnow;
        if record_video
            movie(:, i) = getframe;
            writeVideo(process, movie(:, i));
        end
        delete(handler);
        delete(handler2);
    end
    
    if record_video
        close(process);
    end
end

%%
function handler = plot_trajectory(traj, delta)
    handler = [];
    [m, n] = size(traj);
    for i=1:2:m
        traj_i = traj(i, :);
        for j=1:3:n-1
            h = plot([traj_i(j).y + delta, traj_i(j + 1).y + delta], ...
                [traj_i(j).x + delta, traj_i(j + 1).x + delta], ...
                'Color', '#ddd', 'LineStyle','-','LineWidth',1.5); 
            handler = [handler; h];
        end   
    end   
end
