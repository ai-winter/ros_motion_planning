function handler = plot_robot(pose, l, r, color)
%%
% @file: plot_robot.m
% @breif: plot given pose in figure
% @author: Winter
% @update: 2023.1.30

%%
    x = pose(1);
    y = pose(2);
    theta = pi / 2 - pose(3);

    refVector = [l; 0];
    rMatrix = [cos(theta), -sin(theta);
               sin(theta), cos(theta)];
    endPt = rMatrix * refVector + [x; y];

    % plot quiver
    arrow = quiver(x, y, endPt(1) - x, endPt(2) - y, ...
           'MaxHeadSize',5.5,'AutoScaleFactor',1,'AutoScale','off', 'LineWidth', 1.5, 'color', color, ...
           'Marker', 'o', 'MarkerSize', 4, 'MarkerFaceColor',color);
    hold on  

    % plot circle
    aplha = 0:pi/40:2*pi;
    x = x + r * cos(aplha);
    y = y + r * sin(aplha);
    circle = plot(x, y, 'LineWidth', 1.2, 'LineStyle', '-', 'color', color);

    handler = [arrow, circle];
    axis equal
end

