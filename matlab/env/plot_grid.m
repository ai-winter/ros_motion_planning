function plot_grid(grid_map)
%% 
% @file: plot_grid.m
% @breif: plot grid map
% @author: Winter
% @update: 2023.1.29

%% color map
% 1 ------ empty
% 2 ------ obstacle
    cmap = [1 1 1; ...        
                    0 0 0; ...     
                  ];
    colormap(cmap);

    %%
    [rows, cols] = size(grid_map);
    image(1.5, 1.5, grid_map);
    grid on
    set(gca,'xtick', 1:cols+1, 'ytick', 1:rows+1);
    axis image;
    
    for row = 1:rows
        line([1, cols + 1], [row, row], 'Color', '#eee');
    end
    for col = 1:cols
            line([col, col], [1, rows + 1], 'Color', '#eee');
    end
end