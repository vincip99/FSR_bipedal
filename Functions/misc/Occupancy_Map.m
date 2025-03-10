function [x_grid, y_grid, z_heights, map] = Occupancy_Map(row, col, type, q_start, q_goal)
%OCCUPANCY_MAP Generate an occupancy map with different terrain types
%   [X, Y, z_heights] = Occupancy_Map(row, col, type) generates a 2D
%   occupancy map of size 'row' by 'col' with the specified terrain 'type'.
%   Types: 1 - Sine wave terrain
%          2 - Flat terrain
%          3 - Random terrain

    % Define the grid
    step = 0.1;
    bias = -1;
    x_grid = bias:step:col;
    y_grid = bias:step:row;

    % Generate the mesh grid
    [X, Y] = meshgrid(x_grid, y_grid);

    % Generate the terrain based on the type
    switch type
        case 1 
            % Sine wave terrain
            z_heights = (0.05 * cos(2 * pi * X) - 0.05)';
        case 2
            % Flat terrain
            z_heights = (zeros(size(X)))';
        case 3
            z_heights = zeros(size(X))';
            % Define circular obstacle parameters
            center_x = floor(length(x_grid) / 2);
            center_y = floor(length(y_grid) / 4);
            radius = min(floor(length(x_grid) / 10), floor(length(y_grid) / 10)); % radius of the circle
            h = 1; % height of the obstacle
            
            % Create the circular obstacle
            for i = 1:size(X, 1)
                for j = 1:size(X, 2)
                    if (X(i, j) - center_x)^2 + (Y(i, j) - center_y)^2 <= radius^2
                        z_heights(i, j) = h;
                    end
                end
            end
        case 4
            % step terrain
            z_heights = (zeros(size(X)));
            n = 5; % adjust number of step
            step_length = floor(length(x_grid)/n);
            for i = 0:floor(n/2)+1
                z_heights(:,(i+1)*step_length+1:(i+2)*step_length) = i*0.025;
            end
            for i = 1:floor(n/2)
                z_heights(:,(i+1)*step_length+1:(i+2)*step_length) = (floor(n/2)+1)*0.025 - i*0.025;
            end
            z_heights = z_heights';
        case 5
            % random obstacle
            z_heights = (zeros(size(X)));
            h = 1;
            obstacle_length = floor(length(x_grid)/15);
            obstacle_width = floor(length(y_grid)/15);
            obstacle_x = floor(randi(length(x_grid) - obstacle_length));
            obstacle_y = floor(randi(length(y_grid) - obstacle_width));
            z_heights(obstacle_y:obstacle_y+obstacle_width,obstacle_x:obstacle_x+obstacle_length) = h;
            z_heights = z_heights';
        case 6
            % Walls
            z_heights = (zeros(size(X)));
            h = 1; % Walls height
            n1 = 2; % Obstacles walls first side
            n2 = 2; % Obstacles walls opposite side
            obstacle_length = floor(length(y_grid)/randi([2 4]));
            obstacle_width = floor(length(y_grid)/20);
            obstacle_x = floor(randi(length(x_grid) - obstacle_length, n1+n2, 1));
            % lateral Walls
            z_heights(1:1+obstacle_width,:) = h;
            z_heights(end-obstacle_width:end,:) = h;
            % obstacle Walls
            for i = 1:n1
                z_heights(1:obstacle_length,obstacle_x(i):obstacle_x(i)+obstacle_width) = h;
            end
            for i = n1+1:n1+n2
                z_heights(end-obstacle_length:end,obstacle_x(i):obstacle_x(i)+obstacle_width) = h;
            end
            z_heights = z_heights';
        case 7
            % random obstacle
            z_heights = (zeros(size(X)));
            h = 1;
            n = 8; % number of obstacles
            obstacle_length = floor(length(x_grid)/15);
            obstacle_width = floor(length(y_grid)/15);
            obstacle_x = randi(length(x_grid) - obstacle_length, n-1, 1);
            obstacle_y = randi(length(y_grid) - obstacle_width, n-1, 1);
            obstacle_x(n,1) = 40; obstacle_y(n,1) = 7;
            for i = 1:n
                z_heights(obstacle_y(i):obstacle_y(i)+obstacle_width,obstacle_x(i):obstacle_x(i)+obstacle_length) = h;
            end
            z_heights = z_heights';
        case 8
            % random obstacle
            z_heights = (zeros(size(X)));
            h = 1;
            obstacle_length = floor(length(x_grid)/15);
            obstacle_width = floor(length(y_grid)/15);
            obstacle_x1 = (length(x_grid) - obstacle_length)/2;
            obstacle_y1 = (length(y_grid) - obstacle_width)/2;
            z_heights(obstacle_y1:obstacle_y1+obstacle_width,obstacle_x1:obstacle_x1+obstacle_length) = h;
            z_heights = z_heights';
        case 9
            % random obstacle
            z_heights = (zeros(size(X)));
            h = 1;
            obstacle_length = floor(length(x_grid)/15);
            obstacle_width = floor(length(y_grid)/15);
            obstacle_x1 = (length(x_grid) - obstacle_length)/1.9;
            obstacle_y1 = (length(y_grid) - obstacle_width)/2;
            obstacle_x2 = (length(x_grid) - obstacle_length)/3;
            obstacle_y2 = (length(y_grid) - obstacle_width)/6;
            obstacle_x3 = (length(x_grid) - obstacle_length)/1.3;
            obstacle_y3 = (length(y_grid) - obstacle_width)/1.2;
            obstacle_x4 = (length(x_grid) - obstacle_length)/4.5;
            obstacle_y4 = (length(y_grid) - obstacle_width)/1.1;
            obstacle_x5 = (length(x_grid) - obstacle_length)/1.3;
            obstacle_y5 = (length(y_grid) - obstacle_width)/6;
            z_heights(obstacle_y1:obstacle_y1+obstacle_width,obstacle_x1:obstacle_x1+obstacle_length) = h;
            z_heights(obstacle_y2:obstacle_y2+obstacle_width,obstacle_x2:obstacle_x2+obstacle_length) = h;
            z_heights(obstacle_y3:obstacle_y3+obstacle_width,obstacle_x3:obstacle_x3+obstacle_length) = h;
            z_heights(obstacle_y4:obstacle_y4+obstacle_width,obstacle_x4:obstacle_x4+obstacle_length) = h;
            z_heights(obstacle_y5:obstacle_y5+obstacle_width,obstacle_x5:obstacle_x5+obstacle_length) = h;
            z_heights = z_heights';
        case 10
            z_heights = (zeros(size(X)));
            % Define the height of the walls
            h = 1;
            
            % Define the thickness of the walls
            wall_thickness = floor(length(y_grid)/10);
            
            % Outer walls
            z_heights(1:wall_thickness, :) = h; % Top wall
            z_heights(end-wall_thickness+1:end, :) = h; % Bottom wall
            z_heights(:, 1:wall_thickness) = h; % Left wall
            z_heights(:, end-wall_thickness+1:end) = h; % Right wall

            obstacle_length = floor(length(x_grid)/15);
            obstacle_width = floor(length(y_grid)/5);
            
            % Define interior walls 
            interior_wall_1_x = floor(length(x_grid)/3):floor(length(x_grid)/3 + length(x_grid)/10);
            interior_wall_1_y = floor(length(y_grid)/1.5):floor(length(y_grid)/1.5) + wall_thickness - 1;

            interior_wall_2_x = floor(length(x_grid)/3):floor(length(x_grid)/3) + wall_thickness - 1;
            interior_wall_2_y = floor(length(y_grid)/1.5):floor(length(y_grid));

            interior_wall_3_x = floor(length(x_grid)/3):floor(length(x_grid)/3) + wall_thickness - 1;
            interior_wall_3_y = 1:floor(length(y_grid)/4);
    
            interior_wall_4_x = floor(length(x_grid)/1.15):floor(length(x_grid)/1.15) + wall_thickness - 1;
            interior_wall_4_y = 1:floor(length(y_grid)/2.45);

            interior_wall_5_x = floor(length(x_grid)/1.4):floor(length(x_grid)/1.4) + wall_thickness - 1;
            interior_wall_5_y = floor(length(y_grid)/1.5):floor(length(y_grid));

            % Put an obstacle
            obstacle_x1 = floor(length(x_grid)/1.7):floor(length(x_grid)/1.7) + obstacle_length - 1;
            obstacle_y1 = floor(length(y_grid)/1.9):floor(length(y_grid)/1.9) + obstacle_width - 1;

            obstacle_x2 = floor(length(x_grid)/2.4):floor(length(x_grid)/2.4) + obstacle_length - 1;
            obstacle_y2 = floor(length(y_grid)/6):floor(length(y_grid)/6) + obstacle_width - 1;

            obstacle_x3 = floor(length(x_grid)/1.2):floor(length(x_grid)/1.2) + obstacle_length - 1;
            obstacle_y3 = floor(length(y_grid)/1.3):floor(length(y_grid)/1.3) + obstacle_width - 1;

            % Add interior walls to the grid
            z_heights(interior_wall_1_y, interior_wall_1_x) = h;
            z_heights(interior_wall_2_y, interior_wall_2_x) = h;
            z_heights(interior_wall_3_y, interior_wall_3_x) = h;
            z_heights(interior_wall_4_y, interior_wall_4_x) = h;
            z_heights(interior_wall_5_y, interior_wall_5_x) = h;
            z_heights(obstacle_y1, obstacle_x1) = h/2;
            z_heights(obstacle_y2, obstacle_x2) = h/3;
            z_heights(obstacle_y3, obstacle_x3) = h/3;
            
            % Transpose the grid for correct orientation
            z_heights = z_heights';
        case 11
            z_heights = (zeros(size(X)));
            % Define the height of the walls
            h = 1;
            
            % Define the thickness of the walls
            wall_thickness = 2;
            
            % Outer walls
            z_heights(1:wall_thickness, :) = h; % Top wall
            z_heights(end-wall_thickness+1:end, :) = h; % Bottom wall
            z_heights(:, 1:wall_thickness) = h; % Left wall
            z_heights(:, end-wall_thickness+1:end) = h; % Right wall

            obstacle_length = floor(length(x_grid)/15);
            obstacle_width = floor(length(y_grid)/5);
            
            % Define interior walls 
            interior_wall_1_x = 12:20;
            interior_wall_1_y = 14:14 + wall_thickness - 1;

            interior_wall_2_x = 12:12 + wall_thickness - 1;
            interior_wall_2_y = 14:20;

            interior_wall_3_x = 20:20 + wall_thickness - 1;
            interior_wall_3_y = 1:6;
            
            interior_wall_4_x = 45:45 + wall_thickness - 1;
            interior_wall_4_y = 1:5;

            interior_wall_5_x = 45:45 + wall_thickness - 1;
            interior_wall_5_y = 15:20;
    
            % Put an obstacle
            obstacle_x1 = 35:35 + obstacle_length - 1;
            obstacle_y1 = 8:8 + obstacle_width - 1;

            obstacle_x2 = 26:26 + obstacle_length - 1;
            obstacle_y2 = 13:13 + obstacle_width - 1;

            % Add interior walls to the grid
            z_heights(interior_wall_1_y, interior_wall_1_x) = h;
            z_heights(interior_wall_2_y, interior_wall_2_x) = h;
            z_heights(interior_wall_3_y, interior_wall_3_x) = h;
            z_heights(interior_wall_4_y, interior_wall_4_x) = h;
            z_heights(interior_wall_5_y, interior_wall_5_x) = h;
            z_heights(obstacle_y1, obstacle_x1) = h/2;
            %z_heights(obstacle_y2, obstacle_x2) = h/3;
            
            % Transpose the grid for correct orientation
            z_heights = z_heights';
        case 12
            z_heights = zeros(size(X));
            h = 1;
            obstacle_length = floor(length(x_grid)/15);
            obstacle_width = floor(length(y_grid)/15);
            
            obstacle_x1 = (length(x_grid) - obstacle_length)/1.6;
            obstacle_y1 = (length(y_grid) - obstacle_width)/1.2;
            obstacle_x2 = (length(x_grid) - obstacle_length)/9;
            obstacle_y2 = (length(y_grid) - obstacle_width)/1;
            obstacle_x3 = (length(x_grid) - obstacle_length)/2.8;
            obstacle_y3 = (length(y_grid) - obstacle_width)/3;
            obstacle_x4 = (length(x_grid) - obstacle_length)/1.1;
            obstacle_y4 = (length(y_grid) - obstacle_width)/1.2;
            obstacle_x5 = (length(x_grid) - obstacle_length)/1.5;
            obstacle_y5 = (length(y_grid) - obstacle_width)/5;
            
            z_heights(obstacle_y1:obstacle_y1+obstacle_length, obstacle_x1:obstacle_x1+obstacle_length) = h;
            z_heights(obstacle_y2:obstacle_y2+obstacle_width, obstacle_x2:obstacle_x2+obstacle_length) = h;
            z_heights(obstacle_y5:obstacle_y5+obstacle_length/1.5, obstacle_x5:obstacle_x5+obstacle_length*1.5) = h;

            % Create a circular obstacle for the fourth one
            center_x4 = obstacle_x4 + obstacle_length / 2;
            center_y4 = obstacle_y4 + obstacle_length / 2;
            radius4 = obstacle_length / 1.5;
            
            for i = 1:size(z_heights, 1)
                for j = 1:size(z_heights, 2)
                    if (i - center_y4)^2 + (j - center_x4)^2 <= radius4^2
                        z_heights(i, j) = h;
                    end
                end
            end
            
            % Create a circular obstacle for the third one
            center_x3 = obstacle_x3 + obstacle_length / 2;
            center_y3 = obstacle_y3 + obstacle_length / 2;
            radius3 = obstacle_length / 2;
            
            for i = 1:size(z_heights, 1)
                for j = 1:size(z_heights, 2)
                    if (i - center_y3)^2 + (j - center_x3)^2 <= radius3^2
                        z_heights(i, j) = h;
                    end
                end
            end         
            
            z_heights = z_heights';

        otherwise
            error('Invalid terrain type. Choose 1 to 10 map type.');
    end

    % Create a binary map from z_heights
    % Rotate the z_heights matrix 90 degrees counter-clockwise
    grid = rot90(z_heights) > 0;
    grid = double(grid);

    % Inflate map for correct motion planning
    inflate = q_start(3)/step;
    obstaclePos = find(grid ~= 0);
    [i, j] = ind2sub(size(grid), obstaclePos);
    obstaclePos = [i'; j'];

    configGrid = grid;
    % Expand the obstacle 
    for i = 1:size(obstaclePos,2)
        for j = -floor(inflate/2):floor(inflate/2)
            for k = -floor(inflate/2):floor(inflate/2)
                if obstaclePos(1,i)+j >= 1 && obstaclePos(1,i)+j <= size(grid,1) && ...
                        obstaclePos(2,i)+k >= 1 && obstaclePos(2,i)+k <= size(grid,2)
                    configGrid(obstaclePos(1,i)+j,obstaclePos(2,i)+k) = 1;
                end
            end
        end
    end   

    % Define the fraction of 1's to keep
    fractionToKeep = 0.6; % Adjust this value as needed (e.g., 0.5 to keep 50% of the ones)
    
    % Find the positions of the 1's
    [oneRows, oneCols] = find(grid == 1);
    numOnes = length(oneRows);
    
    % Determine the number of 1's to keep
    numToKeep = round(numOnes * fractionToKeep);
    
    % Randomly select indices to keep
    indicesToKeep = randperm(numOnes, numToKeep);
    
    % Create a new grid with fewer 1's
    newGrid = zeros(size(grid));
    for i = 1:numToKeep
        newGrid(oneRows(indicesToKeep(i)), oneCols(indicesToKeep(i))) = 1;
    end

    lGrid = newGrid;

    % Define a map struct
    map = struct('grid', grid,'cgrid', configGrid, 'lgrid', lGrid, 'row', row, 'column', col, 'step', step,...
        'bias', bias);
    
    % Define a 3d map struct
    map3 = struct('X', X, 'Y', Y, 'Z', z_heights, 'cZ', rot90(configGrid,-1));

    % Plot 3D map
    figure(1)
    plotOccupancy(X, Y, z_heights, type, q_start, q_goal)
    
    % Plot Config. space map
    figure(2)
    plotOccupancy(X, Y, rot90(configGrid,-1), type, q_start, q_goal)

    % Plot 3D mesh
    % figure(3)
    % plotMesh(X, Y, z_heights, type, q_start, q_goal)
end

function plotOccupancy(X, Y, z_heights, type, q_start, q_goal)
    
    % Plotting the map
    surf(X, Y, z_heights', 'EdgeColor', 'none'); % Plot the surface without grid lines
    hold on;

    % Mark the start and goal points
    scatter3(q_start(1), q_start(2), q_start(3), 100, 'g', 'filled', 'DisplayName', 'Start');
    text(q_start(1), q_start(2), q_start(3), '  Start', 'VerticalAlignment', 'bottom', 'FontSize', 12);
    scatter3(q_goal(1), q_goal(2), q_goal(3), 100, 'm', 'filled', 'DisplayName', 'Goal');
    text(q_goal(1), q_goal(2), q_goal(3), '  Goal', 'VerticalAlignment', 'bottom', 'FontSize', 12);
    
    % Enhance the plot
    title(sprintf('Generated Terrain Type %d', type), 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X-axis', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y-axis', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Height', 'FontSize', 12, 'FontWeight', 'bold');

    % Add legends
    legend('Terrain', 'Start Point', 'Goal Point', 'Location', 'BestOutside');

    % Define custom colormap
    custom_colormap = [0.3 0.3 0.3;  % Black (bottom)
                       0.5 0.5 0.5; % Grey for walls
                       0.8 0.4 0.2; % Brown for obstacles
                       0.8 0.4 0.2; % Brown for obstacles
                       0.8 0.4 0.2; % Brown for obstacles
                       1 1 1]; % White (top)

    colormap(custom_colormap); % Set the colormap
    % colormap(jet); % Set the colormap
    colorbar; % Add a colorbar to indicate height
    axis tight; % Remove extra whitespace
    axis equal; % Equal scaling for all axes
    
    % Adjust lighting and material properties
    shading interp;
    lighting gouraud;
    material shiny;
    lightangle(-45, 30);
    
    % Additional enhancements
    set(gca, 'FontSize', 12, 'FontWeight', 'bold'); % Set the font size and weight for the axes
    hold off;

end

function plotMesh(X, Y, z_heights, type, q_start, q_goal)
% Plot the terrain map
    hold on;
    mesh(X, Y, z_heights', 'FaceAlpha', 0.7);

    % Plot start and goal points
    plot3(q_start(1), q_start(2), q_start(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(q_start(1), q_start(2), q_start(3), '  Start', 'VerticalAlignment', 'bottom', 'FontSize', 12);
    plot3(q_goal(1), q_goal(2), q_goal(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(q_goal(1), q_goal(2), q_goal(3), '  Goal', 'VerticalAlignment', 'bottom', 'FontSize', 12);

    % Customize the color map
    colormap(jet);
    colorbar;

    % Adjust lighting and material properties
    lighting gouraud;
    material shiny;
    lightangle(-45, 30);
    % Set the initial view to isometric
    view(3);
    
    % Improved labels and title
    xlabel('X-axis (meters)');
    ylabel('Y-axis (meters)');
    zlabel('Height (meters)');
    title(['Occupancy Map with Terrain Type ' num2str(type)]);

    % Add legends
    legend('Terrain', 'Start Point', 'Goal Point', 'Location', 'BestOutside');

    hold off;
end




