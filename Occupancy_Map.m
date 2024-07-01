function [x_grid, y_grid, z_heights, X, Y] = Occupancy_Map(row, col, type)
%OCCUPANCY_MAP Generate an occupancy map with different terrain types
%   [X, Y, z_heights] = Occupancy_Map(row, col, type) generates a 2D
%   occupancy map of size 'row' by 'col' with the specified terrain 'type'.
%   Types: 1 - Sine wave terrain
%          2 - Flat terrain
%          3 - Random terrain

    % Define the grid
    x_grid = -1:0.1:row;
    y_grid = -1:0.1:col;

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
            % a% Slope on X
            z_heights = (0.3.*X)';
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
            obstacle_x = randi(length(x_grid) - obstacle_length);
            obstacle_y = randi(length(y_grid) - obstacle_width);
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
            obstacle_x = randi(length(x_grid) - obstacle_length, n1+n2, 1);
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
            obstacle_x2 = (length(x_grid) - obstacle_length)/3;
            obstacle_y2 = (length(y_grid) - obstacle_width)/4;
            z_heights(obstacle_y1:obstacle_y1+obstacle_width,obstacle_x1:obstacle_x1+obstacle_length) = h;
            %   z_heights(obstacle_y2:obstacle_y2+obstacle_width,obstacle_x2:obstacle_x2+obstacle_length) = h;
            z_heights = z_heights';
        otherwise
            error('Invalid terrain type. Choose 1 to 8.');
    end

end

% function Z = building(x, y, h, length, width)
% 
% % building z coordinate
% Z(y:y+width,x:x+length) = h;
% Z = Z';
% 
% end



