function obstacle = sensing(u)

q = reshape(u(1:2), 2, 1);
map = reshape(u(3:end), 21, 61);

% Convert from the meter world to the grid world
q_grid = grid_mapping(q);

% Sensing distance action
r_grid = 10;

% Initializing an obstacle far away for the algorithm
inf_obstacle = 100000;
% obstacle = [inf_obstacle; inf_obstacle];
max_obstacles = 400;
obstacle = inf_obstacle * ones(2, max_obstacles); % Initialize with inf

% Condition simulating the sight of the robot,aka lidar's range
index = 1;
for i = q_grid(2)-r_grid:q_grid(2)+r_grid
    for j = q_grid(1)-r_grid:q_grid(1)+r_grid
        % into the map
        if i > 1 && i < size(map,1) && j > 1 && j < size(map,2)
            if map(i,j) == 1
                %Save each obstacle seen by the sensor
                obstacle(:,index) = grid_inverse_mapping([j;i]);
                index = index + 1;
            end
            % boundary of the map as obstacles
        elseif ((i == 1 || i == size(map, 1)) && (j >= 1 && j <= size(map, 2))) ...
                || ((j == 1 || j == size(map, 2)) && (i >= 1 && i < size(map, 1)))
            obstacle(:, index) = grid_inverse_mapping([j; i]);
            index = index + 1;
       end
    end
end

obstacle = reshape(obstacle, 1, 2*max_obstacles);

end

function q_index = grid_mapping(q)
%GRID_MAPPING Summary of this function goes here
%   Detailed explanation goes here
q_index = [round((q(1) + 1)/0.1 + 1, 0); round((-q(2) + 1)/0.1 + 1)];
end

function q_meter = grid_inverse_mapping(q)
%GRID_INVERSE_MAPPING Summary of this function goes here
%   Detailed explanation goes here
q_meter = [0.1*(q(1) - 1) - 1; -0.1*(q(2) - 1) + 1];
end

% function q_index = grid_mapping(q)
% %GRID_MAPPING Summary of this function goes here
% %   Detailed explanation goes here
% q_index = [round(10*q(1) + 11, 0); round(-10*q(2) + 11, 0)];
% end
% 
% function q_meter = grid_inverse_mapping(q)
% %GRID_INVERSE_MAPPING Summary of this function goes here
% %   Detailed explanation goes here
% q_meter = [(q(1) - 11)/10; (-q(2) + 11)/10];
% end



