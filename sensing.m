function obstacle = sensing(u)

% Sensing distance action
r = 1;

q = reshape(u(1:2), 2, 1);
map = reshape(u(3:end), 21, 61);

% Convert from the meter world to the grid world
q_grid = grid_mapping(q);
r_grid = grid_mapping(r);

% Initializing an obstacle far away for the algorithm
inf_obstacle = 100000;
% obstacle = [inf_obstacle; inf_obstacle];
max_obstacles = 400;
obstacle = inf_obstacle * ones(2, max_obstacles); % Initialize with inf

% Condition simulating the sight of the robot,aka lidar's range
index = 1;
for i = q_grid(1)-r_grid:q_grid(1)+r_grid
    for j = q_grid(2)-r_grid:q_grid(2)+r_grid
        % into the map
        if i > 1 && i < size(map,1) && j > 1 && j < size(map,2)
            if map(i,j) == 1
                %Save each obstacle seen by the sensor
                obstacle(:,index) = grid_inverse_mapping([j;i]);
                index = index + 1;
            end
        % boundary of the map as obstacles
        elseif i == 1 || i == size(map,1) || j == 1 || j == size(map,2)
            obstacle(:,index) = grid_inverse_mapping([j;i]);
            index = index + 1;
        end
    end
end

obstacle = reshape(obstacle, 1, 2*max_obstacles);

end

function q_index = grid_mapping(q)
%GRID_MAPPING Summary of this function goes here
%   Detailed explanation goes here
q_index = (10*round(q,1)+11);
end

function q_meter = grid_inverse_mapping(q)
%GRID_INVERSE_MAPPING Summary of this function goes here
%   Detailed explanation goes here
q_meter = (q - 11)/10;
end



