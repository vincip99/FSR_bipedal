function [path, varargout] = Motion_Planner(q_start, q_goal, map, type)
    % MOTION_PLANNER Plans a path from start to goal position using the specified method.
    %
    % Inputs:
    %   q_start - Starting configuration (e.g., [x, y, theta])
    %   q_goal - Goal configuration (e.g., [x, y, theta])
    %   map - Structure containing the map grid and configuration space grid
    %   type - Integer indicating the planning algorithm to use
    %          1: Artificial Potential Fields (APF)
    %          2: Rapidly-exploring Random Trees (RRT)
    %          3: Numerical Navigation Function (NNF)
    %
    % Outputs:
    %   path - Planned path from start to goal
    %   varargout - Optional additional outputs depending on the chosen algorithm

% Plot the binary map with inverted colors
figure;
clf;
imshow(map.grid, 'InitialMagnification', 'fit');
colormap(flipud(gray)); % Invert the grayscale colormap
title('Binary Map');
xlabel('X-axis', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y-axis', 'FontSize', 12, 'FontWeight', 'bold');
hold on;
axis equal;
set(gca, 'YDir', 'reverse'); % Reverse Y-axis for proper grid display

% Mark the start and goal points
q_start_grid = grid_mapping(q_start(1:2), map);
q_goal_grid = grid_mapping(q_goal(1:2), map);
scatter(q_start_grid(1), q_start_grid(2), 100, 'g', 'filled', 'DisplayName', 'Start');
scatter(q_goal_grid(1), q_goal_grid(2), 100, 'm', 'filled', 'DisplayName', 'Goal');

% Plot the binary map with inverted colors
figure;
clf;
imshow(map.cgrid, 'InitialMagnification', 'fit');
colormap(flipud(gray)); % Invert the grayscale colormap
title('Binary Configuration Space Map');
xlabel('X-axis', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y-axis', 'FontSize', 12, 'FontWeight', 'bold');
hold on;
axis equal;
set(gca, 'YDir', 'reverse'); % Reverse Y-axis for proper grid display

% Mark the start and goal points
q_start_grid = grid_mapping(q_start(1:2), map);
q_goal_grid = grid_mapping(q_goal(1:2), map);
scatter(q_start_grid(1), q_start_grid(2), 100, 'g', 'filled', 'DisplayName', 'Start');
scatter(q_goal_grid(1), q_goal_grid(2), 100, 'm', 'filled', 'DisplayName', 'Goal');

switch type
    case 1
        [path, f, U, q_o, f_x, f_y] = artificialPotential(q_start, q_goal, map, 1);
        varargout = {f, U, q_o, f_x, f_y};
    case 2
        path = rrt(q_start, q_goal, map);
        varargout = {};
    case 3
        [path, v, filledMap] = NNF(q_start_grid, q_goal_grid, map);
        varargout = {v, filledMap};
end

end

function [path, f_t, U_t, q_o, f_x,f_y] = artificialPotential(q_start, q_goal, map, type)

i = 1;
q_next(:, 1) = q_start;
U_t(:, 1) = 0;
f_t(:, 1) = [0; 0];
max_iterations = 200;
q_o = [];

% Iterate until near the goal or max_iteration 
while norm(q_goal - q_next(:, i)) > 0.1 && i < max_iterations
    grid_mapping(q_next(:,i), map)
    q_o = sense(q_next(:,i), map, 10);

    % Grid value of actual q and q_o
    q_o_grid = [];
    for j = 1:size(q_o,2)
        q_o_grid(:,j) = grid_mapping(q_o(:,j), map);
    end
    
    q_grid = grid_mapping(q_next(:,i), map);

    % Plot the detected obstacles
    scatter(q_o_grid(1,:), q_o_grid(2,:), 'y', 'filled');

    % Plot the robot's current position
    scatter(q_grid(1), q_grid(2), 'r', 'filled');

    [q_next(:,i+1), f_t(:,i+1), U_t(i+1)] = artificialPotentialField(q_next(:,i), q_goal, q_o, type);

    q_next_grid = grid_mapping(q_next(:,i+1), map)

        % Check for collision
        collision_detected = false;
        for j = 1:size(q_o_grid, 2)
            if isequal(q_grid, q_o_grid(:, j))
                collision_detected = true;
                break;
            end
        end
        
        if collision_detected
            disp('Collision detected. Stopping the algorithm.');
            return;
        end

    % Plot the robot's path
    plot([q_grid(1) q_next_grid(1)], [q_grid(2) q_next_grid(2)], 'r', 'LineWidth', 1.5, 'DisplayName', 'Path');
    
    % Plot lines from the robot to the detected obstacles
    % plot([q_grid(1) q_o_grid(1, :)], [q_grid(2) q_o_grid(2, :)], 'b');

    pause(0.005)
    i = i + 1;
end

hold off;

path = q_next;

% Define the grid of points
[X, Y] = meshgrid(1:size(map.grid, 2), 1:size(map.grid, 1));

% Initialize arrays for forces and potential
f_x = zeros(size(X));
f_y = zeros(size(Y));
U = zeros(size(X));

q_o_max = sense(q_next(:,i), map, 10000);
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        [~, f, U_val] = artificialPotentialField(grid_inverse_mapping([X(i, j); Y(i, j)], map), q_goal, q_o_max, 1);
        f_x(i, j) = f(1);
        f_y(i, j) = f(2);
        U(i, j) = U_val;
    end
end

f_x = flip(f_x);
f_y = flip(f_y);

magnitude = sqrt(f_x.^2 + f_y.^2);
magnitude = 1;
f_x_norm = f_x ./ magnitude;
f_y_norm = f_y ./ magnitude;

% Plot the vector field using quiver with scaling set to 0 to show true vector lengths
figure;
quiver(X, Y, f_x_norm, f_y_norm, 0);
title('Normalized Vector Field');
xlabel('X');
ylabel('Y');
axis equal; % Make sure the aspect ratio is equal to properly visualize vectors

% Plot the potential field using contour or surf
U = flip(U);
figure;
contour(X, Y, U);
title('Potential Field');
xlabel('X');
ylabel('Y');
colorbar; % Add a colorbar to show potential values

end

function [q_next, f_t, U_t] = artificialPotentialField(q, q_goal, q_o, type)
%ARTIFICIALPOTENTIAL Implements the artificial potential field method for trajectory planning.

    k_a = 10; % attrattive gain
    k_r = 20;    % repulsive gain
    gamma = 2;  % repulsive factor
    Ts = 0.01; % samlpe time

    max_vel_x = 1.25; % m/s
    max_vel_y = 0.5; % m/s
    
    e = q_goal - q;    % error as difference between goal and actual config.
    
    % Attrattive potential
    if norm(e) < 1  % closer to the goal
        U_a = 1/2 * k_a * norm(e)^2; 
        f_a = k_a * e;
    elseif norm(e) >= 1 % bounded force when far from the goal
        U_a = k_a * norm(e);
        f_a = k_a * e/norm(e);
    end

    %   Repulsive potential
    dist = zeros(1,size(q_o,2));
    for i = 1:size(q_o,2)
        dist(i) = norm(q - q_o(:,i));
    end
    [eta, idx] = min(dist); % minimum distance from the obstacles points
    b = q_o(:,idx);

 
    f_r = [0; 0];
    eta_o = 1;
    if eta <= eta_o
        U_r = k_r/gamma * (1/eta - 1/eta_o)^gamma;
        if type == 1
            f_r = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q - b)/eta;
        elseif type == 2
            f_r(1,1) = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(2) - b(2))/eta;
            f_r(2,1) = -k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(1) - b(1))/eta;
        end
    else
        U_r = 0;
        f_r = [0; 0];
    end

    U_t = U_a + U_r;
    f_t = f_a + f_r
    
    % Max velocity saturation if flag is set
    % if flag == 1
        if f_t(1) > max_vel_x
            f_t(1) = max_vel_x;
        elseif f_t(1) < 0
            f_t(1) = 0;
        end
        
        if f_t(2) > max_vel_y
            f_t(2) = max_vel_y;
        elseif f_t(2) < -max_vel_y
            f_t(2) = -max_vel_y;
        end
    % end


    % local minima
    % if U_t > 0 && norm(f_t) < 0.5 
    %     % Perform random motion to escape local minima
    %     q_next = random_motion(q, q_o);
    %     % f_r(1,1) = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(2) - b(2))/eta;
    %     % f_r(2,1) = -k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(1) - b(1))/eta;
    %     % f_t = f_a + f_r;
    %     % q_next = q + Ts * f_t;
    % else
        % Update current position
        q_next = q + Ts * f_t; 
    %end

end

function v = saturation(v, max_vel_x, max_vel_y)

    % Max velocity saturation if flag is set
    if v(1) > max_vel_x
        v(1) = max_vel_x;
    elseif v(1) < 0
        v(1) = 0;
    end
    
    if v(2) > max_vel_y
        v(2) = max_vel_y;
    elseif v(2) < -max_vel_y
        v(2) = -max_vel_y;
    end
end

function q_next = random_motion(q, q_o)
    % Possible movement directions: left, right, up, down, diagonals
    directions = [1, 0; -1, 0; 0, 1; 0, -1; 1, 1; -1, -1; 1, -1; -1, 1];
    
    amplifier=0.03;

    while ~is_free_space(q, q_o)
        % random direction for the movement
        random_dir = directions(randi(size(directions, 1)), :);
        
        % Calculate the next position
        q_next = q +amplifier*random_dir';%trasponse to make it a column vector

    end
    
end

function is_free = is_free_space(position, q_o)
    % Check if the position is free from obstacles
    is_free = true;
    for i = 1:size(q_o, 2)
        if norm(position - q_o(:, i)) == 0
            is_free = false;
            break;
        end
    end
end

function obstacle = sense(q, map, r_grid)

% Sensing distance action
% r = 1;

% Convert from the meter world to the grid world
q_grid = grid_mapping(q, map);
% r_grid = grid_mapping(r);

% r_grid = 5;

% Initializing an obstacle far away for the algorithm
inf_obstacle = 100000;
% obstacle = [inf_obstacle; inf_obstacle];
max_obstacles = 400;
obstacle = inf_obstacle * ones(2, max_obstacles); % Initialize with inf

% Condition simulating the sight of the robot, aka lidar's range
index = 1;
% Checking in a circular manner
for i = q_grid(2)-r_grid:q_grid(2)+r_grid
    for j = q_grid(1)-r_grid:q_grid(1)+r_grid
        % Check if the point is within the circle
        %if (i - q_grid(1))^2 + (j - q_grid(2))^2 <= r_grid^2
            % into the map
            if i > 1 && i < size(map.cgrid, 1) && j > 1 && j < size(map.cgrid, 2)
                if map.cgrid(i, j) ~= 0
                    % Save each obstacle seen by the sensor
                    obstacle(:, index) = grid_inverse_mapping([j; i], map);
                    index = index + 1;
                end
            % boundary of the map as obstacles
            elseif ((i == 1 || i == size(map.cgrid, 1)) && (j >= 1 && j <= size(map.cgrid, 2))) ...
                        || ((j == 1 || j == size(map.cgrid, 2)) && (i >= 1 && i < size(map.cgrid, 1)))
                    obstacle(:, index) = grid_inverse_mapping([j; i], map);
                    index = index + 1;
            end
        %end
    end
end

% Remove the unused columns
obstacle(:,index+1:end) = [];

end

function q_index = grid_mapping(q, map)
%GRID_MAPPING Summary of this function goes here
%   Detailed explanation goes here
q_index = [round((q(1) + 1)/map.step + 1, 0); round((-q(2) + 1)/map.step + 1, 0)];
end

function q_meter = grid_inverse_mapping(q, map)
%GRID_INVERSE_MAPPING Summary of this function goes here
%   Detailed explanation goes here
q_meter = [map.step*(q(1) - 1) - 1; -map.step*(q(2) - 1) + 1];
end

%% RRT
function [tree, v] = rrt(q_start, q_goal, map)
    
q_start = grid_mapping(q_start);
q_goal = grid_mapping(q_goal);
% init the tree list
tree = q_start;
v = [];
Ts = 0.01;
% init the list of point each one of them preceded the tree point
near = q_start;
% number of iterations
k = 150;
% incremental distance
dq = 4;

for i = 1 : k 
    % Generate a random configuration on the map
    qr = randomConfiguration(map);
    
    % Choosing the nearest tree configuration to the random configuration 
    qNear = nearestConfiguration(qr, tree);
    
    % Generate the new point starting from the nearest configuration
    qNew = newConfiguration(qNear, qr, dq);
    
    % Updating the tree
    [tree, goal, near] = updateTree(qNear, qNew, dq, q_goal, tree, map, near);
    v = [v, (tree(end)-tree(end-1))/Ts];

    % Plot a red dot for the coordinates at the end of the queue 
    scatter(tree(1,end), tree(2,end), 30, 'r', 'filled'); 
    
    % Plot lines between dots
    plot([near(1, end), tree(1, end)], [near(2, end), tree(2, end)], 'b', 'LineWidth', 1.5); 

    % Add labels
    xlabel('X');
    ylabel('Y');
    title('RRT Path from Start to End');

    if goal == true
        disp('Goal reached!');
        plotEnviroment(tree, q_start, q_goal, dq); 
        return;
    end
    
    hold on
end

if goal == false
    disp('Goal not reached, try to increase the max iteration number');
end

plotEnviroment(tree, q_start, q_goal, dq);

end

% Function that generates a random point in the map space
function qr = randomConfiguration(map)
    % Picking 2 random coordinates between 1 to row, 1 to column
    qr = [randi(size(map, 2)); randi(size(map, 1))];
end

% Function that pick the nearest point to the random generated point,
% choosing from the tree elements
function [qNear, normVector] = nearestConfiguration(qr, G)
    % creating the norm vector between each G element and qr
    normVector = zeros(size(G(1,:)));
    for i = 1 : size(G,2)
        normVector(i) = norm(G(:,i) - qr);
    end
    % finding the minimum norm
    [~, minIndex] = min(normVector);
    % the closest point is the one with the same index of the minimum norm
    % in the norm vector
    qNear = G(:, minIndex);
end

% Function that generates the new point at distance dq from the nearest
% point in the tree
function [qNew, beta] = newConfiguration(qNear, qr, dq)
    %   qr
    %    |\      tan(beta) = b/c
    %  b | \ a    cos(beta) = c/a 
    %    |  \        sin(beta) = b/a
    %    |___\beta  
    %       c   qNear
    % Computing the angle between qr and qNear
    beta = atan2(qr(2) - qNear(2), qr(1) - qNear(1));
    % Computing the new point coordinates
    qNew = round(qNear + dq * [cos(beta); sin(beta)]);
end

% Function that update the tree with the new point
function [tree, goal, near] = updateTree(qNear, qNew, dq, qf, tree, map, near)
   
    % Initialize goal flag
    goal = false;
    % Checking if the point is inside the map boundaries and not colliding with obstacles
    if qNew(1) >= 1 && qNew(1) <= size(map, 2) && ...
       qNew(2) >= 1 && qNew(2) <= size(map, 1) && ...
       map(qNew(2), qNew(1)) == 0 && ...
       collisionDetector(map, qNear, qNew)
       % Add the new point to the tree
       tree = [tree, qNew];
       % Store the nearest point to the actual tree point 
       near = [near, qNear];
       % Check if the new point is close to the goal
       if norm(qNew - qf) < dq
           goal = true;
           return;
       end
    end
end

% Function that detect if there arey collision in the rectangular area
% between the previous and actual point
function collisionFree = collisionDetector(map, qNear, qNew)
    % Checking if all the blocks inside the rectangular area between qNear
    % and qNew are 1, so without obstacles
    collisionFree = all(map(min(qNew(2), qNear(2)):max(qNew(2), qNear(2)),...
        min(qNew(1), qNear(1)):max(qNew(1), qNear(1))) == 0, 'all');

end

% Plot function
function plotEnviroment(tree, qi, qf, dq)
        % Plot starting and final points with the goal circumference
        rectangle('Position', [qf(1) - dq, qf(2) - dq, 2*dq, 2*dq], ...
                  'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 1, ...
                  'LineStyle', '--');  % shaping the rectangular as a circle
        scatter(qi(1), qi(2),'g','filled');
        scatter(qf(1), qf(2),'y','filled');
        scatter(tree(1,end), tree(2,end), 'b','filled') % final tree point
        % Plot the line distance between the last tree point and the goal
        plot([tree(1,end) qf(1)], [tree(2,end) qf(2)], 'b', 'LineWidth', 1.5, ...
            'LineStyle', '--');
end


%% Numerical navigation function
function [path, v, filledMap] = NNF(qs, qg, map)

% generate the modified map
map.cgrid(map.cgrid ~= 0) = -1;
Ts = 0.01;

% Flooding algorithm to generate the filled map
filledMap = breadth4explorer(map.cgrid, qg);

% Display the filled map
figure;
imshow(filledMap, [], 'InitialMagnification', 'fit');
colormap([0 0 0; jet(256)]); % Set colormap with black for the first color
caxis([-1, max(filledMap(:))]); % Ensure the range includes -1 for black
colorbar;
title('Binary Map with Path');
hold on;

max_vel_x = 1.25; % m/s
max_vel_y = 1; % m/s

% Path finding and cost 
path = depthFirstHeuristic(filledMap, qs, qg);
v = zeros(4,size(path,2));
for i = 1:size(path,2)-1
    v(1:2,i) = grid_inverse_mapping(path(:,i+1),map)/Ts-grid_inverse_mapping(path(:,i),map)/Ts;
    v(1:2,i) = saturation(v(1:2,i), max_vel_x, max_vel_y);
    if v(2,i) ~= 0
        Ts = 0.3;
    else
        Ts = 0.1;
    end
        % Orientation of the force
    v(3,i) = atan2(v(2,i),v(1,i));
    v(4,i+1) = v(4,i) + Ts;
end
[hop, cost] = hopCounter(filledMap, path);

% Plot the start and goal positions
plot(qs(1), qs(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start'); % Start point
plot(qg(1), qg(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal'); % Goal point

% Plot the path
pathX = path(1,:);
pathY = path(2,:);
plot(pathX, pathY, 'b-', 'LineWidth', 2, 'DisplayName', 'Path'); % Path

% Plot the visited points
scatter(path(1,:), path(2,:), 'm', 'filled', 'DisplayName', 'Visited Points');

% Add legend
legend;

% Set axis properties
axis on;
grid on;
xlabel('X-axis', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y-axis', 'FontSize', 12, 'FontWeight', 'bold');
title('Pathfinding from Start to Goal');

hold off;

end

% floading algorithm with 4 adjacent cells
function [map, path] = breadth4explorer(emptyMap, qg)
    % Init the queue and the visited matrix
    x = qg(1); 
    y = qg(2);
    queue = [x; y];
    visited = zeros(size(emptyMap));
    visited(y, x) = 1;
    
    % Iteration until the queue has no more unexplored points
    while ~isempty(queue)
        % Queue pop
        x = queue(1, 1);
        y = queue(2, 1);
        queue(:, 1) = [];
        
        % Searching the four adjacent cells
        list = fourNeighbors(emptyMap, x, y);
        
        % For every cell in the list, modify the respective value in the map
        for i = 1:size(list, 2)
            nx = list(1, i);
            ny = list(2, i);
            % If the cell was not yet visited
            if visited(ny, nx) == 0
                % Flooding the 4 adjacent cells
                emptyMap(ny, nx) = emptyMap(y, x) + 1;
                visited(ny, nx) = 1; % Marking the cell as visited
                queue = [queue, [nx; ny]]; % Update the queue
            end
        end
    end
    
    map = emptyMap;
    path = find(visited); % This will return linear indices of visited cells
end

function list = fourNeighbors(map, x, y)

    list = [];
    % position of south cell
    if y + 1 <= size(map,1) && map(y + 1, x) == 0
        list = [list, [x; y + 1]];
    end
    % position of north cell
    if y - 1 >= 1 && map(y - 1, x) == 0
        list = [list, [x; y - 1]];
    end
    % position of east cell
    if x + 1 <= size(map,2) && map(y, x + 1) == 0
        list = [list, [x + 1; y]];
    end
    % position of west cell
    if x - 1 >= 1 && map(y, x - 1) == 0
        list = [list, [x - 1; y]];
    end

end

% path finding algorithm with heuristic stack sort
function [path, stack, heuristic] = depthFirstHeuristic(map, qs, qg)
    % Init the stack and the visited vector
    x = qs(1);
    y = qs(2);
    stack = [x; y];
    visited = [];
    
    %until we reach the goal
    while (x ~= qg(1) || y ~= qg(2)) && ~isempty(stack)
        % pop
        x = stack(1,end);
        y = stack(2,end);
        stack(:,end) = [];
        % marking the point as visited
        visited = [visited, [x; y]];
        % searching the 8 adjacent cells with min value
        [minValue, list] = minNeighbors(map, x, y);
        heuristic = heuristicSorting(map, list);
        % for each valid cell update the stack
        for i = 1 : size(list,2)
            if (~ismember(visited(1,:),minValue(1))) | (~ismember(visited(2,:),minValue(2)))
               stack = [stack, minValue];
            end
        end
    end
    path = visited;
end

% Heuristic stack sort
function heuristic = heuristicSorting(map, list)
    valueList = zeros(1, size(list, 2));
    for i = 1:size(list, 2)
        valueList(i) = map(list(2, i), list(1, i));
    end
    [~, index] = sort(valueList);
    heuristic = list(:, index);
end

% Adjacent cell list function
function [minValue, list] = minNeighbors(map, x, y)
    list = [];
    value = [];
    
    % Adding neighbors row and column value only if they are not wall or out of the map
    if y + 1 <= size(map, 1) && map(y + 1, x) ~= -1
        list = [list, [x; y + 1]];
        value = [value, map(y + 1, x)];
    end
    if y - 1 >= 1 && map(y - 1, x) ~= -1
        list = [list, [x; y - 1]];
        value = [value, map(y - 1, x)];
    end
    if x + 1 <= size(map, 2) && map(y, x + 1) ~= -1
        list = [list, [x + 1; y]];
        value = [value, map(y, x + 1)];
    end
    if x - 1 >= 1 && map(y, x - 1) ~= -1
        list = [list, [x - 1; y]];
        value = [value, map(y, x - 1)];
    end
    if y - 1 >= 1 && x - 1 >= 1 && map(y - 1, x - 1) ~= -1
        list = [list, [x - 1; y - 1]];
        value = [value, map(y - 1, x - 1)];
    end
    if y - 1 >= 1 && x + 1 <= size(map, 2) && map(y - 1, x + 1) ~= -1
        list = [list, [x + 1; y - 1]];
        value = [value, map(y - 1, x + 1)];
    end
    if y + 1 <= size(map, 1) && x - 1 >= 1 && map(y + 1, x - 1) ~= -1
        list = [list, [x - 1; y + 1]];
        value = [value, map(y + 1, x - 1)];
    end
    if y + 1 <= size(map, 1) && x + 1 <= size(map, 2) && map(y + 1, x + 1) ~= -1
        list = [list, [x + 1; y + 1]];
        value = [value, map(y + 1, x + 1)];
    end
    
    % Pick the min value 
    [~, index] = min(value);
    minValue = list(:, index);
end

function [hop, cost] = hopCounter(map, path)
    % Counting the numer of cell traveled (like the number of router!)
    hop = size(path,2);
    
    % Total cost of the path
    cost = 0;
    for i = 1 : size(path,2)
        cost = cost + map(path(2,i), path(1,i));
    end
end

