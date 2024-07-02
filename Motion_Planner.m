function [path, f_t, U_t,X,Y,f_x,f_y] = Motion_Planner(q_start, q_goal, map)
%MOTION_PLANNER Summary of this function goes here
%   Detailed explanation goes here
[path, f_t, U_t, X,Y,f_x,f_y] = artificialPotential(q_start, q_goal, map);
end

function [path, f_t, U_t, X,Y,f_x,f_y] = artificialPotential(q_start, q_goal, map)

i = 1;
q_next(:,1) = q_start;
U_t(:,1) = [0; 0];
f_t(:,1) = [0; 0];
% Iterate until near the goal or max_iteration 
while norm(q_goal - q_next(:,i)) > 0.1 && i < 350
    grid_mapping(q_next(:,i))
    q_o = sense(q_next(:,i),map);

    [q_next(:,i+1), f_t(:,i+1), U_t(i+1)] = artificialPotentialField(q_next(:,i), q_goal, q_o, 1);

    scatter(grid_mapping(q_o(1,:)),grid_mapping(q_o(2,:)), 'y', 'filled')
    scatter(grid_mapping(q_next(1,i)),grid_mapping(q_next(2,i)), 'r', 'filled')
    plot([q_next(1,i) q_next(1,i+1)], [q_next(2,i) q_next(2,i+1)], 'b', 'LineWidth', 1.5, ...
            'LineStyle', '--');

    hold on
    pause(0.05)
    i = i + 1;
end

path = q_next;

% Define the grid of points
[X, Y] = meshgrid(1:size(map, 2), 1:size(map, 1));

% Initialize arrays for forces and potential
f_x = zeros(size(X));
f_y = zeros(size(Y));
U = zeros(size(X));


for i = 1:size(X,1)
    for j = 1:size(X,2)
        [~, f, U] = artificialPotentialField(grid_inverse_mapping([X(i,j); Y(i,j)]), q_goal, q_o, 0);
        f_x(i, j) = f(1);
        f_y(i, j) = f(2);
        U(i, j) = U;

    end
end

end

function [q_next, f_t, U_t] = artificialPotentialField(q, q_goal, q_o, flag)
%ARTIFICIALPOTENTIAL Implements the artificial potential field method for trajectory planning.

    k_a = 10; % attrattive gain
    k_r = 10;    % repulsive gain
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

 
    eta_o = 4;
    if eta <= eta_o
        U_r = k_r/gamma * (1/eta - 1/eta_o)^gamma;
        %f_r = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q - b)/eta;
        f_r(1,1) = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(2) - b(2))/eta;
        f_r(2,1) = -k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(1) - b(1))/eta;
    elseif eta > eta_o 
        U_r = 0;
        f_r = [0; 0];
    end

    U_t = U_a + U_r;
    f_t = f_a + f_r;
    
    % Max velocity saturation if flag is set
    if flag == 1
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
    end


    % local minima
    % if U_t > 0 && norm(f_t) == 0 
    %     % Perform random motion to escape local minima
    %     q_next = random_motion(q, q_o);
    % else
        % Update current position
        q_next = q + Ts * f_t;
    % end

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

function obstacle = sense(q, map)

% Sensing distance action
r = 1;

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


function tree = rrt(q_i, q_goal, dq, map)
    
% init the tree list
tree = q_i;
% init the list of point each one of them preceded the tree point
near = q_i;

for i = 1 : k 
    % Generate a random configuration on the map
    qr = randomConfiguration(map);
    
    % Choosing the nearest tree configuration to the random configuration 
    qNear = nearestConfiguration(qr, tree);
    
    % Generate the new point starting from the nearest configuration
    qNew = newConfiguration(qNear, qr, dq);
    
    % Updating the tree
    [tree, goal, near] = updateTree(qNear, qNew, dq, q_goal, tree, map, near);

    % Plot a red dot for the coordinates at the end of the queue 
    scatter(tree(end,1), tree(end,2), 30, 'r', 'filled'); 
    
    % Plot lines between dots
    plot([near(end,1), tree(end,1)], [near(end,2), tree(end,2)], 'b', 'LineWidth', 1.5); 

    % Add labels
    xlabel('X');
    ylabel('Y');
    title('RRT Path from Start to End');

    if goal == true
        disp('Goal reached!');
        plotEnviroment(tree, qi, q_goal, dq); 
        return;
    end
    
    hold on
end

end

% Function that generates a random point in the map space
function qr = randomConfiguration(map)
    % Picking 2 random coordinates between 1 to row, 1 to column
    qr = [randi(size(map, 2)), randi(size(map, 1))];
end

% Function that pick the nearest point to the random generatet point,
% choosing from the tree elements
function [qNear, normVector] = nearestConfiguration(qr, G)
    % creating the norm vector between each G element and qr
    normVector = zeros(size(G(:,1)));
    for i = 1 : size(G(:,1))
        normVector(i) = norm(G(i,:) - qr);
    end
    % finding the minimum norm
    [~, minIndex] = min(normVector);
    % the closest point is the one with the same index of the minimum norm
    % in the norm vector
    qNear = G(minIndex, :);
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
    qNew = round(qNear + dq * [cos(beta), sin(beta)]);
end

% Function that update the tree with the new point
function [tree, goal, near] = updateTree(qNear, qNew, dq, qf, tree, map, near)
   
    % Initialize goal flag
    goal = false;
    % Checking if the point is inside the map boundaries and not colliding with obstacles
    if qNew(1) >= 1 && qNew(1) <= size(map, 2) && ...
       qNew(2) >= 1 && qNew(2) <= size(map, 1) && ...
       map(qNew(2), qNew(1)) == 1 && ...
       collisionDetector(map, qNear, qNew)
       % Add the new point to the tree
       tree = [tree; qNew];
       % Store the nearest point to the actual tree point 
       near = [near; qNear];
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
        min(qNew(1), qNear(1)):max(qNew(1), qNear(1))) == 1, 'all');

end

% Plot function
function plotEnviroment(tree, qi, qf, dq)
        % Plot starting and final points with the goal circumference
        rectangle('Position', [qf(1) - dq, qf(2) - dq, 2*dq, 2*dq], ...
                  'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 1, ...
                  'LineStyle', '--');  % shaping the rectangular as a circle
        scatter(qi(1), qi(2),'g','filled');
        scatter(qf(1), qf(2),'y','filled');
        scatter(tree(end,1), tree(end,2), 'b','filled') % final tree point
        % Plot the line distance between the last tree point and the goal
        plot([tree(end,1) qf(1)], [tree(end,2) qf(2)], 'b', 'LineWidth', 1.5, ...
            'LineStyle', '--');
end
