function out = artificialPotential(u)
%ARTIFICIALPOTENTIAL Implements the artificial potential field method for trajectory planning.

    k_a = 2.25; % attrattive gain
    k_r = 40;    % repulsive gain
    gamma = 2;  % repulsive factor
    Ts = 0.01; % samlpe time

    max_vel_x = 1.25; % m/s
    max_vel_y = 1; % m/s
    min_vel = 0.5; % m/s

    % Input Variables
    q = reshape(u(1:2), 2, 1);
    q_goal = reshape(u(3:4), 2, 1);
    q_o = reshape(u(5:end), 2, []); % reshape to ensure it's a 2xN matrix
    
    e = q_goal - q;    % error as difference between goal and actual config.
    
    %   Attrattive potential
    if norm(e) < 1  % closer to the goal
        U_a = 1/2 * k_a * norm(e)^2; 
        f_a = k_a * e;
        Ts = Ts/10; % Reducin integration time if near the goal
    elseif norm(e) >= 1 % bounded force when far from the goal
        U_a = k_a * norm(e);
        f_a = k_a * e/norm(e);
    end

    %   Closest obstacles
    dist = zeros(1,size(q_o,2));
    for i = 1:size(q_o,2)
        dist(i) = norm(q - q_o(:,i));
    end
    [eta, idx] = min(dist); % minimum distance from the obstacles points
    b = q_o(:,idx);
 
    %   Repulsive potential
    eta_o = 0.8;
    if eta <= eta_o
        U_r = k_r/gamma * (1/eta - 1/eta_o)^gamma;
        %   f_r = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q - b)/eta;
        f_r(1,1) = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(2) - b(2))/eta;
        f_r(2,1) = -k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(1) - b(1))/eta;
        Ts = Ts/10; % Reducin integration time if near the obstacle
    elseif eta > eta_o 
        U_r = 0;
        f_r = [0; 0];
    end

    U_t = U_a + U_r;
    f_t = f_a + f_r;

    % Orientation of the force
    yaw = atan2(f_t(2),f_t(1));
    
    % max velocity saturation
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

    if f_t(1) == 0 && f_t(2) ~= 0
        f_t(1) = min_vel;
    end

    % local minima
    if U_t > 0 && norm(f_t) == 0 
        % Perform random motion to escape local minima
        q_next = random_motion(q, q_o, 10);
    else
        % Update current position
        q_next = q + Ts * f_t;
    end

    out(1:2) = f_t';
    out(3) = 0; %yaw;

end

function q_next = random_motion(q, q_o)
    % Possible movement directions: left, right, up, down, diagonals
    directions = [1, 0; -1, 0; 0, 1; 0, -1; 1, 1; -1, -1; 1, -1; -1, 1];
    
    amplifier=3;

    while ~is_free_space(q_next, q_o)
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


