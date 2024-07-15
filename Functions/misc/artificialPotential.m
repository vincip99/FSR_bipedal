function out = artificialPotential(u)
%ARTIFICIALPOTENTIAL Implements the artificial potential field method for trajectory planning.

    k_a = 1.5; % attrattive gain
    k_r = 400;    % repulsive gain
    gamma = 2;  % repulsive factor
    Ts = 0.01; % samlpe time

    max_vel_x = 1; % m/s
    max_vel_y = 1; % m/s
    min_vel = 0.2; % m/s
    max_yaw = pi/4; 

    % Input Variables
    q = reshape(u(1:2), 2, 1);
    q_goal = reshape(u(3:4), 2, 1);
    
    q_o = reshape(u(5:end), 2, []); % reshape to ensure it's a 2xN matrix
    
    e = q_goal - q;    % error as difference between goal and actual config.
    
    %   Attrattive potential
    if norm(e) < 1  % closer to the goal
        U_a = 1/2 * k_a * norm(e)^2; 
        f_a = k_a * e;
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

    % Condition to select points where both coordinates are less than 100
    condition = (q_o(1, :) < 100) & (q_o(2, :) < 100);

    % Select only the points satisfying the condition
    q_obs = q_o(:, condition);

    % f_r = [0; 0];
    % U_r = 0;
    % for i = 1:size(q_obs,2)
    %     eta = norm(q - q_obs(:,i));
    %     eta_o = 0.7;
    %     if eta <= eta_o
    %         U_r = U_r + k_r/gamma * (1/eta - 1/eta_o)^gamma;
    %         f_r = f_r + k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q - q_obs(:,i))/eta;
    %         f_r(1,1) = f_r(1,1) + k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(2) - q_obs(2,i))/eta;
    %         f_r(2,1) = f_r(2,1) -k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(1) - q_obs(1,i))/eta;
    %         Ts = Ts/10; % Reducin integration time if near the obstacle
    %     elseif eta > eta_o 
    %         U_r = U_r + 0;
    %         f_r = f_r + [0; 0];
    %     end
    % end
    
 
    yaw = 0;
    %   Repulsive potential
    eta_o = 1.6;
    if eta <= eta_o
        U_r = k_r/gamma * (1/eta - 1/eta_o)^gamma;
        % f_r = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q - b)/eta;
        f_r(1,1) = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(2) - b(2))/eta;
        f_r(2,1) = -k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q(1) - b(1))/eta;
    elseif eta > eta_o 
        U_r = 0;
        f_r = [0; 0];
    end

    U_t = U_a + U_r;
    f_t = f_a + f_r;
    
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

    % Orientation of the force
    % yaw = atan2(f_t(2),f_t(1));

    if yaw > max_yaw
        yaw = max_yaw;
    elseif yaw < -max_yaw
        yaw = -max_yaw;
    end

    out(1:2) = f_t';
    out(3) = yaw; % yaw;

end


