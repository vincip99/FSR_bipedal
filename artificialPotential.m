function [q_next, f_t, U_t] = artificialPotential(q, q_goal, q_o, Ts, k_a, k_r)
%ARTIFICIALPOTENTIAL Implements the artificial potential field method for trajectory planning.

    gamma = 2;  % repulsive factor
    
    e = q_goal - q;    % error as difference between goal and actual config.
    
    %   Attrattive potential
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

 
    eta_o = 2;
    if eta <= eta_o
        U_r = k_r/gamma * (1/eta - 1/eta_o)^gamma;
        f_r = k_r/eta^2 * (1/eta - 1/eta_o)^(gamma-1)*(q - b)/eta;
    elseif eta > eta_o 
        U_r = 0;
        f_r = 0;
    end

    U_t = U_a + U_r;
    f_t = f_a + f_r;

    % local minima
    % k_v = k_r/2;
    % 
    % if eta <= eta_o
    %     U_vir = k_v/(q - b);
    %     f_vir = k_v * (q - b)/eta;
    % elseif eta > eta_o
    %     U_vir = 0;
    %     f_vir = 0;
    % end
    % 
    % if f_t == 0 & U_t > 0
    %     U_t = U_a + U_r + U_vir;
    %     f_t = f_a + f_r - f_vir;
    % else
    %     %   Total contribution
    %     U_t = U_a + U_r;
    %     f_t = f_a + f_r;
    % end

    
    %   updating current position
    q_next = q + Ts * f_t;

end



