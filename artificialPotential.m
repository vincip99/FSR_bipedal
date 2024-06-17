function [q_next, f_t, U_t] = artificialPotential(q, q_goal, q_o, Ts, k_a, k_r)
%ARTIFICIALPOTENTIAL Summary of this function goes here

e = q_goal - q;    % error as difference between goal and actual config.
gamma = 2;  % repulsive factor

%   Attrattive potential
if norm(e) < 1
    U_a = 1/2 * k_a * norm(e)^2; 
    f_a = k_a * e;
elseif norm(e) >= 1
    U_a = k_a * norm(e);
    f_a = k_a * e/norm(e);
end

%   Repulsive potential
dist = [];
for i = 1:length(q_o)
    dist = [dist, norm(q - q_o)];
end
[eta, idx] = min(dist);
b = q_o(:,idx);


eta_o = 4;
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
if Ut > 0 && f_t == 0
    q_next = q + Ts * rand(1,3)*f_t
end

q_next = q + Ts * f_t;

end

