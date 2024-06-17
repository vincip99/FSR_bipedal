clear; close all; clc

q = [0; 0];
q_goal = [2; 2];
q_o = 1*[1, 1.1; 1, 1.2; 1, 1.3; 2.5, 2.5; 2.5, 2.6; 1.5, 1; 1.6,1]';
Ts = 0.01;
k_a = 10; k_r = 10;

q_next = q;
figure
plot(q_o(1,:), q_o(2,:));
hold on
for i = 1:100
    q_next = [q_next, artificialPotential(q_next(:,i), q_goal, q_o, Ts, k_a, k_r)];
    [tilde, f, U] = artificialPotential(q_next(:,i), q_goal, q_o, Ts, k_a, k_r)
    scatter(q_next(1,i),q_next(2,i))
    hold on
    pause(0.1)
end
