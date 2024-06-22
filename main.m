clear; close all; clc

%% map 
q_start = [8; 8];
q_goal = [20; 20];

obstacles = 4;

Ts = 0.01;
k_a = 10; k_r = 10;

q_next(:,1) = q_start;

map2D = map(40, 40, 10);

    % Plot the image
    figure
    imshow(map2D,'InitialMagnification','fit');
    hold on
    scatter(q_start(1), q_start(2),'g','filled');
    scatter(q_goal(1), q_goal(2),'b','filled');
    hold on

    i = 1;
while norm(q_goal - q_next(:,i)) > 1
    q_o = sensing(map2D,ceil(q_next(:,i)),2);
    
    [q_next(:,i+1), f(:,i), U(:,i)] = artificialPotential(q_next(:,i), q_goal, q_o, Ts, k_a, k_r);
    scatter(q_next(1,i),q_next(2,i), 'r', 'filled')
    hold on
    pause(0.05)
    i = i + 1;
end



% [X,Y] = meshgrid(q_next(1,2:end), q_next(2,2:end));
% quiver(q_next(1,2:end), q_next(2,2:end),f(1,:),f(2,:))

function showMap(image_map, q_start, q_goal)

    % Plot the image
    figure
    imshow(image_map,'InitialMagnification','fit');
    hold on
    scatter(q_start(1), q_start(2),'g','filled');
    scatter(q_goal(1), q_goal(2),'b','filled');
    hold on

end