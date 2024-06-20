function obstacle = sensing(map, q, r)

inf_obstacle = 100000;

global obstacle 

obstacle = [inf_obstacle; inf_obstacle];

for i = q(1)+r:q(1)-r
    for j = q(2)-r:q(2)+r
    %Euclidean Distance of every cell from the robot position
        %distance=sqrt((j-robo_position(2)).^2+(i-robo_position(1)).^2);
        %Condition simulating the sight of the robot,aka lidar's range
        if (map(i,j) == 0)
        %Save each obstacle seen by the sensor
            obstacle = [obstacle,[i;j]];
        end
    end
end

end



