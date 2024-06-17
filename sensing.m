function obstacle=sensing(map,robo_position,r_sphere)

obstacle=[];

for i=(robo_position(1)+r_sphere):(robo_position(1)-r_sphere)
    for j=(robo_position(2)-r_sphere):(robo_position(2)+r_sphere)
%Euclidean Distance of every cell from the robot position
        %distance=sqrt((j-robo_position(2)).^2+(i-robo_position(1)).^2);
%Condition simulating the sight of the robot,aka lidar's range
        if (map(i,j)~=0)
%Save each obstacle seen by the sensor
            obstacle=[obstacle;[i,j]];
        end
    end
end

end

