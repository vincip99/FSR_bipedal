clear; close all; clc

%% Defining map parameters
res = 1; % cells per meter
row = 30; % map width
col = 30; % map length
height = 10; % map heigth

step = 1;

map3D = occupancyMap3D(res);


pose = [0 0 0 1 0 0 0]; % initial pose [x y z quaternion]

Ground = groundGenerator(row, col);

pattern = randomHome(row, col, height);
pattern = vertcat(pattern, buildingsPattern(30, row, col, height));

figure
surf(pattern)

occval = 1;


setOccupancy(map3D,Ground,1);
updateOccupancy(map3D,pattern,1);
maxRange = 5;

%insertPointCloud(map3D,pose,ground,maxRange)
figure
show(map3D)

function Ground = groundGenerator(row, col)
    % Generate the flor 
    [X, Y, Z] = meshgrid(0:row, 0:col, 0); % row by column plane
    Ground = [X(:) Y(:) Z(:)]; % Concatenate and reshape the matrices into a single array 

end

function Building = buildingGenerator(x, y, length, width, height)
    if length >= 0 && width >= 0
        [X, Y, Z] = meshgrid(x:x+length, y:y+width, 0:height);
    elseif length < 0 && width >= 0
        [X, Y, Z] = meshgrid(x+length:x, y:y+width, 0:height);
    elseif length >= 0 && width < 0
        [X, Y, Z] = meshgrid(x:x+length, y+width:y, 0:height);
    elseif length < 0 && width < 0 
        [X, Y, Z] = meshgrid(x+length:x, y+width:y, 0:height);
    end
     
     Building = [X(:) Y(:) Z(:)];

end

function Plane = planeGenerator(x, y, z, length, width)

     [X, Y, Z] = meshgrid(x:x+length, y:y+width, z);
     Plane = [X(:) Y(:) Z(:)];

end

function [Wall, xfin, yfin] = wallGenerator(x, y, length, height, orient)
    
    if orient == 0
        Wall = buildingGenerator(x, y, length, 0, height);
        xfin = x + length;
        yfin = y;
    elseif orient == 1 
        Wall = buildingGenerator(x, y, 0, length, height);
        xfin = x;
        yfin = y + length;
    end
    
end

function pattern = buildingsPattern(numberBuildings, width, depth, height)
    pattern = [];  % Initialize an empty array to store building patterns

    for i = 1:numberBuildings
        x = randi(depth); % Random x-coordinate within the depth
        y = randi(width); % Random y-coordinate within the width
        zHeight = randi(height); % Random building height within the max height

        % Generate the building and concatenate to pattern1
        pattern = vertcat(pattern, buildingGenerator(x, y, 1, 1, zHeight));
    end
end

function pattern = wallPattern(number, row, col, height)
    pattern = [];  % Initialize an empty array to store wall patterns
    
    x = 0; y = 0;
    for i = 1:number
        length = randi([min(row,col)/2, max(row,col)]);
        tickness = randi([0, 1]);
        % Generate the building and concatenate to pattern1
        [Wall, xfin, yfin] = wallGenerator(x, y, length, tickness, height, randi(5)-1);
        pattern = vertcat(pattern, Wall);
        x = xfin;
        y = yfin;
    end

end

function pattern = randomHome(row, col, height)
    
    x = 0; y = 0;
    [pattern, xfin, yfin] = wallGenerator(x, y, row, height, 0);
    [Wall, xfin, yfin] = wallGenerator(xfin, yfin, col, height, 1);
    pattern = vertcat(pattern, Wall);
    [Wall, xfin, yfin] = wallGenerator(xfin, yfin, -row, height, 0);
    pattern = vertcat(pattern, Wall);
    pattern = vertcat(pattern, wallGenerator(xfin, yfin, -col, height, 1));

end





