function map2D = map(row, col, obstacles)
%MAP Creates a 2D binary map with random obstacles
%   map2D = map(row, col, obstacles) creates a 2D binary map with
%   specified number of rows and columns and places the given number
%   of obstacles at random positions.

% Create an empty map 
map2D = zeros(row, col);

% mark the boundaries of the map as walls
map2D(2:row-1,2:col-1) = 1;

% Generate random obstacle positions
x_o = randi(row, obstacles, 1);
y_o = randi(col, obstacles, 1);

% generate random length and width of the obstacle
length = randi(row/10, obstacles, 1);
width = randi(col/10, obstacles, 1);

% Mark obstacle positions in the map
for i = 1:obstacles
    % starting from q_o generate a more realistic obstacle
    for j = x_o(i):x_o(i)+length
        for k = y_o(i):y_o(i)+width
            % if the x-y position is in the bound of the map
            if(k < col && j < row)
                map2D(j,k) = 0;
            end            
        end
    end
end

end







