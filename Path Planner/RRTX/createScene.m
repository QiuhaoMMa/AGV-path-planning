function environment=createScene(scene_id)
obstacles_vertices=generatingObstacles(scene_id);

% Create Obstacle objects for each circle
obstacles = Obstacle.empty;
for i = 1:numel(obstacles_vertices)
    obstacles(i) = Obstacle('polygon', 'vertices', obstacles_vertices{i});
end

% Environment settings
boundary=[1 29;1 29];
start = [2,2]; % Start point
goal = [26, 26]; % Goal point
environment = Environment(start,goal,boundary,'obstacles',obstacles,'resolution',2);
end

function obstacles_vertices = generatingObstacles(scene_id)
    obstacles_vertices = [];
    
    % Predefine number of points for circular shapes
    numPoints = 50;
    switch scene_id
        case 1
            % Rectangular obstacles_vertices
            obstacles_vertices{1} = [5, 8, 10, 7; 7, 4, 7, 10];
            obstacles_vertices{2} = [5, 10, 11, 11; 22, 14, 25, 25];
            obstacles_vertices{3} = [13, 17, 13, 16, 22, 16; 26, 22, 18, 15, 22, 29];
            obstacles_vertices{4} = obstacles_vertices{3};  % Duplicate block 3
            
            % Circular block
            obstacles_vertices{5} = generateCircles(22, 11, 5, numPoints);

        case 2
            % Various rectangular obstacles_vertices
            obstacles_vertices{1} = [5 10 10 5; 4 4 9 9];
            obstacles_vertices{2} = [14 16 16 16 5 5 14; 4 4 6 14 14 12 12];
            obstacles_vertices{3} = [19 25 25 19; 4 4 10 10];
            obstacles_vertices{4} = [23 27 19 19; 12 19 19 19];
            obstacles_vertices{5} = [5 16 16 7 7 5; 18 18 21 21 26 26];
            obstacles_vertices{6} = [13 16 16 13; 23 23 26 26];

        case 3
            obstacles_vertices{1} = [7 11 13 9 5; 7 7 12 15 12];
            obstacles_vertices{2} = [21 25 22 11 3 17; 4 14 23 26 20 17];

        case 4
            % Various rectangular and more complex shapes
            obstacles_vertices{1} = [8 9 9 8; 1 1 5 5];
            obstacles_vertices{2} = [13 19 19 22 22 19 19 18 18 13; 3 3 7 7 8 8 11 11 4 4];
            obstacles_vertices{3} = [1 4 4 5 5 12 12 5 5 4 4 1; 8 8 4 4 12 12 13 13 18 18 9 9];
            obstacles_vertices{4} = [8 8 16 16 19 19 13 13 12 12 10 10 15 15; 9 8 8 16 16 17 17 21 21 17 17 16 16 9];
            obstacles_vertices{5} = [25 26 26 23 23 26 26 25 25 23 23 21 21 20 20 17 17 22 22 25; 6 6 11 11 16 16 21 21 17 17 27 27 29 29 27 27 26 26 10 10];
            obstacles_vertices{6} = [12 19 19 13 13 12; 23 23 24 24 29 29];
            obstacles_vertices{7} = [1 1 7 7 4 4 8 8; 26 25 25 20 20 19 19 26];
            obstacles_vertices{8} = [1 4 4 1; 22 22 23 23];
            obstacles_vertices{9} = [19 26 26 25 25 19; 3 3 6 6 4 4];

        case 5
            % Various rectangular obstacles_vertices and one complex polygonal shape
            obstacles_vertices{1} = [1 5 5 6 6 12 12 6 6 9 8 4 1; 16 16 4 4 12 12 13 13 19 22 23 19 19];
            obstacles_vertices{2} = [9 10 10 16 16 18 18 16 16 15 15 12 12 15 15 9; 1 1 8 8 16 16 17 17 22 22 17 17 16 16 9 9];
            obstacles_vertices{3} = [19 24 24 20 25 25 20 20 19 13 16 14 9 10 12 19 19 20 20 24 19; 7 7 8 8 22 23 23 25 26 26 29 29 24 23 25 25 20 20 22 22 9];
            obstacles_vertices{4} = [21 29 29 21; 3 3 4 4];
            obstacles_vertices{5} = [26 29 29; 12 12 16];
            obstacles_vertices{6} = [8 9 10 9; 23 22 23 24];
            obstacles_vertices{7} = [1 4 14 1; 19 19 29 29];

        case 12
            % Combination of rectangular and circular obstacles_vertices
            obstacles_vertices{1} = [1 11 11 1; 29 29 25 25];
            obstacles_vertices{2} = [4 10 10 4; 13 13 4 4];
            obstacles_vertices{3} = [12 23 23 20 20 12; 27 27 17 17 24 24];
            obstacles_vertices{4} = [13 17 17 13; 19 19 8 8];
            obstacles_vertices{5} = [13 18 18 13; 4 4 1 1];
            obstacles_vertices{6} = generateCircles(7, 19, 3, numPoints);
            obstacles_vertices{7} = generateCircles(25, 9, 4, numPoints);

        case 13
            % Large set of circles
            centerX = [2 3 4 7 8 8 10 11 12 12 13 14 15 16 16 17 19 20 21 21 22 23 24 25 26 26];
            centerY = [21 6 16 12 7 21 13 26 23 10 5 20 15 27 11 6 12 20 16 8 27 3 23 11 15 6];
            obstacles_vertices = generateCircles(centerX, centerY, 1, numPoints);
            % Additional larger circles
            obstacles_vertices{27} = generateCircles(4, 11, 1.5, numPoints);
            obstacles_vertices{28} = generateCircles(5, 25, 1.5, numPoints);
            obstacles_vertices{29} = generateCircles(10, 17, 1.5, numPoints);

        case 14
            % Another set of circles
            centerX = [4 8 12 16 16 21 21 23 24 26];
            centerY = [16 7 23 27 11 16 8 3 23 15];
            obstacles_vertices = generateCircles(centerX, centerY, 1, numPoints);
            % Additional larger circles
            obstacles_vertices{11} = generateCircles(5, 25, 1.5, numPoints);
            obstacles_vertices{12} = generateCircles(10, 17, 1.5, numPoints);
            obstacles_vertices{13} = generateCircles(16, 3, 1.5, numPoints);
            obstacles_vertices{14} = generateCircles(26, 19, 1.5, numPoints);

        case 15
            % Final case with rectangular obstacles_vertices
            obstacles_vertices{1} = [1 13 13 12 12 1; 12 12 8 8 11 11];
            obstacles_vertices{2} = [8 8 12 12 8 8 7 7; 29 26 26 25 25 19 19 29];
            obstacles_vertices{3} = [15 20 20 19 19 15; 18 18 1 1 17 17];
            obstacles_vertices{4} = [18 29 29 18; 23 23 22 22];
            obstacles_vertices{5} = [25 29 29 25; 12 12 11 11];
    end
    
    % Helper function for generating circles
    function block = generateCircles(centerX, centerY, radius, numPoints)
        theta = linspace(0, 2*pi, numPoints);  % Generate the range of angles
        x = centerX + radius * cos(theta);     % X coordinates of the circle
        y = centerY + radius * sin(theta);     % Y coordinates of the circle
        block = [x; y];                        % Store the coordinates of the circle
    end
end
