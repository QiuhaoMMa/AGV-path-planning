clc
clear
close all

scene_id = 21;
plotting = 1;
environment = createScene(scene_id, false); 
rrt = RRT(environment);
goal_reached = rrt.solve(plotting);


path_indices = rrt.reconstructPath(); 
positions = vertcat(rrt.nodes(path_indices).position);  
disp(positions);                                            

maps = {positions};

for i = 1:length(maps)
    disp(['Simulating map ', num2str(i)])
    simulatePathFollowing(maps{i});
end
