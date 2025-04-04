clear all; clc

scene_id = 5;
plotting = 1;
environment = createScene(scene_id); 
xGoal = [10, 10]; 

informed_rrt_star = InformedRRTStar(environment, xGoal); 
goal_reached = informed_rrt_star.solve(plotting);  

assert(goal_reached, 'Goal was not reached in the Informed RRT* test.');
