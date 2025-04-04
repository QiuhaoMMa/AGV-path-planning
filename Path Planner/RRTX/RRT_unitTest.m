clear all;clc


scene_id=1;
plotting=1;
environment=createScene(1);
rrt= RRT(environment);
goal_reached=rrt.solve(plotting);





