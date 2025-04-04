clear all;clc

scene_id=1;
plotting=1;
environment=createScene(1);
rrt= RRTS(environment);
goal_reached=rrt.solve(plotting);