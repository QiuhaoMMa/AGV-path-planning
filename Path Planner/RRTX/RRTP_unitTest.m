clear all;clc;close all

scene_id=2;
plotting=10;
environment=createScene(scene_id);
rrt= RRTP(environment,'memory_allocation',10,'occupancy_pdf_resolution',1,'steering_resolution',0.1);
goal_reached=rrt.solve(plotting);
