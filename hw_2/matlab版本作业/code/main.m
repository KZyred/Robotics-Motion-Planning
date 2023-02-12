% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 60.0;
yTarget = 60.0;
MAX_X = 60;
MAX_Y = 60;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
    % map是一个[n,2]的数组，存放所有障碍物点和起终点的坐标，其中第一个点为起点，最后一个点为终点

% Waypoint Generator Using the A* 
[visit_nodes, path] = A_star_search(map, MAX_X,MAX_Y);

% visualize the 2D grid map
visualize_map(map, path, visit_nodes);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
