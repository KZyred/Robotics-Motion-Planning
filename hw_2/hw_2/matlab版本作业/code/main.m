% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 50.0;
yTarget = 50.0;
MAX_X = 50;
MAX_Y = 50;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
    % map是一个[n,2]的数组，存放所有障碍物点和起终点的坐标，其中第一个点为起点，最后一个点为终点

% Waypoint Generator Using the A* 
path = A_star_search(map, MAX_X,MAX_Y);

% visualize the 2D grid map
visualize_map(map, path, []);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
