% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
% 生成地图
xStart = 1.0;
yStart = 1.0;
xTarget = 9.0;
yTarget = 9.0;
MAX_X = 10;
MAX_Y = 10;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y); % 返回障碍物出现的地方

% Waypoint Generator Using the A* 
% 路径搜索
path = A_star_search(map, MAX_X,MAX_Y);

% visualize the 2D grid map
% 可视化结果
visualize_map(map, path, []);

% save map
% 保存地图
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
