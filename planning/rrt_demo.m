%% RRT 路径规划演示（2D平面）
clear; clc; close all;

% 2D 状态空间：x, y, theta
ss = stateSpaceSE2;

% 占用栅格地图
map = occupancyMap(zeros(50,50));
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;

% RRT 规划器
planner = plannerRRT(ss, sv);
planner.MaxConnectionDistance = 0.5;
planner.MaxIterations = 5000;

% 起点和终点
start = [5 5 0];
goal  = [45 45 0];

[pth, solInfo] = plan(planner, start, goal);

% 画图
figure;
show(sv.Map); hold on;
plot(start(1), start(2), 'go', 'MarkerSize', 15, 'LineWidth', 3);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 15, 'LineWidth', 3);
if solInfo.IsPathFound
    plot(pth.States(:,1), pth.States(:,2), 'b.-', 'LineWidth', 2);
    title('RRT 规划出的路径（蓝线）');
else
    title('未找到路径');
end
