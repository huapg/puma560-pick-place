%% =====================================================================
%  traj_demo.m — 线性插值轨迹动画
%  功能：50 步从起点线性插值到终点，关节空间轨迹动画演示
%  用法：直接运行  run trajectory/traj_demo.m
%  依赖：Robotics System Toolbox
% =====================================================================
%% 轨迹规划：让机械臂从A平滑走到B
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';

% 起点：零位
q_start = zeros(1,6);

% 终点：用刚才 IK 算出的那个姿势
q_goal = [0.7644 -0.9794 0.1182 0 0.8611 -0.7644];

% 生成中间轨迹：50步
numSteps = 50;
q_traj = zeros(numSteps, 6);
for i = 1:6
    q_traj(:,i) = linspace(q_start(i), q_goal(i), numSteps);
end

% 动画：逐帧显示
figure;
for i = 1:numSteps
    show(puma, q_traj(i,:));
    title(sprintf('第 %d / %d 步', i, numSteps));
    drawnow;
end
