%% =====================================================================
%  trapvel_demo.m — 梯形速度轨迹（加-匀-减）
%  功能：三段式速度规划（加速-匀速-减速），含速度曲线图与动画
%  用法：直接运行  run trajectory/trapvel_demo.m
%  依赖：Robotics System Toolbox
% =====================================================================
%% 梯形速度轨迹：加-匀-减
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';

q_start = zeros(1,6);
q_goal = [0.7644 -0.9794 0.1182 0 0.8611 -0.7644];

% 生成梯形速度轨迹
waypoints = [q_start', q_goal'];          % 6×2：两个路径点
[q_traj, qd_traj, qdd_traj] = trapveltraj(waypoints, 100);
q_traj = q_traj';                          % 转成 100×6

% 画速度曲线
figure;
subplot(2,1,1);
plot(q_traj, 'LineWidth', 1.5);
title('关节位置（平滑过渡）');
subplot(2,1,2);
plot(qd_traj', 'LineWidth', 1.5);
title('关节速度（梯形：加速→匀速→减速）');
xlabel('步');

% 动画
figure;
for i = 1:100
    show(puma, q_traj(i,:));
    title(sprintf('第 %d / 100 步', i, 100));
    drawnow;
end
