%% =====================================================================
%  two_link_arm.m — 手写 2R 机械臂正运动学
%  功能：用 cos/sin 直接计算末端位置，不依赖工具箱，理解正解原理
%  用法：直接运行  run basics/two_link_arm.m（改 L1/L2/theta 后运行）
%  依赖：无
% =====================================================================
%% 2R 机械臂正运动学 - 你的第一个脚本
clear; clc; close all;      % 清空工作区、命令行、关旧图

%% ====== 改这里就行 ======
L1 = 1;                      % 杆1长度
L2 = 1;                      % 杆2长度
theta1 = 60 * pi/180;        % 关节1角度（度→弧度）
theta2 = -30 * pi/180;        % 关节2角度
%% ========================

%% 正运动学计算
p1 = L1 * [cos(theta1); sin(theta1)];           % 关节2位置
p_end = p1 + L2 * [cos(theta1+theta2); sin(theta1+theta2)];  % 末端

%% 打印结果
fprintf('关节2位置: (%.3f, %.3f)\n', p1(1), p1(2));
fprintf('末端位置:   (%.3f, %.3f)\n', p_end(1), p_end(2));

%% 画图
figure;
plot([0 p1(1)], [0 p1(2)], 'b-o', 'LineWidth', 3, 'MarkerSize', 8); hold on;
plot([p1(1) p_end(1)], [p1(2) p_end(2)], 'r-o', 'LineWidth', 3, 'MarkerSize', 8);
plot(p_end(1), p_end(2), 'k*', 'MarkerSize', 15, 'LineWidth', 2);
plot(0, 0, 'ks', 'MarkerSize', 10, 'LineWidth', 2);

axis equal; axis([-0.5 2.5 -0.5 2.5]);
grid on;
legend('杆1','杆2','末端','关节1', 'Location','southwest');
xlabel('X'); ylabel('Y');
title(sprintf('2R 机械臂  θ1=%.0f°  θ2=%.0f°', theta1*180/pi, theta2*180/pi));
