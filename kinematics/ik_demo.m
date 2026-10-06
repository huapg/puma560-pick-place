%% =====================================================================
%  ik_demo.m — 逆运动学演示
%  功能：给定末端目标点，用 IK 求解关节角并可视化验证
%  用法：直接运行  run kinematics/ik_demo.m
%  依赖：Robotics System Toolbox
% =====================================================================
%% 逆运动学：让末端走到指定点
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';

% 1. 创建 IK 求解器
ik = inverseKinematics('RigidBodyTree', puma);
eeName = 'link7';                       % 末端连杆名

% 2. 初始猜测（零位）
qGuess = zeros(1,6);

% 3. 目标位置：我们希望末端到这里
T0 = getTransform(puma, qGuess, eeName);  % 先取当前位姿
Tgoal = T0;
Tgoal(1,4) = 0.8;    % 目标 x = 0.5 米
Tgoal(2,4) = -0.3;    % 目标 y = 0.3 米
Tgoal(3,4) = 0.4;    % 目标 z = 0.6 米

% 4. 求解
weights = [0.25 0.25 0.25 1 1 1];   % 位置权重，姿态权重
[qSol, info] = ik(eeName, Tgoal, weights, qGuess);

% 5. 画出来
figure;
show(puma, qSol);
hold on;
plot3(Tgoal(1,4), Tgoal(2,4), Tgoal(3,4), ...
      'rp', 'MarkerSize', 25, 'MarkerFaceColor','r');
title('逆运动学：红色五角星 = 目标点，机械臂末端对准它');
