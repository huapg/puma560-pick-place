%% =====================================================================
%  ik_multi.m — 逆运动学多解性实验
%  功能：同一目标点、不同初始猜测 → 得到不同关节角构型（展示 IK 多解性）
%  用法：直接运行  run kinematics/ik_multi.m
%  依赖：Robotics System Toolbox
% =====================================================================
%% IK 多解性实验
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';
eeName = 'link7';
ik = inverseKinematics('RigidBodyTree', puma);
weights = [0.25 0.25 0.25 1 1 1];

% 同一个目标点
q0 = zeros(1,6);
T0 = getTransform(puma, q0, eeName);
Tgoal = T0;
Tgoal(1,4) = 0.5;  Tgoal(2,4) = 0.3;  Tgoal(3,4) = 0.6;

% 用不同的初始猜测，求同一个目标点
q_guess1 = zeros(1,6);
q_guess2 = [pi/2, -pi/2, pi/2, 0, 0, 0];   % 换个初始姿势

[q1, ~] = ik(eeName, Tgoal, weights, q_guess1);
[q2, ~] = ik(eeName, Tgoal, weights, q_guess2);

% 对比画出来
figure;
subplot(1,2,1);
show(puma, q1);
title(sprintf('解1: 从零位猜测'));

subplot(1,2,2);
show(puma, q2);
title(sprintf('解2: 从另一姿势猜测'));

sgtitle('同一个末端位置，两种完全不同的关节姿势');

% 打印关节角对比
disp('解1关节角:'); disp(q1);
disp('解2关节角:'); disp(q2);
