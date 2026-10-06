%% =====================================================================
%  puma_compare.m — Puma560 四宫格姿势对比
%  功能：同一机械臂在零位/单关节旋转下的四种姿势并排显示
%  用法：直接运行  run basics/puma_compare.m
%  依赖：Robotics System Toolbox（importrobot('puma560.urdf')）
% =====================================================================
%% Puma560 四个姿势对比
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';

figure('Position', [100 100 1200 900]);

% 子图1：全零
subplot(2,2,1);
q = zeros(1,6);
show(puma, q, 'Parent', gca);
title('全零（基准）');

% 子图2：只转关节1
subplot(2,2,2);
q = zeros(1,6);  q(1) = pi/4;
show(puma, q, 'Parent', gca);
title('关节1转45°');

% 子图3：只转关节2
subplot(2,2,3);
q = zeros(1,6);  q(2) = pi/4;
show(puma, q, 'Parent', gca);
title('关节2转45°');

% 子图4：只转关节3
subplot(2,2,4);
q = zeros(1,6);  q(3) = pi/4;
show(puma, q, 'Parent', gca);
title('关节3转45°');
