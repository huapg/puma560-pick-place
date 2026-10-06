%% =====================================================================
%  pid_demo.m — PID 控制器参数对比
%  功能：四宫格对比不同 Kp/Ki/Kd 组合的阶跃响应差异
%  用法：直接运行  run control/pid_demo.m
%  依赖：MATLAB Control System Toolbox
% =====================================================================
%% PID 四宫格对比
clear; clc; close all;
sys = tf(1, [1 0.5 1]);

figure('Position', [100 100 1000 800]);

subplot(2,2,1);
step(feedback(pid(1,0,0)*sys, 1));
title('Kp=1, Ki=0, Kd=0'); grid on;

subplot(2,2,2);
step(feedback(pid(10,0,0)*sys, 1));
title('Kp=10, Ki=0, Kd=0'); grid on;

subplot(2,2,3);
step(feedback(pid(50,0,0)*sys, 1));
title('Kp=50, Ki=0, Kd=0'); grid on;

subplot(2,2,4);
step(feedback(pid(50,5,1)*sys, 1));
title('Kp=50, Ki=5, Kd=1'); grid on;
