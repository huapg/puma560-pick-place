%% =====================================================================
%  joint_pid.m — 单关节机械臂 PID 控制仿真
%  功能：单关节角度闭环 PID 控制，观察阶跃跟踪与稳态误差
%  用法：直接运行  run control/joint_pid.m（可改 Kp/Ki/Kd）
%  依赖：MATLAB Control System Toolbox
% =====================================================================
%% 单关节机械臂 PID 控制
clear; clc; close all;

% 单关节动力学：J*q_ddot + B*q_dot = tau
J = 1;   % 转动惯量
B = 1;   % 阻尼
G = tf(1, [J B 0]);    % 1/(J*s^2 + B*s)

% ===== 改这三个数 =====
Kp = 50;
Ki = 0;
Kd = 10;
% ======================

C = pid(Kp, Ki, Kd);
sys_cl = feedback(C*G, 1);

step(sys_cl);
title(sprintf('单关节阶跃响应  Kp=%.1f  Ki=%.1f  Kd=%.1f', Kp, Ki, Kd));
grid on;
