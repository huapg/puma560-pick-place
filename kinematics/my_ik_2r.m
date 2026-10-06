%% 自己写2R逆运动学
clear; clc; close all;

L1 = 1; L2 = 1;

% 目标点（你自己改）
x_target = 0.5;
y_target = 0.8;

% 第一步：求 θ2
cos_theta2 = (x_target^2 + y_target^2 - L1^2 - L2^2) / (2*L1*L2);

% 检查能不能到
if abs(cos_theta2) > 1
    error('目标点超出工作空间！');
end

% 两个解：肘上和肘下
theta2_up   =  atan2( sqrt(1-cos_theta2^2), cos_theta2);
theta2_down =  atan2(-sqrt(1-cos_theta2^2), cos_theta2);

% 第二步：求 θ1
phi = atan2(y_target, x_target);
beta_up   = atan2(L2*sin(theta2_up),   L1+L2*cos(theta2_up));
beta_down = atan2(L2*sin(theta2_down), L1+L2*cos(theta2_down));

theta1_up   = phi - beta_up;
theta1_down = phi - beta_down;

% 打印结果
fprintf('目标点 (%.2f, %.2f)\n', x_target, y_target);
fprintf('肘上解: θ1=%.2f°  θ2=%.2f°\n', theta1_up*180/pi, theta2_up*180/pi);
fprintf('肘下解: θ1=%.2f°  θ2=%.2f°\n', theta1_down*180/pi, theta2_down*180/pi);

% 画图对比两种解
figure;
% 肘上
x1_up = L1*cos(theta1_up);  y1_up = L1*sin(theta1_up);
x2_up = x1_up + L2*cos(theta1_up+theta2_up);
y2_up = y1_up + L2*sin(theta1_up+theta2_up);

x1_dn = L1*cos(theta1_down);  y1_dn = L1*sin(theta1_down);
x2_dn = x1_dn + L2*cos(theta1_down+theta2_down);
y2_dn = y1_dn + L2*sin(theta1_down+theta2_down);

plot([0 x1_up x2_up], [0 y1_up y2_up], 'b-o', 'LineWidth', 2); hold on;
plot([0 x1_dn x2_dn], [0 y1_dn y2_dn], 'r-o', 'LineWidth', 2);
plot(x_target, y_target, 'k*', 'MarkerSize', 15, 'LineWidth', 2);
axis equal; grid on;
legend('肘上解', '肘下解', '目标点');
title('自己写的2R逆运动学：两种解都到同一个点');
