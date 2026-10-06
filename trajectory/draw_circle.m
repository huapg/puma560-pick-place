%% 让机械臂末端画一个圆
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';
eeName = 'link7';
ik = inverseKinematics('RigidBodyTree', puma);
weights = [0.25 0.25 0.25 1 1 1];

% 圆的参数：圆心、半径、高度
center_x = 0.5;
center_y = 0;
center_z = 0.8;
radius = 0.3;

% 生成圆上的点（50个）
theta = linspace(0, 2*pi, 50);
circle_x = center_x + radius*cos(theta);
circle_y = center_y + radius*sin(theta);
circle_z = center_z * ones(size(theta));

% 对每个点求 IK
q_traj = zeros(50, 6);
q_guess = zeros(1,6);
for i = 1:50
    T = getTransform(puma, q_guess, eeName);
    T(1,4) = circle_x(i);
    T(2,4) = circle_y(i);
    T(3,4) = circle_z(i);
    [q, ~] = ik(eeName, T, weights, q_guess);
    q_traj(i,:) = q;
    q_guess = q;   % 用上一个解作为下一个的初始猜测
end

% 动画 + 画末端轨迹
figure;
ee_path = [];
for i = 1:50
    show(puma, q_traj(i,:));
    hold on;
    T_now = getTransform(puma, q_traj(i,:), eeName);
    ee_path = [ee_path; T_now(1,4), T_now(2,4), T_now(3,4)];
    plot3(ee_path(:,1), ee_path(:,2), ee_path(:,3), 'r.-', 'LineWidth', 2);
    plot3(circle_x, circle_y, circle_z, 'k--');
    hold off;
    title(sprintf('画圆中... 第%d/50步', i));
    drawnow;
end
