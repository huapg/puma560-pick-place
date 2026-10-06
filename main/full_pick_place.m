%% =====================================================================
%  full_pick_place.m — 主程序：完整工业级 Pick-Place 动作序列
%  功能：接近→下降→抓取→抬起→平移→下降→放置→抬起→回零，完整可复现
%  用法：直接运行  run main/full_pick_place.m
%  依赖：Robotics System Toolbox；基于 Puma560 模型
% =====================================================================
%% 完整工业 Pick-Place 流程
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';
eeName = 'link7';
ik = inverseKinematics('RigidBodyTree', puma);
weights = [0.25 0.25 0.25 1 1 1];

q0 = zeros(1,6);
T0 = getTransform(puma, q0, eeName);

% 抓取点 A 和放置点 B
T_A = T0;  T_A(1,4)=0.4;  T_A(2,4)=0.2;  T_A(3,4)=0.3;
T_B = T0;  T_B(1,4)=0.5;  T_B(2,4)=-0.3; T_B(3,4)=0.3;

% 上方位置（比 A/B 高 0.2）
T_A_up = T_A;  T_A_up(3,4) = T_A(3,4) + 0.2;
T_B_up = T_B;  T_B_up(3,4) = T_B(3,4) + 0.2;

% IK 求每个点
[q_A, ~]     = ik(eeName, T_A,     weights, q0);
[q_Aup, ~]   = ik(eeName, T_A_up,  weights, q_A);
[q_Bup, ~]   = ik(eeName, T_B_up,  weights, q_Aup);
[q_B, ~]     = ik(eeName, T_B,     weights, q_Bup);

% 轨迹段：每段60步，停顿20步
seg = @(a,b,n) trapveltraj([a' b'], n);
hold_ = @(q,n) repmat(q', 1, n);

q_traj = [seg(q0, q_Aup, 60), ...
          seg(q_Aup, q_A,  40),  ...
          hold_(q_A, 20),        ...
          seg(q_A, q_Aup,  40),  ...
          seg(q_Aup, q_Bup, 60), ...
          seg(q_Bup, q_B,  40),  ...
          hold_(q_B, 20),        ...
          seg(q_B, q_Bup,  40),  ...
          seg(q_Bup, q0,   60)]';

% 动画
figure;
for i = 1:size(q_traj,1)
    show(puma, q_traj(i,:));
    hold on;
    plot3(T_A(1,4), T_A(2,4), T_A(3,4), 'go', 'MarkerSize', 12, 'LineWidth', 2);
    plot3(T_B(1,4), T_B(2,4), T_B(3,4), 'bo', 'MarkerSize', 12, 'LineWidth', 2);
    plot3(T_A_up(1,4), T_A_up(2,4), T_A_up(3,4), 'g+', 'MarkerSize', 10);
    plot3(T_B_up(1,4), T_B_up(2,4), T_B_up(3,4), 'b+', 'MarkerSize', 10);
    hold off;
    title(sprintf('第%d/%d步', i, size(q_traj,1)));
    drawnow;
end
