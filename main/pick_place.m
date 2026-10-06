%% =====================================================================
%  pick_place.m — 简单 Pick-Place：A→B 往返（进阶版见 main/full_pick_place.m）
%  功能：两目标点间直线往返的基础抓放演示，含动作阶段提示
%  用法：直接运行
%  依赖：Robotics System Toolbox
% =====================================================================
%% Pick-and-Place 综合演示
clear; clc; close all;
puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';
eeName = 'link7';
ik = inverseKinematics('RigidBodyTree', puma);
weights = [0.25 0.25 0.25 1 1 1];

% 四个关键点的笛卡尔位置
q0 = zeros(1,6);
T0 = getTransform(puma, q0, eeName);

T_A = T0;  T_A(1,4)=0.4;  T_A(2,4)=0.2;  T_A(3,4)=0.3;   % 抓取点
T_B = T0;  T_B(1,4)=0.5;  T_B(2,4)=-0.3; T_B(3,4)=0.6;   % 放置点

% IK 求每个点的关节角
[q_A, ~] = ik(eeName, T_A, weights, q0);
[q_B, ~] = ik(eeName, T_B, weights, q0);

% 分段轨迹：q0 -> A -> 停顿 -> B -> 停顿 -> q0
[seg1, ~] = trapveltraj([q0', q_A'], 50);          % q0->A
seg2 = repmat(q_A', 1, 20);                         % 在A停20步（抓取）
[seg3, ~] = trapveltraj([q_A', q_B'], 50);          % A->B
seg4 = repmat(q_B', 1, 20);                         % 在B停20步（放置）
[seg5, ~] = trapveltraj([q_B', q0'], 50);           % B->q0

q_traj = [seg1, seg2, seg3, seg4, seg5]';

% 动画
figure;
for i = 1:size(q_traj,1)
    show(puma, q_traj(i,:));
    hold on;
    plot3(T_A(1,4), T_A(2,4), T_A(3,4), 'go', 'MarkerSize', 15, 'LineWidth', 3);
    plot3(T_B(1,4), T_B(2,4), T_B(3,4), 'bo', 'MarkerSize', 15, 'LineWidth', 3);
    hold off;
    
    % 根据当前帧提示动作
    if i <= 50
        act = '移动到抓取点 A';
    elseif i <= 70
        act = '>>> 抓取中... <<<';
    elseif i <= 120
        act = '移动到放置点 B';
    elseif i <= 140
        act = '>>> 放置中... <<<';
    else
        act = '返回零位';
    end
    title(sprintf('%s   (第%d/%d步)', act, i, size(q_traj,1)));
    drawnow;
end

