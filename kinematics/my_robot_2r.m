%% 全手写2R机械臂仿真：正运动学 + 逆运动学 + 轨迹规划
clear; clc; close all;

L1 = 1; L2 = 1;

%% ===== 1. 起点和终点 =====
start_point = [0.5, 0.8];   % 起点
end_point   = [1.2, 0.5];   % 终点

%% ===== 2. 逆运动学：从点求关节角（解析法）=====
function q = my_ik(x_target, y_target, L1, L2)
    cos_theta2 = (x_target^2 + y_target^2 - L1^2 - L2^2) / (2*L1*L2);
    theta2 = atan2(-sqrt(1-cos_theta2^2), cos_theta2);  % 肘下解
    phi = atan2(y_target, x_target);
    beta = atan2(L2*sin(theta2), L1+L2*cos(theta2));
    theta1 = phi - beta;
    q = [theta1; theta2];
end

%% ===== 3. 正运动学：从关节角求位置（手写）=====
function [x, y] = my_fk(q, L1, L2)
    x = L1*cos(q(1)) + L2*cos(q(1)+q(2));
    y = L1*sin(q(1)) + L2*sin(q(1)+q(2));
end

%% ===== 4. 轨迹规划：梯形速度（加-匀-减）=====
N = 50;
q_start = my_ik(start_point(1), start_point(2), L1, L2);
q_end   = my_ik(end_point(1), end_point(2), L1, L2);

% 梯形速度：前20%加速，中间60%匀速，后20%减速
q_traj = zeros(2, N);
for i = 1:N
    t = (i-1)/(N-1);  % 0到1（归一化）
    
    % 梯形速度对应的位置（积分）
    if t < 0.2
        s = t^2 / 0.32;           % 加速段
    elseif t > 0.8
        s = 1 - (1-t)^2 / 0.32;   % 减速段
    else
        s = 1.25*t - 0.125;          % 匀速段
    end
    
    q_traj(1,i) = q_start(1) + s*(q_end(1)-q_start(1));
    q_traj(2,i) = q_start(2) + s*(q_end(2)-q_start(2));
end

%% ===== 5. 动画 =====
figure;
for i = 1:N
    q = q_traj(:,i);
    [x, y] = my_fk(q, L1, L2);
    
    x1 = L1*cos(q(1));  y1 = L1*sin(q(1));
    
    plot([0 x1 x], [0 y1 y], 'b-o', 'LineWidth', 3, 'MarkerSize', 8); hold on;
    plot(start_point(1), start_point(2), 'g*', 'MarkerSize', 15);
    plot(end_point(1), end_point(2), 'r*', 'MarkerSize', 15);
    axis([-1.5 1.5 -1.5 1.5]);
    axis equal; grid on;
    drawnow;
    hold off;
end

fprintf('起点: (%.2f, %.2f) → 终点: (%.2f, %.2f)\n', start_point, end_point);

%% ===== 6. 画位置和速度曲线 =====
figure;

% 位置曲线
subplot(2,1,1);
plot(0:N-1, q_traj(1,:)*180/pi, 'b-', 'LineWidth', 2); hold on;
plot(0:N-1, q_traj(2,:)*180/pi, 'r-', 'LineWidth', 2);
xlabel('步数'); ylabel('关节角（度）');
legend('θ1', 'θ2');
title('关节位置 - 时间');
grid on;

% 速度曲线（差分算速度）
dq = diff(q_traj, 1, 2);  % 相邻两步的差
subplot(2,1,2);
plot(0:N-2, dq(1,:)*180/pi, 'b-', 'LineWidth', 2); hold on;
plot(0:N-2, dq(2,:)*180/pi, 'r-', 'LineWidth', 2);
xlabel('步数'); ylabel('关节速度（度/步）');
legend('θ1速度', 'θ2速度');
title('关节速度 - 时间（梯形形状）');
grid on;

