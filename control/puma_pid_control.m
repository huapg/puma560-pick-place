%% 6轴机械臂轨迹跟踪：PID控制
clear; clc; close all;

puma = importrobot("puma560.urdf");
puma.DataFormat = 'row';
eeName = 'link7';

% 目标：让机械臂末端走一个圆
N = 100;
t = linspace(0, 2*pi, N);
r = 0.1;  % 圆半径
cx = 0.4; cy = 0; cz = 0.5;

x_target = cx + r*cos(t);
y_target = cy + r*sin(t);
z_target = cz * ones(1, N);

% 逆运动学：每一步求关节角
ik = inverseKinematics('RigidBodyTree', puma);
weights = [0.25 0.25 0.25 1 1 1];
q0 = zeros(1,6);

q_desired = zeros(N, 6);
for i = 1:N
    T = trvec2tform([x_target(i), y_target(i), z_target(i)]);
    [q_sol, ~] = ik(eeName, T, weights, q0);
    q_desired(i,:) = q_sol;
    q0 = q_sol;  % 下一步用当前解做初始猜测
end

% 画期望轨迹
figure;
subplot(2,1,1);
plot(t, q_desired(:,1)*180/pi, 'b-', 'LineWidth', 2); hold on;
plot(t, q_desired(:,2)*180/pi, 'r-', 'LineWidth', 2);
xlabel('时间'); ylabel('关节角（度）');
legend('θ1', 'θ2');
title('期望关节轨迹');
grid on;

subplot(2,1,2);
plot(x_target, y_target, 'k-', 'LineWidth', 2);
xlabel('x'); ylabel('y');
title('末端期望轨迹（圆）');
axis equal; grid on;

fprintf('轨迹生成完成，共%d步\n', N);

%% ===== PID 轨迹跟踪 =====
% 假设每个关节是独立的二阶系统：J*q_ddot + B*q_dot = tau
J = 1;   % 转动惯量
B = 5;   % 阻尼

% PID参数（6个关节各一套，这里先用一样的）
Kp = 200;
Ki = 50;
Kd = 30;

% 仿真
dt = 0.05;  % 时间步长
q_actual = zeros(N, 6);     % 实际关节角
q_dot_actual = zeros(N, 6); % 实际关节角速度
integral_error = zeros(1, 6);

q_actual(1,:) = q_desired(1,:);  % 初始位置等于第一步

for i = 2:N
    % 误差
    e = q_desired(i,:) - q_actual(i-1,:);
    
    % 积分
    integral_error = integral_error + e*dt;
    
    % 微分（用角速度近似）
    derivative = -q_dot_actual(i-1,:);
    
    % PID输出：力矩
    tau = Kp*e + Ki*integral_error + Kd*derivative;
    
    % 二阶系统：J*q_ddot = tau - B*q_dot
    q_ddot = (tau - B*q_dot_actual(i-1,:)) / J;
    
    % 积分更新
    q_dot_actual(i,:) = q_dot_actual(i-1,:) + q_ddot*dt;
    q_actual(i,:) = q_actual(i-1,:) + q_dot_actual(i,:)*dt;
end

% 画跟踪误差
figure;
subplot(2,1,1);
plot(t, q_desired(:,1)*180/pi, 'b--', 'LineWidth', 2); hold on;
plot(t, q_actual(:,1)*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('时间'); ylabel('θ1（度）');
legend('期望', '实际');
title('关节1跟踪效果');
grid on;

subplot(2,1,2);
error = (q_desired - q_actual)*180/pi;
plot(t, error(:,1), 'r-', 'LineWidth', 1.5);
xlabel('时间'); ylabel('误差（度）');
title('关节1跟踪误差');
grid on;
