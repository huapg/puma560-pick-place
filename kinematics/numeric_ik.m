%% 数值逆运动学：牛顿-拉夫逊法
clear; clc; close all;

L1 = 1; L2 = 1;
x_target = 0.5;
y_target = 0.8;

% 初始猜测（随便给一个）
q = [pi/4; pi/4];

% 迭代
for iter = 1:50
    % 正运动学：当前位置
    x = L1*cos(q(1)) + L2*cos(q(1)+q(2));
    y = L1*sin(q(1)) + L2*sin(q(1)+q(2));
    
    % 误差
    e = [x_target - x; y_target - y];
    
    % 雅可比矩阵（分别求q1，q2偏导）
    J = [-L1*sin(q(1))-L2*sin(q(1)+q(2))  -L2*sin(q(1)+q(2));
          L1*cos(q(1))+L2*cos(q(1)+q(2))   L2*cos(q(1)+q(2))];
    
    % 求解 Δq：J·Δq = e
    dq = J \ e;
    
    % 更新
    q = q + dq;
    
    fprintf('第%d次迭代: 误差=%.6f\n', iter, norm(e));
    
    if norm(e) < 0.001
        break;
    end
end

fprintf('\n最终角度: θ1=%.2f°  θ2=%.2f°\n', q(1)*180/pi, q(2)*180/pi);
