%% 二阶系统不同阻尼比对比
clear; clc; close all;

wn = 1;   % 固定自然频率

figure;
for zeta = [0.1 0.3 0.7 1.0 2.0]
    G = tf(wn^2, [1 2*zeta*wn wn^2]);
    step(G);
    hold on;
end
legend('ζ=0.1', 'ζ=0.3', 'ζ=0.7', 'ζ=1.0', 'ζ=2.0');
title('不同阻尼比 ζ 的阶跃响应');
grid on;
