%% =====================================================================
%  robot_2r.m — 用 rigidBodyTree 建 2R 机械臂模型
%  功能：工具箱方式建模 2R，与手写版（two_link_arm）对比验证
%  用法：直接运行  run basics/robot_2r.m
%  依赖：Robotics System Toolbox
% =====================================================================
%% 用 Robotics System Toolbox 建 2R 机械臂
clear; clc; close all;

% 1. 创建一个空机器人对象
robot = rigidBodyTree("DataFormat","row");

% 2. 杆1：从基座(base)伸出来
body1 = rigidBody('body1');
jnt1  = rigidBodyJoint('jnt1','revolute');     % 旋转关节
setFixedTransform(jnt1, trvec2tform([0 0 0]));% 关节1在原点
body1.Joint = jnt1;
addBody(robot, body1, 'base');                 % 接到 base 上

% 3. 杆2：接在 body1 的 x=1 处（也就是杆1长1米）
body2 = rigidBody('body2');
jnt2  = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2, trvec2tform([1 0 0]));% 关节2在 body1 末端
body2.Joint = jnt2;
addBody(robot, body2, 'body1');

% 4. 末端点：接在 body2 的 x=1 处（杆2也长1米）
ee = rigidBody('endeffector');
jnt3 = rigidBodyJoint('jnt3','fixed');
setFixedTransform(jnt3, trvec2tform([1 0 0]));% 固定关节，不转
ee.Joint = jnt3;
addBody(robot, ee, 'body2');

% 5. 画出来
show(robot);
