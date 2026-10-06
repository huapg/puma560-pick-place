# MATLAB 机械臂仿真项目

基于 MATLAB 的机械臂运动学、轨迹规划与控制仿真。从手写 2R 正逆运动学到 6 自由度 Puma560 轨迹跟踪控制，覆盖机器人学核心知识点。

## 项目内容

### 1. 运动学（手写实现）
- **正运动学**：手写 2R 机械臂 cos/sin 直接求解
- **解析逆运动学**：余弦定理求 θ2，几何关系求 θ1，肘上/肘下两解
- **数值逆运动学**：牛顿-拉夫逊法，雅可比迭代求解
- **全手写仿真**：正运动学 + 逆运动学 + 梯形速度轨迹，不用工具箱

### 2. 轨迹规划
- 线性插值轨迹
- 梯形速度轨迹（加-匀-减）
- 圆周轨迹生成
- RRT 路径规划

### 3. 控制
- PID 阶跃响应分析（P/I/D 三参数作用）
- 单关节机械臂 PID 控制
- 6 自由度机械臂轨迹跟踪 PID 控制
- Simulink 闭环仿真

### 4. 工业应用
- Puma560 六轴机械臂建模
- 完整 Pick-Place 动作序列（接近→下降→抓取→抬起→平移→下降→放置）

## 文件夹结构

```
robotics/
├── README.md
├── basics/                  # 基础建模
│   ├── two_link_arm.m        # 手写 2R 正运动学
│   ├── robot_2r.m            # rigidBodyTree 建 2R
│   └── puma_compare.m        # Puma560 六轴模型
├── kinematics/               # 运动学
│   ├── ik_demo.m             # 工具箱逆运动学
│   ├── ik_multi.m            # IK 多解性实验
│   ├── my_ik_2r.m            # 手写解析逆运动学（余弦定理）
│   ├── numeric_ik.m          # 手写数值逆运动学（牛顿-拉夫逊）
│   └── my_robot_2r.m          # 全手写 2R 仿真（FK+IK+梯形轨迹）
├── trajectory/               # 轨迹规划
│   ├── traj_demo.m           # 线性插值
│   ├── trapvel_demo.m        # 梯形速度
│   └── draw_circle.m         # 圆周轨迹
├── planning/                 # 运动规划
│   └── rrt_demo.m            # RRT 路径规划
├── control/                  # 控制
│   ├── pid_demo.m            # PID 参数对比
│   ├── joint_pid.m            # 关节 PID 闭环
│   ├── second_order.m        # 二阶系统分析
│   ├── puma_pid_control.m    # 6轴轨迹跟踪 PID 控制
│   └── test.slx              # Simulink 模型
└── main/                     # 主程序
    ├── pick_place.m          # 简单 Pick-Place
    └── full_pick_place.m     # 完整工业级动作序列
```

## 核心知识点

- **运动学**：DH 参数、正逆运动学、多解性、奇异位形、雅可比矩阵
- **轨迹规划**：线性插值、梯形速度、加减速平滑
- **控制**：PID 三参数作用、二阶系统响应、轨迹跟踪、稳态误差
- **工业应用**：Puma560 建模、Pick-Place 动作序列

## 环境要求

- MATLAB R2024a
- Robotics System Toolbox
- Control System Toolbox

## 技术栈

`MATLAB` · `Robotics System Toolbox` · `Simulink` · `Control System Toolbox` · 机械臂运动学 · 逆运动学 · 轨迹规划 · PID 控制
