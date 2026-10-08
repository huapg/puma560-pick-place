# 机械臂仿真与控制项目

基于 MATLAB 和 MuJoCo 的机械臂运动学、轨迹规划与控制仿真。从手写 2R 正逆运动学到 6 自由度 Puma560 轨迹跟踪控制，再到 MuJoCo 真实物理仿真下的 UR5e Pick-Place 作业，覆盖机器人学核心知识点。

---

## 项目内容

### Part 1：MATLAB 仿真

#### 1. 运动学（手写实现）
- **正运动学**：手写 2R 机械臂 cos/sin 直接求解
- **解析逆运动学**：余弦定理求 θ2，几何关系求 θ1，肘上/肘下两解
- **数值逆运动学**：牛顿-拉夫逊法，雅可比迭代求解
- **全手写仿真**：正运动学 + 逆运动学 + 梯形速度轨迹，不用工具箱

#### 2. 轨迹规划
- 线性插值轨迹
- 梯形速度轨迹（加-匀-减）
- 圆周轨迹生成
- RRT 路径规划

#### 3. 控制
- PID 阶跃响应分析（P/I/D 三参数作用）
- 单关节机械臂 PID 控制
- 6 自由度机械臂轨迹跟踪 PID 控制
- Simulink 闭环仿真

#### 4. 工业应用
- Puma560 六轴机械臂建模
- 完整 Pick-Place 动作序列（接近→下降→抓取→抬起→平移→下降→放置）

---

### Part 2：MuJoCo 物理仿真

#### 1. 模型加载与基础控制
- UR5e 六自由度机械臂模型加载
- 重力下塌现象观察
- 单关节位置控制
- 多关节协调控制

#### 2. 运动学
- 正运动学验证（关节角 → 末端位置）
- 末端轨迹可视化
- 逆运动学求解（scipy.optimize 优化法）

#### 3. 控制与规划
- 阶跃响应分析
- 正弦轨迹跟踪
- IK + 姿态控制（四元数约束）
- 笛卡尔空间控制

#### 4. 完整 Pick-Place 作业
- Home → 上方 → 下降 → 抬起 → 目标位置 → Home 完整动作序列
- 碰撞检测与紧急停止
- 末端姿态保持（全程朝下）
- 平滑轨迹插值

---

## 文件夹结构

```
robotics/
├── README.md
├── basics/                  # MATLAB基础建模
├── kinematics/              # MATLAB运动学
├── trajectory/             # MATLAB轨迹规划
├── planning/                # MATLAB运动规划
├── control/                 # MATLAB控制
├── main/                    # MATLAB主程序
├── mujoco_demo/             # MuJoCo物理仿真
│   ├── 01_load_ur5e.py      # 加载UR5e模型
│   ├── 02_gravity_drop.py   # 重力下塌
│   ├── 03_position_control.py  # 位置控制
│   ├── 04_sine_tracking.py  # 正弦跟踪
│   ├── 05_multi_joint.py    # 多关节协调
│   ├── 06_step_response.py  # 阶跃响应
│   ├── 07_forward_kinematics.py  # 正运动学
│   ├── 08_draw_circle.py    # 末端轨迹
│   ├── 09_pick_place.py     # 关节空间Pick-Place
│   ├── 10_inverse_kinematics.py  # 逆运动学
│   ├── 11_pick_place_with_ik.py  # IK版Pick-Place
│   ├── 12_pose_control.py   # 姿态控制
│   ├── 13_collision.py      # 碰撞检测
│   ├── 14_cartesian_control.py  # 笛卡尔控制
│   ├── 15_full_pick_place.py  # 完整Pick-Place
│   └── ur5e/               # UR5e模型文件
└── (以上为MATLAB部分文件夹)
```

---

## 核心知识点

### 运动学
- DH 参数、正逆运动学、多解性、奇异位形、雅可比矩阵
- 解析解 vs 数值解（优化法）
- 姿态表示：四元数

### 轨迹规划
- 线性插值、梯形速度、加减速平滑
- Pick-Place 动作序列设计

### 控制
- PID 三参数作用、二阶系统响应、轨迹跟踪
- 位置控制 vs 力矩控制
- 平滑插值控制（指数趋近）

### 工业应用
- Puma560 建模、UR5e 建模
- Pick-Place 完整作业
- 碰撞检测与安全保护

---

## 环境要求

### MATLAB 部分
- MATLAB R2024a
- Robotics System Toolbox
- Control System Toolbox

### MuJoCo 部分
- Python 3.10+
- MuJoCo 3.15
- NumPy
- SciPy

---

## 技术栈

`MATLAB` · `Simulink` · `MuJoCo` · `Python` · `NumPy` · `SciPy` · `URDF` · `机械臂运动学` · `逆运动学` · `轨迹规划` · `PID控制` · `碰撞检测` · `Pick-Place`
