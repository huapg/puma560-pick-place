import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

# 定义几个关键姿势（手动调好的）
# home: 初始姿势
pose_home = np.array([-1.57, -1.57, 1.57, -1.57, -1.57, 0.0])
# above: 抬起来
pose_above = np.array([-1.57, -1.0, 1.0, -1.57, -1.57, 0.0])
# reach: 伸出去
pose_reach = np.array([-1.0, -0.8, 0.8, -1.57, -1.57, 0.0])

# 动作序列：home → above → reach → above → home
poses = [pose_home, pose_above, pose_reach, pose_above, pose_home]
pose_names = ["Home", "Above", "Reach", "Above", "Home"]

with mujoco.viewer.launch_passive(model, data) as viewer:
    current_pose = 0
    step_in_pose = 0

    while viewer.is_running():
        target = poses[current_pose]

        # 平滑插值：从当前位置慢慢过渡到目标
        alpha = 0.02
        data.ctrl[:] = data.ctrl[:] + alpha * (target - data.ctrl[:])

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        step_in_pose += 1

        # 每个姿势停500步
        if step_in_pose > 500:
            step_in_pose = 0
            current_pose = (current_pose + 1) % len(poses)
            print(f"Moving to: {pose_names[current_pose]}")
