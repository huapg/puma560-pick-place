import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.optimize import minimize

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

ee_name = "wrist_3_link"
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name)


def solve_ik_pose(target_pos, target_quat, q_init):
    """给定目标位置和姿态，求关节角"""

    def cost(q):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)

        # 位置误差
        ee_pos = data.xpos[ee_id]
        pos_err = np.sum((ee_pos - target_pos) ** 2)

        # 姿态误差（四元数差）
        ee_quat = data.xquat[ee_id]
        pose_err = np.sum((ee_quat - target_quat) ** 2)

        # 偏好项
        preference = np.sum((q - q_init) ** 2) * 0.01

        return pos_err * 1.0 + pose_err * 0.1 + preference

    result = minimize(cost, q_init, method='BFGS')
    return result.x


# home姿势
mujoco.mj_resetDataKeyframe(model, data, 0)
q_home = data.qpos[:6].copy()

# 目标：同一个位置，但不同姿态
target_pos = np.array([0.4, 0.0, 0.4])

# 两个不同的姿态
quat_down = np.array([0, 0, 0, 1])  # 朝下
quat_forward = np.array([0.707, 0, 0, 0.707])  # 朝前

print("Solving IK...")
q1 = solve_ik_pose(target_pos, quat_down, q_home)
q2 = solve_ik_pose(target_pos, quat_forward, q_home)

poses = [q_home, q1, q_home, q2, q_home]
pose_names = ["Home", "Down", "Home", "Forward", "Home"]

data.qpos[:6] = q_home
data.ctrl[:] = q_home

with mujoco.viewer.launch_passive(model, data) as viewer:
    current_pose = 0
    step_in_pose = 0
    while viewer.is_running():
        target = poses[current_pose]
        alpha = 0.02
        data.ctrl[:] = data.ctrl[:] + alpha * (target - data.ctrl[:])
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
        step_in_pose += 1
        if step_in_pose > 1000:
            step_in_pose = 0
            current_pose = (current_pose + 1) % len(poses)
            print(f"→ {pose_names[current_pose]}")
