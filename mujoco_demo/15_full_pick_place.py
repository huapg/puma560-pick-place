import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.optimize import minimize

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")

# 你刚调好的四元数
quat_down = np.array([-0.700443684, 0.713707675, -3.74725972e-10, 2.57250556e-06])


def solve_ik_pose(target_pos, target_quat, q_init):
    def cost(q):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        ee_pos = data.xpos[ee_id]
        ee_quat = data.xquat[ee_id]

        pos_err = np.sum((ee_pos - target_pos) ** 2)
        pose_err = np.sum((ee_quat - target_quat) ** 2)
        preference = np.sum((q - q_init) ** 2) * 0.01

        return pos_err + pose_err * 1 + preference # 姿态控制权重

    result = minimize(cost, q_init, method='BFGS')
    return result.x


# Pick-Place 动作
poses_pos = [
    np.array([0.0, 0.0, 0.5]),
    np.array([0.5, 0.0, 0.55]),
    np.array([0.5, 0.0, 0.15]),
    np.array([0.5, 0.0, 0.55]),
    np.array([0.3, 0.3, 0.55]),
    np.array([0.3, 0.3, 0.15]),
    np.array([0.3, 0.3, 0.55]),
    np.array([0.0, 0.0, 0.5]),
]
pose_names = ["Home", "Above A", "Down A", "Lift A", "Above B", "Down B", "Lift B", "Home"]

print("Solving IK...")
qs = []
q_prev = data.qpos[:6].copy()
for pos in poses_pos:
    q = solve_ik_pose(pos, quat_down, q_prev)
    qs.append(q)
    q_prev = q

data.qpos[:6] = qs[0]
data.ctrl[:] = qs[0]

print("开始完整Pick-Place...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    current_pose = 0
    step_in_pose = 0
    i = 0

    while viewer.is_running():
        target = qs[current_pose]
        alpha = 0.02
        data.ctrl[:] = data.ctrl[:] + alpha * (target - data.ctrl[:])

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        step_in_pose += 1
        i += 1

        if step_in_pose > 1000:
            step_in_pose = 0
            current_pose = (current_pose + 1) % len(qs)
            print(f"→ {pose_names[current_pose]}")

        if i % 100 == 0:
            ee_pos = data.xpos[ee_id]
            print(f"末端: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")
