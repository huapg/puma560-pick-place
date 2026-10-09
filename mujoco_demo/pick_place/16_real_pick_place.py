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
cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")

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

        return pos_err + pose_err * 1 + preference

    result = minimize(cost, q_init, method='BFGS')
    return result.x


# 物体A在 (0.5, 0, 0.075)，要搬到 (0.3, 0.3, 0.075)
poses_pos = [
    np.array([0.0, 0.0, 0.5]),
    np.array([0.5, 0.0, 0.6]),
    np.array([0.5, 0.0, 0.15]),
    np.array([0.5, 0.0, 0.6]),
    np.array([0.3, 0.3, 0.6]),
    np.array([0.3, 0.3, 0.15]),
    np.array([0.3, 0.3, 0.6]),
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

# 物体初始位置
cube_pos_initial = np.array([0.5, 0.0, 0.075])
cube_held = False  # 是否抓住了物体

print("开始真实Pick-Place...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    current_pose = 0
    step_in_pose = 0
    i = 0

    while viewer.is_running():
        target = qs[current_pose]
        alpha = 0.02
        data.ctrl[:] = data.ctrl[:] + alpha * (target - data.ctrl[:])

        mujoco.mj_step(model, data)

        # 如果抓住了物体，物体跟着末端走
        if cube_held:
            # 物体位置 = 末端位置 - 向下偏移
            target_cube_pos = data.xpos[ee_id] - np.array([0, 0, 0.15])
            # 物体有7个自由度（3位置+4四元数）
            data.qpos[6:9] = target_cube_pos  # 前6个是机械臂，后面是物体
            # 同时固定姿态，不让它转
            data.qpos[9:13] = np.array([1, 0, 0, 0])  # 四元数，保持水平

        viewer.sync()
        time.sleep(0.001)

        step_in_pose += 1
        i += 1

        if step_in_pose > 1000:
            step_in_pose = 0

            # 到达Down A时，抓住物体
            if pose_names[current_pose] == "Down A":
                cube_held = True
                print("→ 抓住物体！")

            # 到达Down B时，放下物体
            if pose_names[current_pose] == "Down B":
                cube_held = False
                # 把物体位置固定在当前位置
                data.qpos[6:9] = data.xpos[cube_id]
                print("→ 放下物体！")

            current_pose = (current_pose + 1) % len(qs)
            print(f"→ {pose_names[current_pose]}")

        if i % 100 == 0:
            ee_pos = data.xpos[ee_id]
            cube_pos = data.xpos[cube_id]
            print(
                f"末端: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})  物体: ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f})")
