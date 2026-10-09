import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.optimize import minimize

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "obstacle")


def solve_ik(target_pos, q_init):
    def cost(q):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        ee_pos = data.xpos[ee_id]
        pos_err = np.sum((ee_pos - target_pos) ** 2)
        preference = np.sum((q - q_init) ** 2) * 0.01
        return pos_err + preference

    result = minimize(cost, q_init, method='BFGS')
    return result.x


# 三个关键位置
pos_home = np.array([0.0, 0.0, 0.5])  # 收回来
pos_pick = np.array([0.4, 0.0, 0.4])  # 往前伸
pos_place = np.array([0.2, 0.3, 0.5])  # 移到右边

print("Solving IK...")
q_home = solve_ik(pos_home, data.qpos[:6])
q_pick = solve_ik(pos_pick, q_home)
q_place = solve_ik(pos_place, q_pick)

# 动作序列
poses = [q_home, q_pick, q_home, q_place, q_home]
pose_names = ["Home", "Pick", "Home", "Place", "Home"]

data.qpos[:6] = q_home
data.ctrl[:] = q_home

print("开始Pick-Place...")
ee_pos_last = data.xpos[ee_id].copy()
with mujoco.viewer.launch_passive(model, data) as viewer:
    current_pose = 0
    step_in_pose = 0
    i = 0

    while viewer.is_running():
        target = poses[current_pose]

        # 位置卡住检测
        ee_pos_now = data.xpos[ee_id].copy()
        ctrl_diff = np.linalg.norm(data.ctrl - target)

        stuck = False
        if i > 0:
            delta = np.linalg.norm(ee_pos_now - ee_pos_last)
            if delta < 0.0001 and ctrl_diff > 0.1:
                stuck = True
                data.ctrl[:] = 0
                if i % 50 == 0:
                    print(f"卡住了！位置变化: {delta:.6f}")

        ee_pos_last = ee_pos_now

        if not stuck:
            alpha = 0.02
            data.ctrl[:] = data.ctrl[:] + alpha * (target - data.ctrl[:])

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        step_in_pose += 1
        i += 1

        if step_in_pose > 800:
            step_in_pose = 0
            current_pose = (current_pose + 1) % len(poses)
            print(f"→ {pose_names[current_pose]}")

        if i % 50 == 0:
            ee_pos = data.xpos[ee_id]
            print(f"末端: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        step_in_pose += 1
        i += 1

        if step_in_pose > 800:
            step_in_pose = 0
            current_pose = (current_pose + 1) % len(poses)
            print(f"→ {pose_names[current_pose]}")

        if i % 50 == 0:
            ee_pos = data.xpos[ee_id]
            print(f"末端: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")
