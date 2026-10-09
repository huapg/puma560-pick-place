import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.optimize import minimize

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

ee_name = "wrist_3_link"
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name)


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


# 三个关键位置（末端位置，不是关节角）
# 三个关键位置（末端位置）
pos_home = np.array([0.0, 0.0, 0.4])      # home：抬起来
pos_pick = np.array([0.4, 0.0, 0.3])       # pick：往前伸，高度适中
pos_place = np.array([0.2, 0.3, 0.4])      # place：移到右边，抬起来

# 用IK求每个位置对应的关节角
print("Solving IK...")
q_home = solve_ik(pos_home, np.array([-1.57, -1.57, 1.57, -1.57, -1.57, 0.0]))
q_pick = solve_ik(pos_pick, q_home)
q_place = solve_ik(pos_place, q_pick)

print(f"Home: {q_home}")
print(f"Pick: {q_pick}")
print(f"Place: {q_place}")

# 动作序列
poses = [q_home, q_pick, q_home, q_place, q_home]
pose_names = ["Home", "Pick", "Home", "Place", "Home"]

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

        if step_in_pose > 800:
            step_in_pose = 0
            current_pose = (current_pose + 1) % len(poses)
            print(f"→ {pose_names[current_pose]}")


# import mujoco
#
# model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
# data = mujoco.MjData(model)
#
# mujoco.mj_resetDataKeyframe(model, data, 0)
#
# ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
# print(f"初始末端位置: {data.xpos[ee_id]}")

# import mujoco
#
# model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
# data = mujoco.MjData(model)
# mujoco.mj_resetDataKeyframe(model, data, 0)
#
# ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
# print(f"末端位置: {data.xpos[ee_id]}")
