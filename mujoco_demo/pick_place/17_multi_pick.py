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
cube1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
cube2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube2")

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


# 完整动作序列：
# 抓绿色cube1，搬到B1(0.3, 0.3, 0.075)
# 再抓蓝色cube2，搬到B2(0.3, 0.45, 0.075)
poses_pos = [
    np.array([0.0, 0.0, 0.5]),    # 0: Home
    np.array([0.48, 0.0, 0.6]),    # 1: Above cube1
    np.array([0.48, 0.0, 0.15]),   # 2: Down cube1
    np.array([0.48, 0.0, 0.6]),    # 3: Lift cube1
    np.array([0.3, 0.3, 0.6]),    # 4: Above B1
    np.array([0.3, 0.3, 0.15]),   # 5: Down B1（放下cube1）
    np.array([0.3, 0.3, 0.6]),    # 6: Lift B1
    np.array([0.39, 0.2, 0.6]),    # 7: Above cube2
    np.array([0.39, 0.2, 0.15]),   # 8: Down cube2
    np.array([0.39, 0.2, 0.6]),    # 9: Lift cube2
    np.array([0.3, 0.45, 0.6]),   # 10: Above B2
    np.array([0.3, 0.45, 0.15]),  # 11: Down B2（放下cube2）
    np.array([0.3, 0.45, 0.6]),   # 12: Lift B2
    np.array([0.0, 0.0, 0.5]),    # 13: Home
]
pose_names = ["Home", "Above C1", "Down C1", "Lift C1", "Above B1", "Down B1", "Lift B1",
              "Above C2", "Down C2", "Lift C2", "Above B2", "Down B2", "Lift B2", "Home"]

print("Solving IK...")
qs = []
q_prev = data.qpos[:6].copy()
for pos in poses_pos:
    q = solve_ik_pose(pos, quat_down, q_prev)
    qs.append(q)
    q_prev = q

data.qpos[:6] = qs[0]
data.ctrl[:] = qs[0]

cube_held = None  # None / "cube1" / "cube2"

print("开始多物体连续抓取...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    current_pose = 0
    step_in_pose = 0
    i = 0
    initial_ncon = data.ncon  # 记录初始碰撞数

    while viewer.is_running():
        target = qs[current_pose]
        alpha = 0.02
        data.ctrl[:] = data.ctrl[:] + alpha * (target - data.ctrl[:])

        mujoco.mj_step(model, data)

        # 如果抓住了物体，物体跟着末端走
        if cube_held == "cube1":
            data.qpos[6:9] = data.xpos[ee_id] - np.array([0, 0, 0.15])
            data.qpos[9:13] = np.array([1, 0, 0, 0])
        elif cube_held == "cube2":
            # cube2在qpos[13:20]，位置在13:16，姿态在16:20
            data.qpos[13:16] = data.xpos[ee_id] - np.array([0, 0, 0.15])
            data.qpos[16:20] = np.array([1, 0, 0, 0])

        viewer.sync()
        time.sleep(0.001)

        step_in_pose += 1
        i += 1

        if step_in_pose > 1000:
            step_in_pose = 0

            current_name = pose_names[current_pose]

            # 抓取逻辑
            if current_name == "Down C1":
                cube_held = "cube1"
                print("→ 抓住绿色方块！")
            elif current_name == "Down C2":
                cube_held = "cube2"
                print("→ 抓住蓝色方块！")
            elif current_name == "Down B1":
                cube_held = None
                print("→ 放下绿色方块！")
            elif current_name == "Down B2":
                cube_held = None
                print("→ 放下蓝色方块！")

            current_pose = (current_pose + 1) % len(qs)
            print(f"→ {pose_names[current_pose]}")
