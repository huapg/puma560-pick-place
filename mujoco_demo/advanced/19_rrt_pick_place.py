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


# Pick-Place + RRT路径
poses_pos = [
    np.array([0.0, 0.0, 0.5]),  # 0: Home
    np.array([0.48, 0.0, 0.6]),  # 1: Above cube1
    np.array([0.48, 0.0, 0.15]),  # 2: Down cube1
    np.array([0.48, 0.0, 0.6]),  # 3: Lift cube1

    # RRT 路径点（绕开障碍物）
    np.array([0.543, 0.088, 0.620]),  # 4: RRT1
    np.array([0.519, 0.184, 0.605]),  # 5: RRT2
    np.array([0.430, 0.231, 0.603]),  # 6: RRT3
    np.array([0.342, 0.278, 0.601]),  # 7: RRT4

    np.array([0.3, 0.3, 0.6]),  # 8: Above B1
    np.array([0.3, 0.3, 0.15]),  # 9: Down B1（放下cube1）
    np.array([0.3, 0.3, 0.6]),  # 10: Lift B1

    np.array([0.39, 0.2, 0.6]),  # 11: Above cube2
    np.array([0.39, 0.2, 0.15]),  # 12: Down cube2
    np.array([0.39, 0.2, 0.6]),  # 13: Lift cube2
    np.array([0.3, 0.45, 0.6]),  # 14: Above B2
    np.array([0.3, 0.45, 0.15]),  # 15: Down B2（放下cube2）
    np.array([0.3, 0.45, 0.6]),  # 16: Lift B2
    np.array([0.0, 0.0, 0.5]),  # 17: Home
]
pose_names = ["Home", "Above C1", "Down C1", "Lift C1",
              "RRT1", "RRT2", "RRT3", "RRT4",
              "Above B1", "Down B1", "Lift B1",
              "Above C2", "Down C2", "Lift C2",
              "Above B2", "Down B2", "Lift B2", "Home"]

print("Solving IK...")
qs = []
q_prev = data.qpos[:6].copy()
for pos in poses_pos:
    q = solve_ik_pose(pos, quat_down, q_prev)
    qs.append(q)
    q_prev = q

data.qpos[:6] = qs[0]
data.ctrl[:] = qs[0]

cube_held = None

print("开始RRT路径Pick-Place...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    current_pose = 0
    step_in_pose = 0
    i = 0

    while viewer.is_running():
        target = qs[current_pose]
        alpha = 0.02
        data.ctrl[:] = data.ctrl[:] + alpha * (target - data.ctrl[:])

        mujoco.mj_step(model, data)

        if cube_held == "cube1":
            data.qpos[6:9] = data.xpos[ee_id] - np.array([0, 0, 0.15])
            data.qpos[9:13] = np.array([1, 0, 0, 0])
        elif cube_held == "cube2":
            data.qpos[13:16] = data.xpos[ee_id] - np.array([0, 0, 0.15])
            data.qpos[16:20] = np.array([1, 0, 0, 0])

        viewer.sync()
        time.sleep(0.001)

        step_in_pose += 1
        i += 1

        if step_in_pose > 1000:
            step_in_pose = 0

            current_name = pose_names[current_pose]

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
