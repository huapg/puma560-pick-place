import mujoco
import mujoco.viewer
import time
import numpy as np
from scipy.optimize import minimize

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

ee_name = "wrist_3_link"
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name)

# 目标末端位置
target_pos = np.array([0.5, -0.3, 0.2])  # x, y, z


def cost(q):
    """代价函数：末端离目标有多远"""
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)
    ee_pos = data.xpos[ee_id]
    return np.sum((ee_pos - target_pos) ** 2)


# 用优化求关节角
print("正在求解逆运动学...")
q0 = data.qpos[:6].copy()
result = minimize(cost, q0, method='BFGS')
q_sol = result.x

print(f"目标位置: {target_pos}")
print(f"求解关节角: {q_sol}")

# 让机械臂动过去
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 平滑过渡到求解的关节角
        alpha = 0.02
        data.ctrl[:] = data.ctrl[:] + alpha * (q_sol - data.ctrl[:])

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
