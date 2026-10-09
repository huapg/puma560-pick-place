import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

# 找到末端执行器的 body id
ee_name = "wrist_3_link"
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name)

with mujoco.viewer.launch_passive(model, data) as viewer:
    i = 0
    while viewer.is_running():
        t = i * 0.01

        # 控制关节动
        data.ctrl[0] = -1.57 + 0.3 * np.sin(t)
        data.ctrl[1] = -1.57 + 0.3 * np.sin(t + 1)
        data.ctrl[2] = 1.57 + 0.5 * np.sin(t + 2)

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        # 读取末端位置
        ee_pos = data.xpos[ee_id]

        # 每100步打印一次
        if i % 100 == 0:
            print(f"末端位置: x={ee_pos[0]:.3f}, y={ee_pos[1]:.3f}, z={ee_pos[2]:.3f}")

        i += 1
