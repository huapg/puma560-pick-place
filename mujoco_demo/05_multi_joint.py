import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

with mujoco.viewer.launch_passive(model, data) as viewer:
    i = 0
    while viewer.is_running():
        t = i * 0.01

        # 6个关节同时动，各有不同频率
        data.ctrl[0] = -1.57 + 0.3 * np.sin(t)  # 基座转
        data.ctrl[1] = -1.57 + 0.3 * np.sin(t + 1)  # 肩膀抬
        data.ctrl[2] = 1.57 + 0.5 * np.sin(t + 2)  # 手肘弯
        data.ctrl[3] = -1.57 + 0.2 * np.sin(t + 3)  # 手腕1
        data.ctrl[4] = -1.57 + 0.2 * np.sin(t + 4)  # 手腕2
        data.ctrl[5] = 0.0 + 0.3 * np.sin(t + 5)  # 手腕3

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
        i += 1
