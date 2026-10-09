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
        # 目标：第2个关节走正弦波
        t = i * 0.01
        target = -1.57 + 0.5 * np.sin(t)

        # 位置控制：直接给目标位置
        data.ctrl[1] = target

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
        i += 1
