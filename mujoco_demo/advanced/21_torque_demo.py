import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

print("开始力矩控制...")
print("给关节1加力矩，看看会怎样")

with mujoco.viewer.launch_passive(model, data) as viewer:
    i = 0
    while viewer.is_running():
        # 给关节1加一点力矩
        data.ctrl[0] = 2 * np.sin(i * 0.005)  # 正弦力矩

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
        i += 1
