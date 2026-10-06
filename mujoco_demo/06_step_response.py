import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

angles = []
targets = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    i = 0
    while viewer.is_running():
        if i < 200:
            target = -1.57
        else:
            target = -0.5

        data.ctrl[1] = target

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        angles.append(data.qpos[1])
        targets.append(target)
        i += 1

        if i == 500:
            plt.figure()
            plt.plot(angles, label='Actual')
            plt.plot(targets, '--', label='Target')
            plt.legend()
            plt.title('Step Response')
            plt.xlabel('Step')
            plt.ylabel('Angle (rad)')
            plt.grid(True)
            plt.show()
