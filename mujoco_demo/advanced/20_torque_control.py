import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

print("关节数:", model.nq)
print("力矩数:", model.nu)

# 先看看ur5e是位置控制还是力矩控制
print("控制类型:", model.actuator_gaintype)
