import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

# 先重置到 home 姿势（臂抬起来的）
mujoco.mj_resetDataKeyframe(model, data, 0)
print("初始姿势: home（臂抬起来的）")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 不给任何控制！让它自由落体
        data.ctrl[:] = 0  # 所有电机都不给力

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
