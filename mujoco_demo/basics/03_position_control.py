import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

with mujoco.viewer.launch_passive(model, data) as viewer:
    i = 0
    while viewer.is_running():
        # 让第2个关节（shoulder_lift）慢慢动
        # 从 -90度(-1.57弧度) 动到 -60度(-1.05弧度)
        data.ctrl[1] = -1.57 + 0.5 * (i % 1000) / 1000

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
        i += 1
