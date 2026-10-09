import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "obstacle")

print("开始运动...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    i = 0
    while viewer.is_running():
        # 检查碰撞
        if data.ncon > 0:
            data.ctrl[:] = 0
            if i % 50 == 0:
                print(f"碰撞！紧急停止！接触数: {data.ncon}")
        else:
            data.ctrl[1] = -1.57 + 0.3 * np.sin(i * 0.002)
            data.ctrl[2] = 1.57 + 0.3 * np.sin(i * 0.002)

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        # 每50步打印一次位置
        if i % 50 == 0:
            ee_pos = data.xpos[ee_id]
            box_pos = data.geom_xpos[box_id]
            dist = np.linalg.norm(ee_pos - box_pos)
            print(f"末端: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})  离箱子: {dist:.3f}")

        i += 1
