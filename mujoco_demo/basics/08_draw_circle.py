import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)

ee_name = "wrist_3_link"
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name)

trajectory = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    i = 0
    while viewer.is_running():
        t = i * 0.02

        # 控制关节走圆轨迹（近似）
        data.ctrl[0] = -1.57 + 0.3 * np.sin(t)
        data.ctrl[1] = -1.57 + 0.2 * np.cos(t)
        data.ctrl[2] = 1.57 + 0.3 * np.sin(t + 0.5)
        data.ctrl[3] = -1.57
        data.ctrl[4] = -1.57
        data.ctrl[5] = 0.0

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        # 记录末端位置
        ee_pos = data.xpos[ee_id].copy()
        trajectory.append(ee_pos)

        i += 1

        # 跑2000步后画轨迹
        if i == 2000:
            import matplotlib

            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt

            traj = np.array(trajectory)
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'b-')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('End Effector Trajectory')
            plt.show()
