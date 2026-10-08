import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")

print("=" * 50)
print("步骤：")
print("1. 点 Pause 暂停")
print("2. 右边 Joint 拖动滑块，调整到末端朝下")
print("3. 调好后关掉窗口")
print("=" * 50)

# 用 launch，按钮和滑块都能用
mujoco.viewer.launch(model, data)

# 关掉窗口后，打印四元数
mujoco.mj_forward(model, data)
print(f"\n你的四元数: {data.xquat[ee_id]}")
print(f"你的位置: {data.xpos[ee_id]}")
