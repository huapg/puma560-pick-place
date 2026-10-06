import mujoco
import mujoco.viewer

# 加载 UR5e 模型
model = mujoco.MjModel.from_xml_path("D:/matlab project/robotics/mujoco_demo/ur5e/scene.xml")
data = mujoco.MjData(model)

print(f"关节数: {model.nq}")
print(f"执行器数: {model.nu}")

# 打开可视化
mujoco.viewer.launch(model, data)
