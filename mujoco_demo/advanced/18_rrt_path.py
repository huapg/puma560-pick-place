import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# 起点和终点
start = np.array([0.5, 0.0, 0.6])
goal = np.array([0.3, 0.3, 0.6])

# 障碍物（立方体）
obstacle_center = np.array([0.6, 0.2, 0.2])
obstacle_size = np.array([0.2, 0.2, 0.4])


# 检查点是否在障碍物内
def in_collision(point):
    x, y, z = point
    ox, oy, oz = obstacle_center
    sx, sy, sz = obstacle_size
    return (ox - sx / 2 < x < ox + sx / 2 and
            oy - sy / 2 < y < oy + sy / 2 and
            oz - sz / 2 < z < oz + sz / 2)


# RRT参数
max_iter = 1000
step_size = 0.1

# 树：每个节点是(位置, 父节点索引)
tree = [start]
parents = [-1]

print("开始RRT...")

for i in range(max_iter):
    # 随机采样一个点
    if np.random.random() < 0.1:
        sample = goal  # 10%概率直接采目标点
    else:
        sample = np.array([
            np.random.uniform(-0.5, 0.8),
            np.random.uniform(-0.5, 0.8),
            np.random.uniform(0.2, 0.8)
        ])

    # 找树里离sample最近的节点
    dists = [np.linalg.norm(node - sample) for node in tree]
    nearest_idx = np.argmin(dists)
    nearest = tree[nearest_idx]

    # 从nearest往sample走一步
    direction = sample - nearest
    length = np.linalg.norm(direction)
    if length < step_size:
        new_node = sample
    else:
        new_node = nearest + direction / length * step_size

    # 检查新节点有没有碰障碍物
    if not in_collision(new_node):
        tree.append(new_node)
        parents.append(nearest_idx)

        # 检查到没到目标
        if np.linalg.norm(new_node - goal) < step_size:
            print(f"找到路径！迭代次数: {i}")
            break

# 从目标往回找路径
path = [goal]
current = len(tree) - 1
while parents[current] != -1:
    path.append(tree[current])
    current = parents[current]
path.append(start)
path.reverse()

print(f"路径节点数: {len(path)}")

# 画出来
path = np.array(path)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b.-', label='Path')
ax.scatter(start[0], start[1], start[2], c='g', s=100, label='Start')
ax.scatter(goal[0], goal[1], goal[2], c='r', s=100, label='Goal')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.title('RRT Path')
plt.show()

print("\n路径点：")
for i, p in enumerate(path):
    print(f"  np.array([{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]),")

