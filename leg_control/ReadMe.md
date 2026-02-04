# 使用方法（ROS 2 + Gazebo）

## 1. 目录要求（重要）
- **leg_control 必须位于你的 Home 目录下**：`~/leg_control`
- 先确认目录位置正确：

```bash
cd ~
ls
```

看到 `leg_control` 目录即为正确。

---

## 2. 配置 URDF 中的 mesh 绝对路径
1. 打开 URDF 文件：
   - 路径：`src/leg_model1/urdf/leg_model_1.urdf`

2. 在该 URDF 内找到所有类似下面的引用（**共 12 个**）：
   - `file:///home/username/leg_control/install/leg_model_1/share/leg_model_1/meshes/Knee_link.STL`

3. 将其中的 `username` 替换为你自己的用户名（例如 `zhr`）：
   - 示例：`file:///home/zhr/leg_control/install/leg_model_1/share/leg_model_1/meshes/Knee_link.STL`

> 提示：如果你不确定用户名，可用 `whoami` 查看。

---

## 3. 编译与环境加载
在工作空间根目录执行：

```bash
cd ~/leg_control
colcon build
source install/setup.bash
```

---

## 4. 启动仿真
> 建议先启动显示（RViz/可视化），再启动 Gazebo。

```bash
cd ~/leg_control
source install/setup.bash

ros2 launch leg_model_1 display.launch.py
ros2 launch leg_model_1 gazebo.launch.py
```

---

## 5. 常见问题（排查思路）
- **找不到 STL / mesh 显示为空**
  - 多数是 URDF 里的 `username` 未替换或路径不正确（请确认 12 处都已修改）。
- **launch 找不到包 / 命令无效**
  - 确认已执行：`source install/setup.bash`
  - 确认当前终端位于同一个工作空间（`~/leg_control`）下构建过。