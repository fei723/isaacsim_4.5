
# 四轮机器人控制器 ROS 2 节点

本项目实现了一个用于四轮机器人的 **轮子控制器** ROS 2 节点。该节点订阅速度指令（`cmd_vel`），并计算机器人轮子的目标状态，包括转向角度和驱动速度。它支持两种主要的工作模式：**阿克曼转向**（Ackermann Steering）用于前进运动，以及**原地旋转**（In-Place Turning）用于旋转运动。

---

## 功能特性

1. **阿克曼转向**：
   - 根据期望的线速度和角速度，计算前轮的转向角度。
   - 计算每个轮子的速度以实现平滑的前进或后退运动。
   - 在直线行驶时将转向角度归零。

2. **原地旋转**：
   - 计算轮子的速度以实现机器人在原地旋转（线速度为零）。
   - 调整转向角度，确保所有轮子朝正确的方向转动以实现最佳旋转效果。

3. **ROS 2 集成**：
   - 订阅 `Twist` 类型的消息，主题为 `cmd_vel`。
   - 发布 `JointState` 类型的消息，主题为 `target_states`，用于控制机器人的执行器。

4. **可配置参数**：
   - 轴距 (`self.wheelbase`)：前后轴之间的距离。
   - 轮距 (`self.track_width`)：左右轮之间的距离。
   - 轮子半径 (`self.wheel_radius`)：用于将线速度转换为角速度。

---

## 工作原理

1. **输入**：
   - 节点订阅 `cmd_vel` 主题，接收速度指令，消息类型为 `geometry_msgs/msg/Twist`：
     - `linear.x`：前进/后退速度。
     - `linear.y`：横向速度（通常在阿克曼转向中不使用）。
     - `angular.z`：绕垂直轴的旋转速度。

2. **处理**：
   - 如果机器人被命令进行原地旋转（`linear.x == 0 && linear.y == 0 && angular.z != 0`），调用 `in_place_turning` 方法。
   - 否则，调用 `ackermann_steering` 方法计算转向角度和轮子速度。

3. **输出**：
   - 计算得到的关节状态通过 `target_states` 主题发布，消息类型为 `sensor_msgs/msg/JointState`：
     - `name`：关节名称（如 `lb_turn`, `lb_dri` 等）。
     - `position`：每个轮子的目标转向角度。
     - `velocity`：每个轮子的目标角速度。

---

## 使用方法

1. **前提条件**：
   - 安装 ROS 2（测试版本为 Humble 或更高版本）。
   - 安装依赖项：
     ```bash
     sudo apt install ros-<ros2-distro>-rclpy ros-<ros2-distro>-geometry-msgs ros-<ros2-distro>-sensor-msgs
     ```

2. **构建包**：
   - 如果尚未创建 ROS 2 工作空间，请先创建：
     ```bash
     mkdir -p ~/ros2_ws/src
     cd ~/ros2_ws/src
     ```
   - 将本项目克隆到 `src` 目录。
   - 构建工作空间：
     ```bash
     cd ~/ros2_ws
     colcon build
     source install/setup.bash
     ```

3. **运行节点**：
   - 使用以下命令启动节点：
     ```bash
     ros2 run <package_name> wheel_controller
     ```
   - 将 `<package_name>` 替换为你的 ROS 2 包名。

4. **发送指令**：
   - 使用 `teleop_twist_keyboard` 包或其他工具向 `cmd_vel` 主题发布 `Twist` 消息：
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
     ```

5. **查看输出**：
   - 使用 `rostopic echo` 查看 `target_states` 主题的输出：
     ```bash
     ros2 topic echo /target_states
     ```

---

## 自定义

你可以在 `WheelController` 类中调整以下参数以匹配你的机器人物理尺寸：
- `self.wheelbase`：前后轴之间的距离。
- `self.track_width`：左右轮之间的距离。
- `self.wheel_radius`：轮子的半径。

---

## 注意事项

- **错误处理**：节点包含基本的错误处理机制，用于记录消息处理中的问题。
- **安全性**：确保机器人的执行器经过校准，以正确处理计算出的关节状态。
- **局限性**：
  - 假设机器人是一个四轮车辆，采用阿克曼转向几何结构。
  - 不考虑动态效应，例如打滑或惯性。

---

## 示例

如果机器人接收到以下 `cmd_vel` 消息：
- `linear.x = 1.0`（前进运动），
- `angular.z = 0.5`（向右转），

节点将计算前轮的适当转向角度，并设置轮子的速度以实现期望的运动。
