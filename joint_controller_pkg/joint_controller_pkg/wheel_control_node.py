import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

# 增强型 PID 控制器类
class PIDController:
    def __init__(self, kp, ki, kd, max_integral=50.0, output_limit=(-100.0, 100.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.max_integral = max_integral
        self.output_low, self.output_high = output_limit

    def update(self, setpoint, current_value):
        if math.isnan(setpoint) or math.isnan(current_value):
            return 0.0

        error = setpoint - current_value
        self.integral += error
        # 积分项限幅
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral

        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        
        # 输出限幅
        output = max(self.output_low, min(output, self.output_high))
        return output


class WheelClosedLoopControlNode(Node):
    def __init__(self):
        super().__init__('wheel_closed_loop_control_node')
        # 订阅目标状态
        self.target_states_sub = self.create_subscription(
            JointState, '/target_states', self.target_states_callback, 10)
        # 订阅当前状态
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        # 发布控制指令
        self.joint_command_pub = self.create_publisher(JointState, 'joint_command', 10)

        # 初始化 PID 控制器（关键参数调整）
        self.steering_pid = PIDController(kp=3.0, ki=0.1, kd=0.5, output_limit=(-1.57, 1.57))
        self.speed_pid = PIDController(kp=0.7, ki=0.1, kd=0.1, output_limit=(-20.0, 20.0))

        # 存储目标值
        self.target_speeds = [0.0] * 4
        self.target_angles = [0.0] * 4
        # 存储当前值
        self.current_speeds = [0.0] * 4
        self.current_angles = [0.0] * 4

    def target_states_callback(self, msg):
        wheel_names = ['lb', 'lf', 'rb', 'rf']
        try:
            for i, name in enumerate(wheel_names):
                turn_idx = msg.name.index(f'{name}_turn')
                dri_idx = msg.name.index(f'{name}_dri')
                self.target_angles[i] = msg.position[turn_idx]
                self.target_speeds[i] = msg.velocity[dri_idx]
            self.get_logger().info(f"目标速度: {self.target_speeds}")
        except ValueError as e:
            self.get_logger().error(f"目标状态解析错误: {e}")

    def joint_states_callback(self, msg):
        try:
            # 解析当前状态
            lb_dri = msg.name.index('lb_dri')
            lf_dri = msg.name.index('lf_dri')
            rb_dri = msg.name.index('rb_dri')
            rf_dri = msg.name.index('rf_dri')
            
            lb_turn = msg.name.index('lb_turn')
            lf_turn = msg.name.index('lf_turn')
            rb_turn = msg.name.index('rb_turn')
            rf_turn = msg.name.index('rf_turn')

            self.current_speeds = [
                msg.velocity[lb_dri],
                msg.velocity[lf_dri],
                msg.velocity[rb_dri],
                msg.velocity[rf_dri]
            ]
            self.current_angles = [
                msg.position[lb_turn],
                msg.position[lf_turn],
                msg.position[rb_turn],
                msg.position[rf_turn]
            ]
            self.get_logger().debug(f"当前速度: {self.current_speeds}")
        except ValueError as e:
            self.get_logger().error(f"当前状态解析错误: {e}")
            return

        # 计算控制信号
        control_angles = [
            self.steering_pid.update(self.target_angles[i], self.current_angles[i])
            for i in range(4)
        ]
        control_speeds = [
            self.speed_pid.update(self.target_speeds[i], self.current_speeds[i])
            for i in range(4)
        ]
        control_speeds = self.target_speeds
        # 构造控制消息
        joint_command = JointState()
        joint_command.name = [
            'lb_dri', 'lb_turn',
            'lf_dri', 'lf_turn',
            'rb_dri', 'rb_turn',
            'rf_dri', 'rf_turn'
        ]
        # joint_command.velocity = [
        #     control_speeds[0], control_angles[0],
        #     control_speeds[1], control_angles[1],
        #     control_speeds[2], control_angles[2],
        #     control_speeds[3], control_angles[3]
        # ]
        joint_command.velocity = [
            control_speeds[0],  0.0,
            control_speeds[1],  0.0,
            control_speeds[2],  0.0,
            control_speeds[3],  0.0
        ]
        joint_command.position = [
            0.0,self.target_angles[0],
            0.0,self.target_angles[1],
            0.0,self.target_angles[2],
            0.0,self.target_angles[3],
        ]
        joint_command.header.stamp = self.get_clock().now().to_msg()
        self.joint_command_pub.publish(joint_command)
        self.get_logger().info(f"下发速度: {control_speeds}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelClosedLoopControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()