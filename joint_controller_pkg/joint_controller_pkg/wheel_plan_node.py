import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(JointState, 'target_states', 10)
        self.wheelbase = 0.58  # 前后轮轴距
        self.track_width = 0.44 # 左右轮轮距
        self.wheel_radius = 0.1  # 新增轮子半径参数，根据实际情况调整
    def limit_angle_to_90(self, angle):
        """将角度限制在 -π/2 到 π/2 范围内"""
        while angle > math.pi / 2:
            angle -= math.pi
        while angle < -math.pi / 2:
            angle += math.pi
        return angle
    def cmd_vel_callback(self, msg):
        try:
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['lb_turn', 'lb_dri', 'lf_turn', 'lf_dri', 'rb_turn', 'rb_dri', 'rf_turn', 'rf_dri']
            joint_state.position = [0.0] * 8
            joint_state.velocity = [0.0] * 8

            if linear_x == 0 and linear_y == 0 and angular_z != 0:
                self.in_place_turning(angular_z, joint_state)
            else:
                self.ackermann_steering(linear_x, linear_y, angular_z, joint_state)

            self.publisher_.publish(joint_state)
        except Exception as e:
            self.get_logger().error(f"Error processing cmd_vel message: {e}")

    def in_place_turning(self, angular_z, joint_state):
        try:
            # 计算线速度并转换为角速度
            wheel_speed_magnitude = abs(angular_z) * math.sqrt((self.wheelbase / 2)**2 + (self.track_width / 2)**2)
            wheel_speed_magnitude /= self.wheel_radius  # 转换为角速度

            direction = 1 if angular_z > 0 else -1
            left_wheel_speed = direction * wheel_speed_magnitude
            right_wheel_speed = -direction * wheel_speed_magnitude

            # 设置驱动轮角速度
            joint_state.velocity[1] = -left_wheel_speed  # 左后轮
            joint_state.velocity[3] = -left_wheel_speed  # 左前轮
            joint_state.velocity[5] = -right_wheel_speed  # 右后轮
            joint_state.velocity[7] = -right_wheel_speed  # 右前轮

            # 计算转向角度
            theta = math.atan2(self.track_width, self.wheelbase)
            # 限制角度在-90°到90°之间
            if theta > math.pi / 2:
                theta -= math.pi
            elif theta < -math.pi / 2:
                theta += math.pi

            joint_state.position[0] = theta   # 左后轮右转
            joint_state.position[2] = -theta  # 左前轮左转
            joint_state.position[4] = -theta  # 右后轮左转
            joint_state.position[6] = theta   # 右前轮右转

        except Exception as e:
            self.get_logger().error(f"Error in in_place_turning: {e}")

    def ackermann_steering(self, linear_x, linear_y, angular_z, joint_state):
        try:
            if angular_z != 0:
                turning_radius = linear_x / angular_z
            else:
                turning_radius = float('inf')

            if turning_radius != float('inf'):
                # 计算前轮转向角度
                theta1 = math.atan2(self.wheelbase, turning_radius - self.track_width / 2)
                theta2 = math.atan2(self.wheelbase, turning_radius + self.track_width / 2)
                
                
                joint_state.position[2] = self.limit_angle_to_90(theta1)  # 左前轮
                joint_state.position[6] = self.limit_angle_to_90(theta2)  # 右前轮
                # joint_state.position[0] = -theta1 # 左后轮
                # joint_state.position[4] = -theta2 # 右后轮
                speed = math.sqrt(linear_x**2 + linear_y**2) 
                speed_direction = 1 if linear_x >= 0 else -1
                speed = speed/ self.wheel_radius * speed_direction# 转换为角速度
                wheel_speeds = [speed] * 4  # 简化处理，实际应根据转弯半径调整
                
                for i in range(1, 8, 2):
                    joint_state.velocity[i] = -wheel_speeds[(i-1)//2]
            else:
                # 直线行驶，计算角速度
                speed_direction = 1 if linear_x >= 0 else -1
                speed = math.sqrt(linear_x**2 + linear_y**2) / self.wheel_radius* speed_direction
                for i in range(1, 8, 2):
                    joint_state.velocity[i] = -speed
                # 转向角归零
                for i in [0,2,4,6]:
                    joint_state.position[i] = 0.0

        except Exception as e:
            self.get_logger().error(f"Error in ackermann_steering: {e}")


def main(args=None):
    rclpy.init(args=args)
    wheel_controller = WheelController()
    rclpy.spin(wheel_controller)
    wheel_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()