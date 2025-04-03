from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动 wheel_control_node 节点
        Node(
            package='joint_controller_pkg',
            executable='wheel_control_node',
            output='screen'
        ),
        # 启动 wheel_closed_loop_control_node 节点
        Node(
            package='joint_controller_pkg',
            executable='wheel_plan_node',
            output='screen'
        )
    ])
    