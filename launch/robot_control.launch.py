import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取你的 ROS 2 包的共享目录
    my_robot_controller_pkg_dir = get_package_share_directory('my_robot_controller')

    # 定义机器人控制器节点
    robot_controller_node = Node(
        package='my_robot_controller',  # 替换为你的实际 ROS 2 包名
        executable='robot_controller_node.py', # 你的节点文件名
        name='robot_controller',
        output='screen', # 将节点输出显示在屏幕上
        # parameters=[{'param_name': 'param_value'}] # 如果你的节点有需要从launch文件传入的参数，可以在这里定义
    )

    # 定义 teleop_twist_keyboard 节点
    # 它默认发布到 /cmd_vel 话题
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e', # 这是一个可选的设置，它会在一个新的终端窗口中打开teleop_twist_keyboard
                              # 方便你直接输入指令。如果不需要新窗口，可以移除此行。
        # remappings=[('/cmd_vel', '/my_custom_cmd_vel_topic')] # 如果需要重映射cmd_vel话题，可以在这里设置
    )

    # 返回启动描述
    return LaunchDescription([
        robot_controller_node,
        teleop_keyboard_node
    ])
