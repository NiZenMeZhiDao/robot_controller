import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取包的共享目录
    my_robot_controller_share_dir = get_package_share_directory('my_robot_controller')

    # ONNX 模型路径
    # 确保这个路径与你的实际模型路径匹配
    onnx_model_path = os.path.join(my_robot_controller_share_dir, 'policy.onnx')
    # 或者如果模型在你的工作区 src/my_robot_controller/ 目录下
    # onnx_model_path = "/home/ares/ros2_ws/src/my_robot_controller/policy.onnx" # 使用绝对路径，确保正确

    # 声明 launch 参数
    # 这些参数可以让你在启动时方便地配置节点
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='USB serial port for robot communication'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for USB serial communication'
    )
    read_frequency_hz_arg = DeclareLaunchArgument(
        'read_frequency_hz',
        default_value='100.0',
        description='Frequency for reading sensor data from hardware (Hz)'
    )
    write_frequency_hz_arg = DeclareLaunchArgument(
        'write_frequency_hz',
        default_value='50.0',
        description='Frequency for sending commands to hardware (Hz)'
    )

    # RobotControllerNode 节点
    robot_controller_node = Node(
        package='my_robot_controller',
        executable='robot_controller_node', # 注意：这里应该是你的 Python 脚本的入口点
        name='robot_controller_node',
        output='screen',
        # parameters=[{'onnx_model_path': onnx_model_path}], # 如果你的 RobotControllerNode 支持参数设置模型路径，可以这样传递
                                                            # 但根据你的代码，它直接硬编码了路径，所以这里不需要
        emulate_tty=True, # 确保日志输出彩色
        arguments=['--ros-args', '--log-level', 'robot_controller_node:=info'] # 调整日志级别，方便调试
    )

    # UsbCommunicatorNode 节点
    usb_communicator_node = Node(
        package='my_robot_controller',
        executable='usb_communicator_node', # 注意：这里应该是你的 Python 脚本的入口点
        name='usb_communicator_node',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baud_rate': LaunchConfiguration('baud_rate')},
            {'read_frequency_hz': LaunchConfiguration('read_frequency_hz')},
            {'write_frequency_hz': LaunchConfiguration('write_frequency_hz')},
            # 传递模拟的关节名称，确保与 RobotControllerNode 使用的名称一致
            {'simulated_joint_names': [
                "front_waist_fl_i_up", "front_waist_fl_o_up", "front_waist_fr_i_up",
                "front_waist_fr_o_up", "front_waist_waist",
                "back_waist_hl_i_up", "back_waist_hl_o_up", "back_waist_hr_i_up",
                "back_waist_hr_o_up"
            ]}
        ],
        emulate_tty=True, # 确保日志输出彩色
        arguments=['--ros-args', '--log-level', 'usb_communicator_node:=info'] # 调整日志级别
    )

    return LaunchDescription([
        port_arg,
        baud_rate_arg,
        read_frequency_hz_arg,
        write_frequency_hz_arg,
        robot_controller_node,
        usb_communicator_node
    ])