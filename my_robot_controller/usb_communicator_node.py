import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np
import threading
import time
import math
# 如果你计划进行真实的USB通信，需要安装pyserial: pip install pyserial
# import serial

class UsbCommunicatorNode(Node):
    def __init__(self):
        super().__init__('usb_communicator_node')

        self.get_logger().info("USB Communicator Node 正在启动...")

        # --- 配置参数 ---
        # 这些参数可以通过 launch 文件或命令行重载
        self.declare_parameter('port', '/dev/ttyUSB0')  # USB 端口名
        self.declare_parameter('baud_rate', 115200)      # 串口波特率
        self.declare_parameter('read_frequency_hz', 100.0) # 从硬件读取传感器数据的频率 (Hz)
        self.declare_parameter('write_frequency_hz', 50.0) # 向硬件发送命令的频率 (Hz)
        self.declare_parameter('simulated_joint_names', [
            "front_waist_fl_i_up", "front_waist_fl_o_up", "front_waist_fr_i_up",
            "front_waist_fr_o_up", "front_waist_waist",
            "back_waist_hl_i_up", "back_waist_hl_o_up", "back_waist_hr_i_up",
            "back_waist_hr_o_up"
        ]) # 模拟的关节名称

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.read_frequency = self.get_parameter('read_frequency_hz').get_parameter_value().double_value
        self.write_frequency = self.get_parameter('write_frequency_hz').get_parameter_value().double_value
        self.simulated_joint_names = self.get_parameter('simulated_joint_names').get_parameter_value().string_array_value


        # --- ROS 2 发布器 ---
        # 发布器用于发布来自下位机的传感器数据
        self.joint_state_pub = self.create_publisher(JointState, 'joint_state', 10)
        self.orient_pub = self.create_publisher(Quaternion, 'orient', 10)
        self.angvel_pub = self.create_publisher(Vector3, 'angvel', 10)

        # --- ROS 2 订阅器 ---
        # 订阅器用于接收 RobotControllerNode 发送的关节动作命令
        self.subscription = self.create_subscription(JointState, 'action', self._action_callback, 10)
        self.get_logger().info(f"订阅话题: action")

        # --- USB 通信 / 模拟变量 ---
        # 在真实的 USB 通信场景中，这里会初始化 pyserial 串口对象
        # self.serial_port = None
        self._last_received_actions = np.zeros(len(self.simulated_joint_names), dtype=np.float32)

        # 模拟状态变量 (用于发布)
        self._sim_joint_positions = np.zeros(len(self.simulated_joint_names), dtype=np.float32)
        self._sim_joint_velocities = np.zeros(len(self.simulated_joint_names), dtype=np.float32)
        self._sim_orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0) # 初始方向为单位四元数
        self._sim_angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)

        # 启动模拟数据发布线程
        self._publisher_thread = threading.Thread(target=self._publish_simulated_data_loop)
        self._publisher_thread.daemon = True # 设置为守护线程，主程序退出时自动终止
        self._publisher_thread.start()

        # 启动命令发送线程 (在真实情况下，这会是发送到串口的线程)
        self._command_thread = threading.Thread(target=self._send_commands_loop)
        self._command_thread.daemon = True
        self._command_thread.start()

        # --- 尝试建立串口连接 (仅在真实情况下) ---
        # self._connect_serial()

        self.get_logger().info(f"UsbCommunicatorNode 初始化完成。 模拟发布频率: {self.read_frequency} Hz, 模拟发送频率: {self.write_frequency} Hz")

    # def _connect_serial(self):
    #     """尝试连接到串口 (真实场景使用)"""
    #     try:
    #         self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=0.1)
    #         self.get_logger().info(f"成功连接到串口: {self.port} @ {self.baud_rate}")
    #     except serial.SerialException as e:
    #         self.get_logger().error(f"无法连接到串口 {self.port}: {e}")
    #         self.serial_port = None # 确保连接失败时为 None

    def _action_callback(self, msg: JointState):
        """
        接收 RobotControllerNode 发布的关节动作命令。
        在真实场景中，你会将这些命令打包并通过 USB 发送给下位机。
        """
        if len(msg.position) != len(self.simulated_joint_names):
            self.get_logger().warn(f"接收到的动作消息关节数量不匹配. 预期: {len(self.simulated_joint_names)}, 实际: {len(msg.position)}")
            return
            
        self._last_received_actions = np.array(msg.position, dtype=np.float32)
        # self.get_logger().info(f"接收到动作: {np.round(self._last_received_actions, 4)}") # 调试用

    def _send_commands_loop(self):
        """
        独立的线程循环，模拟将命令发送到下位机。
        在真实场景中，这里会执行串口写入操作。
        """
        rate = self.write_frequency
        if rate <= 0:
            self.get_logger().warn("命令发送频率设置为 0 或更低，命令发送循环将不运行。")
            return

        interval = 1.0 / rate
        self.get_logger().info(f"命令发送循环以 {rate} Hz 启动...")
        while rclpy.ok():
            start_time = time.time()
            
            # --- 真实场景: 串口写入 ---
            # if self.serial_port and self.serial_port.is_open:
            #     # 示例: 将动作数据转换为字节并发送
            #     # command_bytes = self._encode_commands(self._last_received_actions)
            #     # self.serial_port.write(command_bytes)
            #     pass # 占位符，模拟写入
            # else:
            #     self.get_logger().warn("串口未连接或已关闭，无法发送命令。")

            # --- 模拟场景: 更新内部模拟关节状态 (可选，仅用于演示循环更新) ---
            # 这里简单地让模拟关节位置朝着接收到的动作值变化
            delta_t = interval # 假设一个简单的积分
            self._sim_joint_positions += self._last_received_actions * 0.01 * delta_t # 乘以小系数模拟缓慢变化

            elapsed_time = time.time() - start_time
            sleep_time = interval - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            # else:
                # self.get_logger().warn(f"命令发送循环超时！ 耗时: {elapsed_time:.4f}s, 目标间隔: {interval:.4f}s")


    def _publish_simulated_data_loop(self):
        """
        独立的线程循环，模拟从下位机读取传感器数据并发布。
        在真实场景中，这里会执行串口读取操作并解析数据。
        """
        rate = self.read_frequency
        if rate <= 0:
            self.get_logger().warn("读取频率设置为 0 或更低，模拟数据发布循环将不运行。")
            return

        interval = 1.0 / rate
        self.get_logger().info(f"模拟传感器数据发布循环以 {rate} Hz 启动...")
        while rclpy.ok():
            start_time = time.time()

            # --- 真实场景: 串口读取与数据解析 ---
            # if self.serial_port and self.serial_port.is_open:
            #     # 示例: 从串口读取原始字节数据
            #     # raw_data = self.serial_port.read_until(b'\n') # 读取直到换行符
            #     # if raw_data:
            #     #     parsed_data = self._parse_serial_data(raw_data) # 解析数据
            #     #     # 更新 _sim_joint_positions, _sim_orientation, _sim_angular_velocity 等
            #     # else:
            #     #     pass # 没有数据可读
            #     pass # 占位符，模拟读取
            # else:
            #     # self.get_logger().warn_throttle(5.0, "串口未连接或已关闭，无法读取传感器数据。")
            #     pass # 避免频繁警告

            # --- 模拟场景: 生成并发布假数据 ---
            # 为了让 RobotControllerNode 能够运行，我们生成一些动态的模拟数据
            current_time_sec = self.get_clock().now().nanoseconds / 1e9
            
            # 模拟关节位置和速度 (简单正弦波变化)
            num_joints = len(self.simulated_joint_names)
            for i in range(num_joints):
                # 假设关节在 -pi/4 到 pi/4 之间摆动
                self._sim_joint_positions[i] = 0.5 * math.sin(current_time_sec * (0.5 + i * 0.1))
                self._sim_joint_velocities[i] = 0.5 * (0.5 + i * 0.1) * math.cos(current_time_sec * (0.5 + i * 0.1))

            # 模拟方向 (简单绕Z轴旋转)
            angle_z = 0.1 * current_time_sec # 模拟缓慢旋转
            q_w = math.cos(angle_z / 2)
            q_z = math.sin(angle_z / 2)
            self._sim_orientation = Quaternion(w=q_w, x=0.0, y=0.0, z=q_z)

            # 模拟角速度 (与方向变化对应)
            self._sim_angular_velocity = Vector3(x=0.0, y=0.0, z=0.1) # 对应 Z 轴旋转速度

            # 发布关节状态
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.simulated_joint_names
            joint_state_msg.position = self._sim_joint_positions.tolist()
            joint_state_msg.velocity = self._sim_joint_velocities.tolist()
            self.joint_state_pub.publish(joint_state_msg)
            # self.get_logger().debug(f"发布关节状态: {np.round(self._sim_joint_positions, 2)}") # 调试用

            # 发布方向
            orient_msg = self._sim_orientation
            orient_msg.header.stamp = self.get_clock().now().to_msg() # Quaternion 消息没有 header，需要手动设置
            self.orient_pub.publish(orient_msg)

            # 发布角速度
            angvel_msg = self._sim_angular_velocity
            angvel_msg.header.stamp = self.get_clock().now().to_msg() # Vector3 消息没有 header，需要手动设置
            self.angvel_pub.publish(angvel_msg)

            elapsed_time = time.time() - start_time
            sleep_time = interval - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            # else:
                # self.get_logger().warn(f"传感器数据发布循环超时！ 耗时: {elapsed_time:.4f}s, 目标间隔: {interval:.4f}s")


    # def _encode_commands(self, commands: np.ndarray) -> bytes:
    #     """
    #     将 Numpy 数组的命令编码为适合串口发送的字节串。
    #     这需要根据你下位机的通信协议来实现。
    #     例如: struct.pack, JSON, 自定义协议等。
    #     """
    #     # 示例: 简单地将浮点数转换为字符串，然后用逗号分隔，最后编码为字节
    #     # return (",".join(map(str, commands.tolist())) + "\n").encode('utf-8')
    #     pass

    # def _parse_serial_data(self, raw_data: bytes) -> dict:
    #     """
    #     解析从串口读取的原始字节数据，转换为传感器数据。
    #     这同样需要根据下位机的通信协议来实现。
    #     例如: struct.unpack, JSON 解析等。
    #     返回一个字典，包含 'joint_positions', 'joint_velocities', 'orientation', 'angular_velocity'
    #     """
    #     # 示例: 将字节数据解码为字符串，然后分割解析
    #     # decoded_data = raw_data.decode('utf-8').strip()
    #     # values = [float(x) for x in decoded_data.split(',')]
    #     # return {
    #     #     "joint_positions": np.array(values[0:9]),
    #     #     "joint_velocities": np.array(values[9:18]),
    #     #     "orientation": Quaternion(w=values[18], x=values[19], y=values[20], z=values[21]),
    #     #     "angular_velocity": Vector3(x=values[22], y=values[23], z=values[24])
    #     # }
    #     pass

    def destroy_node(self):
        self.get_logger().info("UsbCommunicatorNode 正在关闭...")
        # if self.serial_port and self.serial_port.is_open:
        #     self.serial_port.close()
        #     self.get_logger().info("串口已关闭。")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UsbCommunicatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点通过 Ctrl+C 干净地停止。')
    except Exception as e:
        node.get_logger().error(f"发生意外错误: {e}")
    finally:
        node.get_logger().info('正在关闭。')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()