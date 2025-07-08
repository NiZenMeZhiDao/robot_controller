import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Vector3, Twist
import numpy as np
import os
import onnxruntime as ort
import math

def quat_inv(q: np.ndarray) -> np.ndarray:
    """计算四元数的逆：q^{-1} = q^* / ||q||^2"""
    norm_sq = np.dot(q, q)
    if norm_sq < 1e-12:
        raise ValueError("零四元数无法求逆。")
    return np.array([q[0], -q[1], -q[2], -q[3]]) / norm_sq

def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """用四元数旋转三维向量：v' = q * v * q^{-1}"""
    w, x, y, z = q / np.linalg.norm(q)  # 归一化并解包
    # 预计算常用项，避免重复
    x2, y2, z2 = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    xw, yw, zw = x*w, y*w, z*w

    rot_matrix = np.array([
        [1 - 2*(y2 + z2), 2*(xy + zw),     2*(xz - yw)],
        [2*(xy - zw),     1 - 2*(x2 + z2), 2*(yz + xw)],
        [2*(xz + yw),     2*(yz - xw),     1 - 2*(x2 + y2)]
    ])
    return rot_matrix @ v

class ModelController:
    """自定义模型控制器 - 专注于 ONNX 推理"""
    OBS_DIM = 36  # 更新为36维观测 (3+3+3+9+9+9)
    ACTION_DIM = 9
    NUM_JOINTS_FOR_OBS = 9  # 仅处理9个关节
    ACTION_CLIP_RANGE = 1.22
    ACTION_SCALE_FACTOR = 0.25

    def __init__(self, model_path: str, ros_node: Node = None):
        self.ros_node = ros_node

        self.session = self._load_onnx_model(model_path)

        # 初始指令设置为零
        self.command = np.array([2.0, 0.0, 0.0], dtype=np.float32)
        self.last_action = np.zeros(self.ACTION_DIM, dtype=np.float32)

        if self.ros_node:
            self.ros_node.get_logger().info("ModelController 初始化完成。")

    def _load_onnx_model(self, model_path: str) -> ort.InferenceSession:
        """加载 ONNX 模型。"""
        if not os.path.exists(model_path):
            error_msg = f"ONNX 模型未找到: {model_path}"
            if self.ros_node:
                self.ros_node.get_logger().error(error_msg)
            raise FileNotFoundError(error_msg)

        providers = ['CPUExecutionProvider']
        if ort.get_device() == 'GPU':
            if self.ros_node:
                self.ros_node.get_logger().info("CUDA 可用，尝试使用 CUDAExecutionProvider。")
            providers.insert(0, 'CUDAExecutionProvider')

        session_options = ort.SessionOptions()
        try:
            session = ort.InferenceSession(model_path, sess_options=session_options, providers=providers)
            if self.ros_node:
                self.ros_node.get_logger().info(f"ONNX 模型加载成功! 输入: {session.get_inputs()[0].name}")
            return session
        except Exception as e:
            error_msg = f"加载 ONNX 模型失败: {e}"
            if self.ros_node:
                self.ros_node.get_logger().error(error_msg)
            raise RuntimeError(error_msg)

    def set_command(self, lin_vel_x: float, lin_vel_y: float, ang_vel_z: float):
        """设置机器人期望的速度指令。"""
        self.command = np.array([lin_vel_x, lin_vel_y, ang_vel_z], dtype=np.float32)
        if self.ros_node and self.ros_node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.ros_node.get_logger().debug(f"指令更新为: {self.command}")

    def forward(self, state_data: dict) -> np.ndarray:
        """
        控制器前向计算。
        Args:
            state_data (dict): 包含 'base_rotation', 'base_angular_velocity',
                               'joint_positions', 'joint_velocities' 作为 numpy 数组。
        Returns:
            np.ndarray: 模型直接输出的 9 元素动作数组 (已裁剪并缩放)。
        """
        obs = self._collect_robot_state(state_data)
        action_raw = self._model_inference_onnx(obs)

        self.last_action = action_raw

        clipped_scaled_action = action_raw * self.ACTION_SCALE_FACTOR

        if self.ros_node and self.ros_node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.ros_node.get_logger().debug(f"原始动作 (9 元素): {np.round(action_raw, 4)}")
            self.ros_node.get_logger().debug(f"已裁剪并缩放动作 (9 元素): {np.round(clipped_scaled_action, 4)}")

        return clipped_scaled_action

    def _collect_robot_state(self, state_data: dict) -> np.ndarray:
        """
        收集并处理机器人状态数据，生成 ONNX 模型所需的观测向量。
        仅使用前9个关节的数据。
        """
        base_rot = state_data["base_rotation"]
        base_ang_vel = state_data["base_angular_velocity"]
        joint_pos = state_data["joint_positions"]
        joint_vel = state_data["joint_velocities"]
        
        # 获取关节数据前9个
        joint_pos = joint_pos[:self.NUM_JOINTS_FOR_OBS]
        joint_vel = joint_vel[:self.NUM_JOINTS_FOR_OBS]

        # 重力处理
        unit_gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        projected_unit_gravity = quat_rotate(quat_inv(base_rot), unit_gravity_world)

        # 初始化36维观测向量
        obs = np.zeros(self.OBS_DIM, dtype=np.float32)

        # 填充观测向量
        obs[0:3] = base_ang_vel
        obs[3:6] = projected_unit_gravity
        obs[6:9] = self.command
        obs[9:18] = joint_pos
        obs[18:27] = joint_vel
        obs[27:36] = self.last_action

        if self.ros_node and self.ros_node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.INFO:
            self.ros_node.get_logger().info("\n--- 详细观测值输出 ---")
            self.ros_node.get_logger().info(f"基座角速度 (0:3): {np.round(obs[0:3], 4)}")
            self.ros_node.get_logger().info(f"投影单位重力 (3:6): {np.round(obs[3:6], 4)}")
            self.ros_node.get_logger().info(f"速度指令 (6:9): {np.round(obs[6:9], 4)}")
            self.ros_node.get_logger().info(f"关节位置 (9:18): {np.round(obs[9:18], 4)}")
            self.ros_node.get_logger().info(f"关节速度 (18:27): {np.round(obs[18:27], 4)}")
            self.ros_node.get_logger().info(f"上次动作 (27:36): {np.round(obs[27:36], 4)}")
            self.ros_node.get_logger().info("-----------------------------------\n")

        return obs

    def _model_inference_onnx(self, state: np.ndarray) -> np.ndarray:
        """使用 ONNX 模型执行推理"""
        obs_tensor = np.expand_dims(state, axis=0)
        input_name = self.session.get_inputs()[0].name
        output_name = self.session.get_outputs()[0].name
        outputs = self.session.run([output_name], {input_name: obs_tensor})
        return outputs[0][0]


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        self._latest_angular_velocity: np.ndarray = np.zeros(3, dtype=np.float32)
        self._valid_joint_indices = None  # 用于存储有效的关节索引映射

        qos_profile_sensor_data = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        qos_profile_commands = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.create_subscription(JointState, 'joint_state', self._joint_state_callback, qos_profile_sensor_data)
        self.create_subscription(Quaternion, 'orient', self._orient_callback, qos_profile_sensor_data)
        self.create_subscription(Vector3, 'angvel', self._angvel_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, qos_profile_commands)

        self.action_pub = self.create_publisher(JointState, 'action', qos_profile_commands)

        self._latest_joint_state: JointState = None
        self._latest_orientation: Quaternion = None

        # 定义9个需要控制的关节名称
        self.controlled_joint_names = [
            "front_waist_fl_i_up", "front_waist_fl_o_up", "front_waist_fr_i_up",
            "front_waist_fr_o_up", "front_waist_waist",
            "back_waist_hl_i_up", "back_waist_hl_o_up", "back_waist_hr_i_up",
            "back_waist_hr_o_up"
        ]

        onnx_model_path = "/home/ares/ros2_ws/src/my_robot_controller/policy.onnx"
        self.model_controller = ModelController(
            model_path=onnx_model_path,
            ros_node=self
        )

        self.create_timer(0.02, self._controller_timer_callback)

        self.get_logger().info('RobotControllerNode 启动成功。')
        self.get_logger().info('订阅话题: joint_state, orient, angvel, cmd_vel')
        self.get_logger().info('发布话题: action (仅 9 个关节)')

    def _joint_state_callback(self, msg: JointState):
        """更新关节状态，并构建关节索引映射"""
        self._latest_joint_state = msg
        
        # 首次接收时构建索引映射
        if self._valid_joint_indices is None:
            self._valid_joint_indices = []
            for joint_name in self.controlled_joint_names:
                try:
                    idx = msg.name.index(joint_name)
                    self._valid_joint_indices.append(idx)
                except ValueError:
                    self.get_logger().error(f"未找到关节: {joint_name}")
                    # 如果找不到所有目标关节，则无法继续
                    if len(self._valid_joint_indices) < len(self.controlled_joint_names):
                        self.get_logger().fatal("无法找到所有目标关节，退出!")
                        raise RuntimeError("缺失目标关节")

    def _orient_callback(self, msg: Quaternion):
        self._latest_orientation = msg

    def _angvel_callback(self, msg: Vector3):
        """
        订阅 angvel 话题的回调函数，更新基座角速度。
        """
        self._latest_angular_velocity = np.array([
            math.radians(msg.x),
            math.radians(msg.y),
            math.radians(msg.z)
        ], dtype=np.float32)

    def _cmd_vel_callback(self, msg: Twist):
        """
        订阅 cmd_vel 话题的回调函数，更新机器人速度指令。
        """
        lin_vel_x = float(msg.linear.x)
        lin_vel_y = float(msg.linear.y)
        ang_vel_z = float(msg.angular.z)
        self.model_controller.set_command(lin_vel_x, lin_vel_y, ang_vel_z)
        self.get_logger().debug(f"接收到 cmd_vel 指令: X={lin_vel_x:.2f}, Y={lin_vel_y:.2f}, Z={ang_vel_z:.2f}")

    def _controller_timer_callback(self):
        """定时器回调函数，用于收集数据、执行推理并发布动作。"""
        if (self._latest_joint_state is None or 
            self._latest_orientation is None or 
            self._latest_angular_velocity is None or 
            self._valid_joint_indices is None):
            self.get_logger().warn('等待所有传感器数据和关节映射，然后运行控制器...')
            return

        # 提取9个目标关节的数据
        joint_positions = np.zeros(9, dtype=np.float32)
        joint_velocities = np.zeros(9, dtype=np.float32)
        
        for i, idx in enumerate(self._valid_joint_indices):
            if idx < len(self._latest_joint_state.position):
                joint_positions[i] = self._latest_joint_state.position[idx]
            if idx < len(self._latest_joint_state.velocity):
                joint_velocities[i] = self._latest_joint_state.velocity[idx]

        state_data = {
            "base_rotation": np.array([
                self._latest_orientation.w,
                self._latest_orientation.x,
                self._latest_orientation.y,
                self._latest_orientation.z
            ], dtype=np.float32),
            "base_angular_velocity": self._latest_angular_velocity,
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities
        }

        action_to_publish = self.model_controller.forward(state_data)

        action_msg = JointState()
        action_msg.header.stamp = self.get_clock().now().to_msg()
        action_msg.name = self.controlled_joint_names
        action_msg.position = action_to_publish.tolist()

        self.action_pub.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
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