import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Joy
from builtin_interfaces.msg import Duration


class JoystickArmController(Node):
    def __init__(self):
        super().__init__('joystick_arm_controller')

        # 로봇팔 조인트 이름
        self.joint_names = [
            'base',
            'shoulder',
            'elbow',
            'wrist_pitch',
            'wrist_roll',
            'gripper_movable_joint'
        ]
        self.current_positions = [0.0] * len(self.joint_names)

        # 퍼블리셔: 로봇팔 trajectory
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # 퍼블리셔: joint_states (시각화를 위한)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # 구독: WebRTC → 브라우저 → ROS → /joystick_data (Joy 메시지)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joystick_data',
            self.joy_callback,
            10
        )

        # 조작 설정
        self.deadzone = 0.3
        self.scale = 0.01  # ← 민감도 (0.003~0.01 범위 추천)

        # 그리퍼 상태
        self.gripper_open = 1.2
        self.gripper_closed = 0.2

        # 주기적으로 퍼블리시 (0.15초 간격)
        self.timer = self.create_timer(0.15, self.publish_all)

    def joy_callback(self, msg: Joy):
        axes = list(msg.axes) + [0.0] * max(0, 5 - len(msg.axes))
        buttons = list(msg.buttons) + [0] * max(0, 6 - len(msg.buttons))

        dx = axes[0] * self.scale if abs(axes[0]) > self.deadzone else 0.0
        dy = axes[1] * self.scale if abs(axes[1]) > self.deadzone else 0.0
        dz = axes[3] * self.scale if abs(axes[3]) > self.deadzone else 0.0
        drx = axes[2] * self.scale if abs(axes[2]) > self.deadzone else 0.0
        dry = axes[4] * self.scale if abs(axes[4]) > self.deadzone else 0.0

        self.current_positions[0] += dx    # base
        self.current_positions[1] += dy    # shoulder
        self.current_positions[2] += -dz   # elbow
        self.current_positions[3] += dry   # wrist_pitch
        self.current_positions[4] += drx   # wrist_roll

        if buttons[4] == 1:  # LB
            self.get_logger().info("Gripper CLOSE")
            self.current_positions[5] = self.gripper_closed
        elif buttons[5] == 1:  # RB
            self.get_logger().info("Gripper OPEN")
            self.current_positions[5] = self.gripper_open

    def publish_all(self):
        self.publish_trajectory()
        self.publish_joint_states()

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        point.velocities = [0.02] * len(self.joint_names)
        point.accelerations = [0.01] * len(self.joint_names)
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points.append(point)
        self.trajectory_pub.publish(msg)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickArmController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()