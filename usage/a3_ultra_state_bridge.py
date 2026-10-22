#!/usr/bin/env python3

"""

[1] 在 PC 的项目根目录上传本文件到 HDU：
scp usage/a3_ultra_state_bridge.py \
  pico_teleop_interface/robot_interfaces/arm_diagnostic_log.py \
  agi@192.168.31.85:/agibot/data/home/agi/Desktop/

[2] 从 PC 登录 HDU：
ssh agi@192.168.31.85

[3] 在 HDU 中加载 low-level 消息环境并运行本文件：
source /opt/ros/jazzy/setup.bash
source /agibot/software/v0/entry/env/env.sh
# 使用固件自带协议包；本机 Python 为 3.11。
export PYTHONPATH=/opt/agibot/share/ros2_package/aima_protocol_ros2_package/lib/python3.11/site-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/opt/agibot/share/ros2_package/aima_protocol_ros2_package/lib:${LD_LIBRARY_PATH:-}
export AMENT_PREFIX_PATH=/opt/agibot/share/ros2_package/aima_protocol_ros2_package:${AMENT_PREFIX_PATH:-}
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=232
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/cfg/ros_dds_configuration.xml
python3 /agibot/data/home/agi/Desktop/a3_ultra_state_bridge.py

成功接收第一帧后会显示：
Waiting for /ros2/body_drive/arm_joint_state
Forwarding /ros2/body_drive/arm_joint_state -> /teleop/arm_joint_state
本文件转发状态、PC 标准指令及 MDU 心跳；底层控制转换由 MDU 上的 usage/mdu_arm_bridge 负责。
请保持该终端和程序持续运行。


"""

"""HDU 跨 Wi-Fi/内网转发 ROS 2 消息；不直接发布 HAL 控制命令。"""

import math
import time

try:
    from arm_diagnostic_log import ArmDiagnosticLog  # HDU：启动脚本上传到同一目录。
except ModuleNotFoundError:
    from pathlib import Path
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from pico_teleop_interface.robot_interfaces.arm_diagnostic_log import ArmDiagnosticLog

import rclpy
from joint_msgs.msg import JointState as LowLevelJointState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState as TeleopJointState
from std_msgs.msg import Bool


# 当前固件实际暴露的 ROS 2 反馈端点，与示例程序内部 topic 的命名不同。
ROBOT_STATE_TOPIC = "/ros2/body_drive/arm_joint_state"
TELEOP_STATE_TOPIC = "/teleop/arm_joint_state"

ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

class ArmStateBridge(Node):
    """反馈转换为标准消息，指令和就绪心跳仅在不同 ROS 2 话题间搬运。"""

    def __init__(self) -> None:
        super().__init__("a3_ultra_arm_state_bridge")
        self.diagnostics = ArmDiagnosticLog("hdu")
        self._last_command_at = None
        self._last_state_at = None
        teleop_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        body_drive_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.state_publisher = self.create_publisher(
            TeleopJointState, TELEOP_STATE_TOPIC, teleop_qos
        )
        self.state_subscription = self.create_subscription(
            LowLevelJointState,
            ROBOT_STATE_TOPIC,
            self._forward_state,
            body_drive_state_qos,
        )
        self._received_first_state = False
        # HDU 同时连接 Wi-Fi 和机器人内网，MDU 不直接暴露给 PC。
        self.command_publisher = self.create_publisher(
            TeleopJointState, "/teleop/mdu/arm_joint_command", teleop_qos
        )
        self.command_subscription = self.create_subscription(
            TeleopJointState, "/teleop/arm_joint_command", self._forward_command,
            teleop_qos,
        )
        self.ready_publisher = self.create_publisher(Bool, "/teleop/arm_bridge_ready", teleop_qos)
        self.ready_subscription = self.create_subscription(
            Bool, "/teleop/mdu/arm_bridge_ready", self._forward_ready, teleop_qos,
        )
        self.get_logger().info(f"Waiting for {ROBOT_STATE_TOPIC}")
        self.get_logger().info("Relaying ROS commands/readiness to/from MDU; HAL conversion runs on MDU.")

    def _forward_command(self, message):
        now = time.monotonic()
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        self.diagnostics.write("command_received", stamp_ns=stamp,
                               name_count=len(message.name), position_count=len(message.position),
                               names=[name[:128] for name in message.name[:14]],
                               position=list(message.position[:14]),
                               receive_gap_ms=(now - self._last_command_at) * 1000
                               if self._last_command_at is not None else None)
        self._last_command_at = now
        self.command_publisher.publish(message)
        self.diagnostics.write("command_forwarded", stamp_ns=stamp,
                               forward_ms=(time.monotonic() - now) * 1000)

    def _forward_ready(self, message):
        self.ready_publisher.publish(message)
        self.diagnostics.write("bridge_ready_forwarded", ready=bool(message.data))

    def _forward_state(self, message: LowLevelJointState) -> None:
        """把底层逐关节状态转换成 PC 使用的标准 JointState。"""

        by_name = {joint.name: joint for joint in message.joints}
        if len(message.joints) != len(ARM_JOINT_NAMES) or set(by_name) != set(
            ARM_JOINT_NAMES
        ):
            self.get_logger().error("Rejected arm state: invalid joint names")
            return
        output = TeleopJointState()
        output.header = message.header
        output.name = list(ARM_JOINT_NAMES)
        output.position = [by_name[name].position for name in ARM_JOINT_NAMES]
        output.velocity = [by_name[name].velocity for name in ARM_JOINT_NAMES]
        output.effort = [by_name[name].effort for name in ARM_JOINT_NAMES]
        if not all(math.isfinite(value) for value in output.position):
            return
        self.state_publisher.publish(output)
        now = time.monotonic()
        self.diagnostics.write("feedback_forwarded",
                               stamp_ns=output.header.stamp.sec * 10**9 + output.header.stamp.nanosec,
                               names=list(output.name), position=list(output.position),
                               velocity=list(output.velocity), effort=list(output.effort),
                               receive_gap_ms=(now - self._last_state_at) * 1000
                               if self._last_state_at is not None else None)
        self._last_state_at = now
        if not self._received_first_state:
            self._received_first_state = True
            self.get_logger().info(
                f"Forwarding {ROBOT_STATE_TOPIC} -> {TELEOP_STATE_TOPIC}"
            )


def main() -> None:
    rclpy.init()
    node = ArmStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.diagnostics.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
