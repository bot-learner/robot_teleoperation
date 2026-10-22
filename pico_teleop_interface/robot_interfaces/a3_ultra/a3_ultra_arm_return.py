"""A3 Ultra 双臂避障回到自然下垂姿态。

运行前必须确认急停可用、官方 motion_control 已停止，且 HDU/MDU 桥接正常。
运行此文件会直接向真实机器人发送指令。
"""

from pathlib import Path
import sys
import time

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from pico_teleop_interface.robot_interfaces.a3_ultra.a3_ultra_real_robot import (
    A3UltraRealRobot,
)


def main() -> None:
    dds_profile = Path(__file__).resolve().parents[3] / "usage/ros_dds_configuration.xml"
    robot = A3UltraRealRobot(dds_profile=dds_profile)
    try:
        robot.connect()
        print("已连接真机，正在规划双臂自然下垂路径……")
        robot.move_to_natural_pose()
        while robot.is_moving:
            time.sleep(0.1)
        print("已到达自然下垂姿态，正在保持。按 Ctrl+C 退出。")
        while True:
            time.sleep(1.)
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()


if __name__ == "__main__":
    main()
