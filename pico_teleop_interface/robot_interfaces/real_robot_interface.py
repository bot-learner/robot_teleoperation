"""真实机器人通信层必须提供的最小接口。

遥操作算法只生成关节目标，不应直接依赖某一家机器人的 SDK。具体机器人需要
实现本文件中的接口，把“连接、反馈、下发和安全停止”适配到厂商通信协议。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence


class RealRobotInterface(ABC):
    """真实机器人通信适配器。

    角度单位统一为弧度。``joint_names`` 的顺序就是对应角度数组的顺序，具体
    实现不得自行重排。连接和关闭操作应当幂等，避免异常清理时重复调用造成故障。
    """

    @abstractmethod
    def connect(self) -> None:
        """连接机器人并取得控制权。

        实现必须检查机器人是否在线、是否允许外部控制以及是否存在故障；条件
        不满足时应抛出异常，不能在未知状态下继续发送命令。
        """

    @abstractmethod
    def read_joint_positions(self, joint_names: Sequence[str]) -> tuple[float, ...]:
        """按 ``joint_names`` 的顺序返回当前关节角度，单位为弧度。

        返回数量必须与关节名称数量一致。通信超时、反馈缺失或数值无效时应抛出
        异常，不能返回旧缓存冒充当前反馈。
        """

    @abstractmethod
    def send_joint_positions(
        self, joint_names: Sequence[str], positions: Sequence[float]
    ) -> None:
        """向机器人发送一帧关节位置目标，单位为弧度。

        实现必须拒绝数量不一致、非有限值、机器人故障或未取得控制权的命令。
        机器人侧还应设置命令超时保护，避免 PC 断连后长期执行最后一帧命令。
        """

    @abstractmethod
    def stop(self) -> None:
        """立即停止更新目标，并让机器人进入厂商规定的安全保持状态。"""

    @abstractmethod
    def close(self) -> None:
        """释放机器人控制权和通信资源；未连接或重复调用时也应安全。"""
