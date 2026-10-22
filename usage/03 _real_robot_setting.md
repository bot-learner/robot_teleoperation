


只有运行了下面的这些步骤，才可以成功运行机器人上肢遥操作：

## PICO 双臂实机入口

按本文启动 HDU、MDU 桥后，关闭滑条 GUI 和其他双臂指令发送程序，在 PC 项目根目录运行：

```bash
conda activate a3_teleop
python pico_teleop_interface/robot_interfaces/a3_ultra/a3_ultra_interface.py
```

PICO 中启用 Head 和两个 Motion Tracker 的 Object Tracking，并开启 Send。
程序打开一个 MuJoCo 窗口，先规划避障路径，限速运动到约定的双臂默认姿态：
左臂 `[-1, 0.1, 0, 0.5, 0, 0, 0]`、右臂 `[-1, -0.1, 0, 0.5, 0, 0, 0]`（rad）。
归位过程中窗口显示真实角度；规划失败则退出，不会直接跳到目标。
实际归位完成后倒计时 6 秒；保持头朝前，双臂向前、肘部微弯。
倒计时结束后以归位后的真实双臂角度建立机器人参考位姿，以首帧有效 PICO 位姿建立人侧零点。
之后 PICO → 相对位姿映射 → IK → 最新目标缓存 → 100 Hz ROS 发送 → HDU → MDU → HAL。
每帧 IK 不运行离散路径规划；实机速度上限为 `REAL_ROBOT_TELEOP_SPEED_RAD_S=0.5 rad/s`。

MuJoCo 窗口显示真机反馈角度，是运动学镜像，不是 SONIC 输出或预测的 PD 轨迹。
PICO 无有效数据时保持最后已发送角度；IK 目标超过 `REAL_ROBOT_STREAM_TIMEOUT_SECONDS=0.2 s`
不更新也停止推进旧目标。实际机器人反馈和 MDU 心跳延迟只在监视窗标红；MDU
明确报告未就绪时仍会中止发送并报错。
关闭窗口或 Ctrl+C 会释放实机 ROS 节点，之后电机行为由 HAL/驱动决定。
IK 仍使用固定的身体碰撞模型，不能据此保证与实际腰腿姿态不同的真机无碰撞。

直接运行 `a3_ultra_interface.py` 即为实机模式，不需要参数。
纯仿真仍可通过 `A3UltraInterface().run_sim()` 启动；本次修改无需改动或重编译 MDU 桥。



## [1] 连接到机器人的 ssh


机器人需要连上 wifi （和 PC 同一个 wifi）

手柄侧显示机器人的外部 ip ， 比如 192.168.31.85

此时可以通过 ssh 连接机器人，注意此时连接的是机器人内部的 HDU

```bash
ssh agi@192.168.31.85
```

pwd 是 1





## [2] 加载底层消息环境

当前实机代码使用底层 `body_drive` 接口，不再使用官方 `MOTION` Action。
HDU 固件已自带 `joint_msgs`，目录为：

```text
/opt/agibot/share/ros2_package/aima_protocol_ros2_package
```

启动脚本自动加载对应 Python 版本的消息目录和动态库，无需先部署整套策略包。
`/agibot/a3_deploy/setup_ros2_msgs.bash` 是自定义部署包的可选环境文件，不是固件必需文件。

现场反馈为 `/ros2/body_drive/arm_joint_state`，类型
`joint_msgs/msg/JointState`，QoS 为 `BEST_EFFORT`。HAL 的双臂指令仅接收 Iceoryx，
因此不能在 HDU 上向同名 ROS 2 话题直接发布底层消息。MDU 控制桥通过本机
Iceoryx 接入，PC 通过就绪心跳与真实关节反馈分别检查通信准备和执行跟踪。

## [3] 停止官方 motion_control

在 ssh 中运行这个命令进入 MDU 内部：


```bash
ssh agi@10.42.10.12
```

pwd 也是 1

确认机器人可靠吊装、急停可用之后，在 MDU 停止官方运控模块：

```bash
aimcli em stop motion_control
```

这会停止整机官方运控，而不只是停止动作播放器。此后必须由 low-level 控制器
负责必要的身体控制，不能只依靠一条偶发的双臂命令。

关闭后可以通过 `exit` or  `ssh agi@10.42.10.10` 切换回 HDU





## [4] 配置 PC 与机器人的 ROS 2 通信

HDU 上的 DDS 配置已复制到 `usage/ros_dds_configuration.xml`。

PC 通过机器人背部调试网口连接后，将 PC 有线网卡设置为一个未被占用的
`10.42.10.x` 地址。然后把 XML 中的：

```xml
<address>10.42.10.10</address>
```

改为 **PC 有线网卡自己的 IP**，不要与 HDU、MDU 或其他设备重复。

运行实机控制时传入：

```bash
--dds-profile usage/ros_dds_configuration.xml
```





## [5] 网线连接机器人，设置 PC 端的 IP 地址






## [-1] 在机器人内部查看一帧数据

运行下面的命令来设置一些必要的环境变量：


```bash
# 在机器人 HDU 侧执行：
source /opt/ros/jazzy/setup.bash
source /agibot/software/v0/entry/env/env.sh

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=232
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/cfg/ros_dds_configuration.xml
```

```bash
curl -s \
  -H 'content-type:application/json' \
  -X POST \
  'http://10.42.10.12:56322/rpc/aimdk.protocol.MotionControlActionService/GetAction' \
  -d '{}'
```

输出类似：

```json
{"header":{"code":"0","msg":"called successfully.","timestamp":{"seconds":"1792465900","nanos":393488896,"ms_since_epoch":"0"},"trace_id":"module_mc","domin":""},"info":{"current_action":"MotionControlAction_MOTION","ext_action":"","status":"MotionControlActionStatus_RUNNING","stage":{"seq":0,"description":"ActionStage::default"},"avatar_mode":"MotionControlAvatarMode_DEFAULT"}}agi@ros2 topic list --no-daemon | grep -E 'motion/control|arm_joint'ol|arm_joint'
/motion/control/arm_collision_info/pb_3Aaimdk_2Eprotocol_2EMotionControlArmCollisionInfoChannel
/motion/control/arm_future_collision_state
/motion/control/arm_joint_command
/motion/control/arm_joint_future
/motion/control/arm_joint_state
/motion/control/hand_joint_command
/motion/control/hand_joint_state
/motion/control/leg_joint_command
/motion/control/leg_joint_state
/motion/control/locomotion_velocity/pb_3Aaimdk_2Eprotocol_2EMotionControlLocomotionVelocityChannel
/motion/control/move_waist/pb_3Aaimdk_2Eprotocol_2EMotionControlMoveWaistChannel
/motion/control/neck_joint_command
/motion/control/neck_joint_state
/motion/control/pnc/arm_joint_command
/motion/control/waist_joint_command
/motion/control/waist_joint_state
/motion/control/walking_future_collision_state
/ros2/body_drive/arm_joint_state



```bash
ros2 topic echo \
  /motion/control/arm_joint_state \
  sensor_msgs/msg/JointState \
  --qos-reliability best_effort \
  --qos-history keep_last \
  --qos-depth 10 \
  --once
```



header:
  stamp:
    sec: 1792465927
    nanosec: 542621054
  frame_id: module_mc
name:
- left_shoulder_pitch_joint
- left_shoulder_roll_joint
- left_shoulder_yaw_joint
- left_elbow_joint
- left_wrist_roll_joint
- left_wrist_pitch_joint
- left_wrist_yaw_joint
- right_shoulder_pitch_joint
- right_shoulder_roll_joint
- right_shoulder_yaw_joint
- right_elbow_joint
- right_wrist_roll_joint
- right_wrist_pitch_joint
- right_wrist_yaw_joint
position:
- 0.291402033843994
- 0.10650776123046857
- -0.0011972782897951006
- 0.8243078616333008
- -0.0012441279602048994
- 0.011654491577148374
- -5.781188964837369e-05
- 0.2947594442749022
- -0.10772846435546857
- 0.0048589108276368975
- 0.8212551501464844
- 2.3901672363102477e-05
- 0.017758007202148374
- 5.781188964837369e-05
velocity:
- -0.0073261260986328125
- -0.0073261260986328125
- 0.0073261260986328125
- 0.0073261260986328125
- -0.0073261260986328125
- -0.0073261260986328125
- 0.0073261260986328125
- 0.0073261260986328125
- -0.0073261260986328125
- 0.0073261260986328125
- -0.0073261260986328125
- -0.0073261260986328125
- 0.0073261260986328125
- -0.0073261260986328125
effort:
- 0.610504150390625
- 1.001220703125
- 0.08547210693359375
- -1.3162384033203125
- 0.0512847900390625
- -0.3296699523925781
- 0.03662872314453125
- 0.5616607666015625
- -1.1965789794921875
- -0.15384674072265625
- -1.3162384033203125
- -0.08547210693359375
- -0.3540916442871094
- -0.03662872314453125
---
```

## [6] 使用单关节滑条测试实机控制

先确认机器人可靠吊装、急停可用，HAL 已启动，并且官方 `motion_control` 已停止。
HDU 转发状态、标准 ROS 指令及心跳，MDU 控制桥通过 Iceoryx 连接底层
（不能只启动 HDU 桥）：

```text
/ros2/body_drive/arm_joint_state  (joint_msgs/msg/JointState)
    -> /teleop/arm_joint_state (sensor_msgs/JointState)

/teleop/arm_joint_command (ROS 2 sensor_msgs/JointState)
    -> HDU /teleop/mdu/arm_joint_command -> MDU 控制桥
    -> /body_drive/arm_joint_command (Iceoryx joint_msgs/msg/JointCommand)
```

```bash
./usage/start_a3_ultra_state_bridge.sh
```

保持桥接终端运行，在另一个终端启动测试界面：

MDU 控制桥的编译、禁发测试、实机启用命令见 [MDU 控制桥说明](mdu_arm_bridge/README.md)。
PC 必须先收到 `/teleop/arm_bridge_ready=True` 才能开始规划和执行。之后心跳延迟
只在右下角监视窗标红，不再中止运动；MDU 明确发布 False 时仍会立即中止。
默认禁发模式不会让机器人运动。

```bash
conda activate a3_teleop
python pico_teleop_interface/robot_interfaces/a3_ultra/a3_ultra_joint_control_gui.py
```

界面先读取全部关节反馈，自动规划避障路径并移动到 MuJoCo 默认双臂姿态，
归位完成后才能启用滑条。贴身起点先逐步脱离，再在至少 10 cm 间距下归位。

执行中跟踪误差接近 0.03 rad 时暂停路径推进，仍以原频率发送最后目标；
误差降到 0.015 rad 后继续。等待超过 2 秒仍未追上，或超过 0.05 rad 硬上限，
会锁存故障并停发，不会自动恢复。终点等待实际到位也有超时。
这些参数集中在 `a3_ultra/config.py`；本机制不能替代正确的 PD 和重力补偿。

### 双臂控制诊断日志

PC GUI、HDU 转发桥、MDU 控制桥启动后都会打印 `Arm diagnostic log:` 和实际路径。
这是独立于 `teleoperation.log` 的逐帧 JSONL 日志：

- PC / HDU：各自的 `${XDG_STATE_HOME:-$HOME/.local/state}/pico_teleop/arm_diagnostics/`。
- MDU：控制桥部署目录下的 `logs/`。

每次启动新建文件，记录启动/正常退出、指令、反馈、收发间隔、暂停/恢复与故障原因。
反馈包含关节名称、实际角度、速度、力矩。三端用 `stamp_ns` 对应同一条 PC 指令，
MDU 另记录底层输出序号及输出时间戳。`iceoryx_publish_returned` 仅表示发布调用返回，
不是 HAL 或电机的执行确认。`monotonic_ns` 只用于本机时序，不能跨设备相减。

日志后台写盘；队列满时不阻塞控制，`dropped_total` 标记累计日志丢帧。
每次运行最多保留当前文件和 4 个轮转文件，每个约 40 MiB，合计约 200 MiB；
不同运行的文件独立保留，历史日志需要自行清理。强制杀进程可能丢失尚未写盘的末尾记录。

启用完整三端诊断前，关闭 GUI，重新运行 HDU 启动脚本（它会一起上传日志辅助文件），
并按 [MDU 控制桥说明](mdu_arm_bridge/README.md) 上传、编译新版 MDU 桥。
不要覆盖正在使用的共享库；先停止旧桥再编译/启动。PD 和保护阈值未因增加日志而改变。
若模型判断当前真机姿态已有穿透，也允许尝试脱离；沿途不得加深已有穿透，
不得新增碰撞。未找到满足条件的脱离路径时仍保持原位并报错。

启用控制后，拖动滑条并松开才提交一次目标；规划和运动期间滑条禁用。
滑条范围仍为所选关节的官方硬限位。虽然只设置一个关节的目标，但双臂联合
规划绕行，中途其他关节也可能运动。规划失败时保持当前位置，不直线追赶目标。
运动期间可点击 `Stop / hold` 中止剩余路径，持续保持最后已发送的角度，
不会继续追赶原目标。规划期间该按钮禁用；关闭窗口会取消尚未提交的规划。

规划使用固定腰、头、腿姿态的自碰撞模型，尚未纳入环境障碍和实时身体反馈。


