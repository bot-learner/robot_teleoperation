# MDU 双臂 ROS 2 → Iceoryx 控制桥

状态链路：HAL → ROS 2 `/ros2/body_drive/arm_joint_state` → HDU 状态桥 → PC。
控制链路：PC `/teleop/arm_joint_command` → HDU 标准 ROS 转发
`/teleop/mdu/arm_joint_command` → **MDU 本机**控制桥 → Iceoryx
`/body_drive/arm_joint_command` → HAL。不改变官方 HAL 配置，不加载全身策略。

## 上传和编译

在 PC 项目根目录执行，Wi-Fi 地址按现场修改：

```bash
scp -r -o ProxyJump=agi@192.168.31.85 usage/mdu_arm_bridge \
  agi@10.42.10.12:/agibot/data/home/agi/
ssh -J agi@192.168.31.85 agi@10.42.10.12
cd /agibot/data/home/agi/mdu_arm_bridge
bash build.sh
```

要求固件 AimRT **1.7.0**、系统 ROS Jazzy、固件 `joint_msgs`。
首次构建下载固定版本 AimRT 和 libunifex；只编译本模块及其依赖，不替换固件库。
离线构建可用 `bash build.sh -DAIMRT_SOURCE_DIR=... -DUNIFEX_SOURCE_DIR=...`。

构建会在头文件副本上应用 `patches/` 中的两个兼容性补丁（复用 SONIC 的已有修复）：
为 Fast DDS 选择具体消息类型支持；先在可扩容的 ROS 缓冲区中序列化，再一次性
复制到 Iceoryx 缓冲区。避免 `typesupport identifier ... not supported` 和
`unable to dynamically resize serialized message`，不修改固件安装目录。
不要直接重编译正在使用的共享库；先停止控制桥，或使用
`A3_BRIDGE_BUILD_DIR=build_fixed bash build.sh` 单独构建。

## 默认禁发测试

在 MDU 运行：

```bash
bash run.sh
```

默认 `A3_ENABLE_COMMANDS=0`：订阅 `/teleop/mdu/arm_joint_command`，输出到
Iceoryx `/teleop/dry_run/arm_joint_command` 并回读，绝不发布 HAL 控制话题。
MDU `/teleop/mdu/arm_bridge_ready` 经 HDU 转发为 `/teleop/arm_bridge_ready`，
禁发模式始终为 False，PC GUI 不会启动运动。
测试指令仍需包含完整 14 个关节和递增时间戳，角度应使用当前反馈。
成功回读显示 `DRY_RUN_ICEORYX_RECEIVED joints=14 PD=OK`。

## 实机启用

先关闭 PC GUI 及其他发送者，确认官方 `motion_control` 已停止，底层 HAL 和
`iox-roudi` 正常，并按现有实机流程做好支撑。停止禁发测试后，在 MDU 显式运行：

```bash
A3_ENABLE_COMMANDS=1 bash run.sh
```

在 PC 另一个终端启动/重启 HDU ROS 转发桥（状态、标准指令、就绪心跳）：

```bash
./usage/start_a3_ultra_state_bridge.sh
```

最后启动 PC GUI。它会在读取反馈、收到 True 心跳后自动规划归位。
心跳含义是“实机模式启用、Iceoryx 状态新鲜、桥接无故障”，不是电机执行确认；
PC 仍用实际关节反馈监测跟踪误差。

控制桥校验名称、限位和递增时间戳，使用参考 PD 参数。较大的目标不会因角度跳变或
反馈偏差锁存故障，也不按反馈角度截断目标。仍保留单帧限速：最大变化为
`0.5 × min(指令间隔秒数, 0.1) + 0.005 rad`；诊断日志的 `limited` 和
`output_position` 可用于确认单帧限速是否生效。
超过 200 ms 无新指令后锁存故障，**停止转发、不重发旧目标**，需重启桥接。
遇到序列化报错时，不能通过放宽此超时解决；应修复并重新编译控制桥。
PC 指令中断、MDU 进程退出后的电机行为取决于 HAL/驱动的超时策略，本桥不承诺
进程退出后持续保持；`Stop / hold` 则由 PC 持续发送最后已执行的角度。
多进程不能同时争用控制桥/双臂控制权。

## 专用诊断日志

每次启动在本目录的 `logs/mdu_*.jsonl` 新建日志，启动时打印准确路径。
后台逐帧记录实际关节位置/速度/力矩、收到的 PC 指令时间戳、接收间隔、
拒收原因、发布调用返回后的输出序号及故障。`iceoryx_publish_returned` 不代表电机执行确认。
原始 PC 时间戳 `stamp_ns` 与 PC/HDU 日志对应；本机 `monotonic_ns` 不能跨机器相减。
每次运行最多约 200 MiB（40 MiB × 5），历史运行独立保留；`dropped_total` 记录队列溢出。
退出会排空日志队列；断电或强制杀进程无法保证最后几条记录落盘。
更新需连同 `diagnostic_log.h` 上传、重新编译；不改变 PD、超时或控制话题。
