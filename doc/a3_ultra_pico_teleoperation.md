# A3 Ultra + PICO 实机遥操作启动流程

本文从机器人开机后开始。机器人侧需要同时运行两个程序：

```text
PICO → PC 遥操作程序 → HDU ROS 转发桥 → MDU Iceoryx 控制桥 → HAL → 双臂
```


一个 terminal:
```bash
# 进入 HDU
# ssh agi@192.168.31.85
ssh agi@10.42.10.11
# 进入 MDU
ssh agi@10.42.10.12
#停止 motion player
curl -fsS -H 'content-type:application/json' \
  -X POST 'http://127.0.0.1:50080/json/stop_app' \
  -d '{"app_name":"motion_player"}'
# 关闭 motion模式

# 温度监测和散热风扇控制模块
# aimcli em stop hal_thermal
aimcli em stop mc

# 在 MDU
cd /agibot/data/home/agi/mdu_arm_bridge
A3_ENABLE_COMMANDS=1 bash run.sh
```

一个 terminal:
```bash
# 有线连接的时候
./usage/connect_pico_usb.sh
# pico 的 pc service
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh
```
然后在 pico 中启动软件


另一个 terminal:
```bash
./usage/start_a3_ultra_state_bridge.sh
```





> 实机启动后会自动移动双臂。开始前应固定机器人、清空双臂周围空间、确认急停可用，
> 并关闭滑条 GUI 和其他关节指令发送程序。

## 1. 关闭 motion_player 和 motion_control

先从 PC 登录机器人 HDU。下列 IP 是当前机器人地址；如果 Wi-Fi 地址改变，需要替换
`192.168.31.85`：

```bash
ssh agi@192.168.31.85
```

再从 HDU 进入 MDU：

```bash
ssh agi@10.42.10.12
```

在 **MDU** 中先停止动作播放器：

```bash
curl -fsS -H 'content-type:application/json' \
  -X POST 'http://127.0.0.1:50080/json/stop_app' \
  -d '{"app_name":"motion_player"}'
```

然后停止官方 motion 控制模块：

```bash
aimcli em stop motion_control
```

这一步会退出官方整机 motion 控制。后续双臂由本项目的底层控制桥负责，不能同时
运行官方 `motion_control` 或其他双臂控制程序。

## 2. 启动两个机器人侧转发程序

两个终端都要保持运行。

### 2.1 启动 MDU 控制桥

在刚才的 **MDU 终端**中执行：

```bash
cd /agibot/data/home/agi/mdu_arm_bridge
A3_ENABLE_COMMANDS=1 bash run.sh
```

看到下面的信息说明控制桥已启用实机指令转发：

```text
LIVE: HAL command forwarding enabled
```

此时没有 PC 指令并不会立即运动。不要关闭这个终端；若出现 `BRIDGE FAULT`，需要先
解决打印出的原因，再重启此命令。

### 2.2 启动 HDU ROS 转发桥

在 **PC 项目根目录的新终端**中执行：

```bash
./usage/start_a3_ultra_state_bridge.sh
```

脚本会把桥接文件上传到 HDU，并在 HDU 前台启动。看到类似下面的信息表示 HDU 已经
同时转发双臂状态、控制指令和 MDU 就绪心跳：

```text
Forwarding /ros2/body_drive/arm_joint_state -> /teleop/arm_joint_state
```

不要关闭这个终端。按 `Ctrl+C` 时，脚本会清理本次启动的 HDU 桥接进程。

## 3. 使用数据线连接 PICO

用支持数据传输的 USB 线连接 PICO 和 PC。首次连接时戴上头显，在弹窗中允许这台
PC 进行 USB 调试。如果 PC 没有 `adb`，先安装：

```bash
sudo apt install adb
```

在 **PC 项目根目录的新终端**中建立 USB 端口转发：

```bash
./usage/connect_pico_usb.sh
```

脚本应能在 `adb devices` 中看到 PICO，并显示下面的端口映射：

```text
tcp:63901 tcp:63901
```

然后启动 XRoboToolkit PC Service：

```bash
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh
```

打开 PICO 中的 XRoboToolkit，把 `PC Service` 地址设置为 `127.0.0.1`，连接到
`WORKING`。有线模式必须填写 `127.0.0.1`，不要填写 PC 的 Wi-Fi 地址。

每次拔插数据线、重启 PICO 或 ADB 转发规则丢失后，都需要重新运行
`./usage/connect_pico_usb.sh`。

## 4. 启动 PICO 遥操作

启动前在 PICO 的 XRoboToolkit 中完成以下设置：

1. PC Service 状态为 `WORKING`。
2. 开启 `Head`。
3. 选择 `Object Tracking`，确认两个 Motion Tracker 都在线。
4. 开启 `Send`。

然后在 **PC 项目根目录的新终端**中执行：

```bash
conda activate a3_teleop
python pico_teleop_interface/robot_interfaces/a3_ultra/a3_ultra_interface.py
```

不需要添加 `--real`。直接运行这个文件会：

1. 启动或复用 XRoboToolkit PC Service；
2. 连接 HDU/MDU 控制链路并读取真实双臂角度；
3. 规划无碰撞路径，让真机双臂运动到约定的默认姿态；
4. 打开 MuJoCo，并用真实关节反馈显示机器人姿态；
5. 归位完成后开始 6 秒倒计时；
6. 用倒计时结束后的首帧 PICO 数据建立相对零点，随后开始遥操作。

倒计时期间保持头部正视前方、双臂向前伸且肘部微弯。看到
`Heading calibrated` 后，头显朝向与双臂零点已经建立。

## 启动顺序速查

```text
机器人开机
   ↓
MDU：停止 motion_player
   ↓
MDU：停止 motion_control
   ↓
MDU：A3_ENABLE_COMMANDS=1 bash run.sh
   ↓
PC：./usage/start_a3_ultra_state_bridge.sh
   ↓
PC：USB 连接 PICO，运行 ./usage/connect_pico_usb.sh
   ↓
PC：启动 XRoboToolkit PC Service
   ↓
PICO：PC Service 地址设为 127.0.0.1，连接到 WORKING
   ↓
PICO：Head + Object Tracking(2) + Send
   ↓
PC：python .../a3_ultra_interface.py
   ↓
真机自动归位 → 6 秒标定 → 开始遥操作
```

## 停止遥操作

先关闭 `a3_ultra_interface.py` 的 MuJoCo 窗口或在其终端按 `Ctrl+C`，再停止 HDU
转发桥，最后停止 MDU 控制桥。程序退出后电机是否继续保持由 HAL/驱动决定，因此退出
前应确保双臂处于安全姿态，并准备使用急停。
