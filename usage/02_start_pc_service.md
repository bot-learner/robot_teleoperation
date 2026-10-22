# 第二步：启动 PC Service

以下命令均在项目根目录 `teleoperation_a3_ultra/` 下执行。PC Service 支持 Ubuntu 22.04 和 Ubuntu 24.04 的 amd64 系统。

## 1. 首次安装

查看当前系统版本和架构：

```bash
. /etc/os-release
echo "$PRETTY_NAME"
dpkg --print-architecture
```

架构应为 `amd64`。根据 Ubuntu 版本选择一个安装包，不要混装：

```bash
# Ubuntu 22.04
sudo apt install ./pico_teleop_interface/third_party/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

# Ubuntu 24.04
sudo apt install ./pico_teleop_interface/third_party/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
```

如果仓库中没有安装包，先下载：

```bash
./pico_teleop_interface/third_party/download_xrobotoolkit_pc_service.sh
```





## 2. 启动服务

仅启动遥操作所需的核心后台服务：

```bash
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh
```

也可以显式使用 `--service`，效果相同：

```bash
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh --service
```

需要演示或录制界面时，选择一个对应参数：

```bash
# Qt 二维演示界面
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh --2d

# Unity 三维演示界面
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh --3d

# 数据录制界面
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh --recorder
```

无参数和 `--service` 会在启动成功后返回命令行；带界面的模式会占用当前终端，关闭界面后命令才会退出。脚本会复用已经运行的核心服务，不会重复启动。查看所有参数：

```bash
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh --help
```

看到“启动成功”和进程 PID 即表示核心服务已运行。也可以手动检查：

```bash
pgrep -af RoboticsServiceProcess
```


```text
       手掌
        |
   -----腕关节-----
        |
    [ Tracker ]       ← 前臂背侧，离腕关节几厘米
        |
       手肘
```




## 3. 让 PICO 连接 PC

### 方式一：局域网连接

1. 确保 PICO 和 PC 在同一个局域网内。
2. 查询 PC 的局域网 IPv4 地址：

   ```bash
   hostname -I
   ```

3. 打开 PICO 中的 `XRoboToolkit`，在 `PC Service` 地址中输入 PC 的局域网 IPv4 地址。
4. 连接成功并显示 `WORKING` 后，按任务选择 `Head`、`Controller`、`Hand`、`Full Body` 或 `Object Tracking`，然后启用 `Send`。

### 方式二：USB 有线连接

用支持数据传输的 USB 线连接 PICO 和 PC，在头显中允许 USB 调试，然后执行：

```bash
./usage/connect_pico_usb.sh
```

脚本依次列出 ADB 设备、建立 `63901` 端口的 USB 反向转发、显示转发规则。
将 PICO 中的 `PC Service` 地址改为 `127.0.0.1`，而不是 PC 的局域网 IP，
再连接至 `WORKING`。PC Service 仍需按第 2 节启动；脚本不负责启动服务或 APK。
若拔线或重启后转发规则丢失，重新运行脚本。

A3 Ultra 双臂遥操作使用两个独立 Motion Tracker：开启 `Head`，选择
`Object Tracking`，确认 `TrackerNum` 为 `2`，然后启用 `Send`。
`Head` 用于首次标定操作者的水平前方，当前 A3 IK 不使用 Controller 数据。

启动 Python 后，在 6 秒倒计时内站稳，头显正视身体前方，双臂前伸、肘部微弯，
不要交叉。倒计时结束后，程序使用首个有效的头显及双 Tracker 帧同时标定朝向和
双手零点；看到 `Heading calibrated` 即表示朝向已固定，之后转头不会改变
双臂控制方向。若显示 `Waiting for heading calibration`，检查 `Head` 和
`Send` 是否开启，并保持正视前方。短时断连恢复不会自动改变标定；需要重新
标定时重启程序，或调用 `robot.reset_ik_reference()` 后摆好上述姿态。

## 停止服务

```bash
pkill -f '/opt/apps/roboticsservice/RoboticsServiceProcess'
```

如果 PICO 无法连接，请依次确认：两台设备处于同一网段、填写的 IP 正确、服务进程存在，以及 PC 防火墙没有拦截通信。
