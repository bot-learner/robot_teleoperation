# 第三方依赖使用说明

本目录存放 PICO 遥操作所需的第三方项目、安装包和下载脚本。目前包括：

- `EgoHumanoid/`：上游遥操作与数据采集项目。
- `download_xrobotoolkit_pc_service.sh`：XRoboToolkit PC Service 下载脚本。
- `install_xrobotoolkit_python_sdk.sh`：Python SDK 安装与导入检查脚本。
- `XRoboToolkit-PC-Service-Pybind/`：Python SDK 源码；本项目版本已移除官方的 3 Tracker 截断。
- `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb`：Ubuntu 22.04 安装包。
- `XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb`：Ubuntu 24.04 安装包。

## 1. 下载 PC Service

已经下载过安装包时可跳过本节。重新下载或断点续传：

```bash
./pico_teleop_interface/third_party/download_xrobotoolkit_pc_service.sh
```

脚本会把 Ubuntu 22.04 和 Ubuntu 24.04 的安装包下载到当前 `third_party` 目录。

## 2. 安装 PC Service

先确认系统版本：

```bash
. /etc/os-release
echo "$PRETTY_NAME"
dpkg --print-architecture
```

当前开发机是 Ubuntu 22.04 amd64，应安装 22.04 版本：

```bash
sudo apt install ./pico_teleop_interface/third_party/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

Ubuntu 24.04 amd64 使用：

```bash
sudo apt install ./pico_teleop_interface/third_party/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
```

不要跨 Ubuntu 版本混装两个包。安装内容位于：

```text
/opt/apps/roboticsservice/
```

## 3. 启动和停止

EgoHumanoid 联调时，建议在项目根目录运行启动脚本：

```bash
./pico_teleop_interface/third_party/start_xrobotoolkit_pc_service.sh
```

脚本会检查 PC Service 是否已安装、避免重复启动，并验证核心进程是否成功运行。也可以直接执行上游启动脚本：

```bash
/opt/apps/roboticsservice/runService.sh
```

确认服务是否运行：

```bash
pgrep -af RoboticsServiceProcess
```

停止服务：

```bash
pkill -f '/opt/apps/roboticsservice/RoboticsServiceProcess'
```

PC Service 不会注册为 systemd 服务，也不会默认开机自启动。

### 演示和录制工具

```bash
# 核心服务 + Qt 二维演示
/opt/apps/roboticsservice/run2D.sh

# 核心服务 + Unity 三维演示
/opt/apps/roboticsservice/run3D.sh

# 核心服务 + 数据录制工具
/opt/apps/roboticsservice/runRobotDataRecorder.sh
```

桌面应用菜单中的 `XRoboToolkit-PC-Service` 实际启动 `run3D.sh`。仅向 EgoHumanoid 提供数据时，使用 `runService.sh` 即可。

## 4. 连接 PICO

1. 让 PICO 和运行 PC Service 的电脑连接同一局域网。
2. 查询电脑的局域网地址：

   ```bash
   hostname -I
   ```

3. 在 PICO 中启动 XRoboToolkit APK。
4. 在 `PC Service` 地址中输入电脑的局域网 IPv4 地址并连接。
5. 连接成功后确认界面状态为 `WORKING`。
6. 根据任务启用 `Head`、`Controller` 或 `Hand`；使用体感追踪器时选择 `Full Body`，最后启用 `Send`。

如果无法连接，优先检查：双方是否处于同一网段、IP 是否填写正确、PC Service 进程是否存在，以及主机防火墙是否拦截通信。

## 5. 安装 Python SDK

`BaseRobotInterface` 直接使用 Python SDK 读取数据。安装到当前 `python3` 对应的环境：

```bash
./pico_teleop_interface/third_party/install_xrobotoolkit_python_sdk.sh
```

官方版本的安装命令是：

```bash
python3 -m pip install xrobotoolkit-sdk
```

本项目为了支持超过 3 个独立 Tracker，安装脚本实际会编译并安装 `third_party/XRoboToolkit-PC-Service-Pybind/` 中的本地修正版，并强制替换同版本的官方包。

如果使用 Conda 或虚拟环境，请先激活目标环境再运行脚本。PC Service 的 `.deb` 和 Python SDK 是两个独立组件，两者都需要安装。

## 6. Whole Body 与 Object Tracking

追踪模式需要先在 PICO 的 XRoboToolkit App 中选择：

- `Full Body`：输出 24 个人体关节，Python 端使用 `get_body_joints_pose()`；
- `Object Tracking`：输出独立 Motion Tracker，Python 端使用 `get_motion_tracker_pose()` 等接口。

Python SDK 负责读取当前模式的数据，不能从 PC 端替 PICO App 切换模式。创建机器人接口时，`tracking_mode` 必须与 PICO App 中的选择一致。

官方 Python Binding 原本把 `Motion.joints` 截断为最多 3 个。本目录保存的修正版改为动态数组，会返回数据包中实际包含的全部 Tracker，包括 5 Tracker 数据。这里取消的是 Python Binding 的截断；PICO App 和 PC Service 必须确实发送对应数量的数据。

## 7. 与 EgoHumanoid 配合

这个 `.deb` 只安装 PC Service、C++ SDK 和演示程序，不会安装供 Python 导入的绑定。EgoHumanoid 还需要单独配置：

- `XRoboToolkit-PC-Service-Pybind`；
- EgoHumanoid 的 Python 环境和依赖；
- PICO 端的 XRoboToolkit APK。

EgoHumanoid 上游说明见：

- [`EgoHumanoid/README.md`](EgoHumanoid/README.md)
- [`EgoHumanoid/data_collection/human_data/install.md`](EgoHumanoid/data_collection/human_data/install.md)
- [`EgoHumanoid/data_collection/robot_data/README.md`](EgoHumanoid/data_collection/robot_data/README.md)

上游 `human_data/install.md` 示例使用 Ubuntu 24.04 安装包；当前开发机是 Ubuntu 22.04，安装时应以本文件的 22.04 命令为准。

## 8. 卸载

```bash
sudo apt remove roboticsservice
```

如需同时删除系统级配置文件：

```bash
sudo apt purge roboticsservice
```

卸载系统软件不会删除本目录保存的 `.deb` 文件和 EgoHumanoid 源码。
