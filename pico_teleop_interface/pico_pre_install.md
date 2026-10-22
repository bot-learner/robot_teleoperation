# PICO 遥操作接口：安装前准备

本目录计划封装 PICO 头显与手柄的输入，将位姿和按键状态传给机器人端。当前仓库尚无通信协议、机器人 SDK 或 ROS 配置，因此这里先列出**开发和联调所需的软件**；具体 Python/ROS 依赖应在接口实现后按代码固定版本。

## 必需

1. **PICO 头显及配套手柄**：确认设备型号、系统版本，完成配对和固件更新。在头显上开启开发者模式和 USB 调试，用数据线连接开发电脑，并在头显内允许调试授权。
2. **Unity Hub + Unity Editor**：用于构建运行在头显上的采集应用。安装 Editor 时勾选 **Android Build Support** 以及其 **Android SDK & NDK Tools**、**OpenJDK** 子模块。Unity 版本应与选定的 PICO SDK 兼容；PICO Unity Integration SDK 当前要求 Unity 2021.3.26 至 Unity 6。[PICO SDK 下载与版本要求](https://developer.picoxr.com/resources/?platform=unity)
3. **PICO Unity SDK，选择一条路线**：优先根据设备型号和系统版本选择 [PICO Unity Integration SDK](https://developer.picoxr.com/document/unity/)；若项目确定采用 OpenXR，再改用 [PICO Unity OpenXR SDK](https://developer.picoxr.com/document/unity-openxr/about-the-pico-unity-openxr-sdk/)。不要在尚未确定方案时同时安装两套并混用示例代码。使用 SDK 获取头显/手柄位姿、按键及扳机输入。
4. **Android Platform Tools（ADB）**：用于确认头显连接、安装 APK、查看运行日志。若 Unity 自带的 `adb` 可用，可以直接使用，无需再装一份。连接后执行 `adb devices`，设备应显示为 `device`。
5. **机器人端运行环境**：安装与后续接收程序匹配的 Python 3、`pip` 和 `venv`。若采用 Python 标准库 `socket` 接收 UDP/TCP 数据，初版无需额外网络包；待通信协议和机器人 SDK 确定后再建立 `requirements.txt` 并锁定依赖。

## 按方案安装

- **ROS 2**：只有机器人控制栈使用 ROS 2 时才安装与机器人系统匹配的 ROS 2 发行版，以及需要的消息类型和驱动包。
- **PICO Developer Center / Live Preview**：需要图形化设备管理或头显内实时预览时再安装；普通 APK 安装和日志查看可使用 ADB。相关工具见 [PICO 资源页](https://developer.picoxr.com/resources/?platform=unity)。
- **PICO 体感追踪器**：只有遥操作方案需要手柄之外的身体追踪输入时才准备，并确认所选设备和 SDK 支持。

## 联调前检查

1. Unity 工程目标平台设为 Android；按 PICO SDK 的项目校验提示检查架构、图形接口和最低 Android 版本。[PICO 项目校验](https://developer.picoxr.com/document/unity/project-validation/)
2. 构建并安装测试 APK，确认可读取头显和左右手柄的位姿、按键状态。
3. 头显与机器人端接入可互通的网络，明确接收端 IP、端口、坐标系、单位、更新频率和断连处理，再接入机器人控制接口。

> 当前仅为预安装清单；还没有 PICO 应用、数据传输程序或机器人控制代码，文中的软件未自动安装。
