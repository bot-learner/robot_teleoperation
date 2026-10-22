# 第一步：在 PICO 上安装 APK

本项目使用 `XRoboToolkit-PICO 1.1.1` 将 PICO 的头显、手柄和体感追踪数据发送到 PC。

## 1. 下载 APK

在 PC 上执行：

```bash
wget -c https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk
```

## 2. 连接 PICO

1. 在 PICO 中开启开发者模式和 USB 调试。
2. 使用 USB 数据线连接 PICO 与 PC。
3. 戴上头显，在弹窗中允许这台 PC 进行 USB 调试。
4. 在 PC 上确认设备已连接：

   ```bash
   adb devices
   ```

设备序列号后应显示 `device`。如果显示 `unauthorized`，请在头显中重新确认 USB 调试授权。

## 3. 安装 APK

```bash
adb install -r XRoboToolkit-PICO-1.1.1.apk

adb install -r usage/XRoboToolkit-PICO-1.1.1.apk
```

命令显示 `Success` 后，在 PICO 应用列表的“未知来源”中找到并启动 `XRoboToolkit`。

> 下一步启动 PC Service 后，PICO 和 PC 必须连接到同一个局域网。
