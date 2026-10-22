# Motion Tracker 双臂映射

## 核心思路

程序不把 Tracker 在房间里的绝对位置直接发送给机器人，只传递它相对于启动位置的变化。

```text
启动时                       移动后

左 Tracker ●                 左 Tracker ● ───→ 移动 10 cm
             \                            \
              \                            \
         记录为相对零点                 计算位置变化

机器人左掌 ■                 机器人左掌 ■ ───→ 移动 10 cm
```

默认 `position_scale = 1.0`：

```text
人手移动 10 cm  ─────────────→  机器人手掌移动约 10 cm
人手旋转 30°    ─────────────→  机器人手掌旋转约 30°
```

计算方式：

```text
机器人目标位置
= 机器人初始位置
+ position_scale × 朝向校正 ×（Tracker 当前位置 - Tracker 启动位置）
```

这样做可以避免 PICO 原点与机器人基座原点不同导致机器人启动时突然跳动。

朝向校正在首帧读取头显的水平朝向，把人的前方对齐到机器人前方：

```text
标定时头显看向的前方 ──→ 机器人前方
操作者右方           ──→ 机器人右方
上方                 ──→ 机器人上方
```

此转换标定后固定，转头不会改变控制方向；手部旋转变化也使用同一个朝向校正。

## 序列号和零点

```text
Tracker 序列号 ──→ 判断这是左侧还是右侧设备
启动时的位姿   ──→ 作为本次运行的相对零点
```

序列号确认后不会因为双臂交叉而改变；每次倒计时标定时正视身体前方，双臂
向前平举、肘部微弯、不要交叉，以便建立朝向和双手零点。

## 数据流程

```text
左右 Motion Tracker
        │
        ▼
PICO Object Tracking
        │
        ▼
BaseRobotInterface 读取位姿
        │
        ▼
A3UltraInterface 计算相对运动和双臂 IK
        │
        ▼
MuJoCo 中的 A3 Ultra 双臂
```

## 启动

在 PICO 中确认：

```text
Mode       = Object Tracking
Head       = ON
TrackerNum = 2
Send       = ON
```

然后运行：

```bash
python -m pico_teleop_interface.robot_interfaces.a3_ultra.sim \
  --left-tracker-serial <左侧序列号> \
  --right-tracker-serial <右侧序列号> \
  --position-scale 1.0
```

倒计时后首个有效头显及双 Tracker 帧建立朝向和相对零点；缺少头显方向时会
等待并提示开启 Head 追踪。短时断连恢复保持原标定；重新标定需要重启，或调用
`robot.reset_ik_reference()` 后重新摆好上述姿态。
