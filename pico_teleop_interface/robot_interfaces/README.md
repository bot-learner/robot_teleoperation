# Robot Interfaces

本目录用于存放不同机器人型号的遥操作适配接口。

`BaseRobotInterface` 通过 `xrobotoolkit_sdk` 直接读取头显、手柄、手部、人体或独立 Tracker 数据，不依赖 ROS。各机器人适配器负责把统一的 `TeleoperationInput` 转换为具体机器人的控制命令。

建议每种机器人使用独立子目录：

```text
robot_interfaces/
├── README.md
├── base_interface.py
├── a3_ultra/
└── <robot_model_b>/
```

单个机器人适配器建议包含：

- 机器人连接和状态读取；
- 坐标系与单位转换；
- 逆运动学或动作重定向；
- 关节限位、速度限制和急停处理；
- 遥操作输入到机器人控制命令的映射；
- 配置文件、启动脚本和基础测试。

第三方机器人 SDK 不要直接复制到这里，应放在 `../third_party/`，本目录只保留项目自身的适配代码。

所有机器人适配器应继承 `BaseRobotInterface`，并实现 `solve_inverse_kinematics()` 和 `send_to_robot()`。调用适配器的 `run()` 后，基类会启动 PC Service、初始化 SDK，并按 `control_rate_hz` 指定的频率执行“读取数据 → 解算 → 发送”循环；按 `Ctrl+C` 时会关闭 SDK。

追踪模式必须与 PICO App 中的选择一致。A3 Ultra 当前使用两个独立
Motion Tracker，因此 PICO App 必须选择 `Object Tracking`，同时开启 `Head`
以标定水平朝向。6 秒倒计时内正视身体前方，双臂前伸、肘部微弯、不要交叉：

```python
robot = A3UltraInterface(tracking_mode="object")
robot.run_sim()
```

`run_sim()` 只读取一次 PICO/IK，并用两个独立进程同时显示直接 IK + PD
和 SONIC 的 MuJoCo 结果。

如果 PC Service 已由其他程序负责管理，可在构造时设置 `auto_start_pc_service=False`。
