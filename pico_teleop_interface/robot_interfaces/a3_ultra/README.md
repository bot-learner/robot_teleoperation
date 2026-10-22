# A3 Ultra Interface

本目录用于实现 A3 Ultra 的遥操作适配器。

## 已下载的双臂 IK 材料

- `models/A3-A3U-robot-model/`：智元官方 A3/A3 Ultra 模型仓库；
- `models/A3-A3U-robot-model/a3_ultra_t3d0/urdf/model_planning.urdf`：Pinocchio 后续加载的主模型；
- `models/A3-A3U-robot-model/a3_ultra_t3d0/urdf/model_planning.srdf`：官方自碰撞过滤信息；
- `models/A3-A3U-robot-model/a3_ultra_t3d0/meshes/`：碰撞与可视化网格；
- `models/A3-A3U-robot-model/a3_ultra_t3d0/mjcf/a3_ultra_t3d0.xml`：与本地 `humanoid_rl` 使用型号一致的 MuJoCo 模型；
- `references/pinocchio/examples/inverse-kinematics.py`：Pinocchio 官方阻尼最小二乘 IK 示例；
- `requirements_ik.txt`：后续双臂 IK 的 Python 依赖。

本机 `/home/art/wanghan_prj/humanoid_rl/config/robots/a3_ultra.yaml` 标明当前项目使用 `a3_ultra_t3d0`，因此后续默认使用 T3D0 模型。仓库中也保留了 T2.5 模型；只有确认真机是 T2.5 时才切换。

重新下载或更新材料：

```bash
./pico_teleop_interface/robot_interfaces/a3_ultra/download_ik_resources.sh
```

安装 IK 依赖：

```bash
python3 -m pip install -r \
  pico_teleop_interface/robot_interfaces/a3_ultra/requirements_ik.txt
```

当前阶段只解算以下 14 个双臂关节，腰、头和手指保持固定：

```text
left_shoulder_pitch_joint     right_shoulder_pitch_joint
left_shoulder_roll_joint      right_shoulder_roll_joint
left_shoulder_yaw_joint       right_shoulder_yaw_joint
left_elbow_joint              right_elbow_joint
left_wrist_roll_joint         right_wrist_roll_joint
left_wrist_pitch_joint        right_wrist_pitch_joint
left_wrist_yaw_joint          right_wrist_yaw_joint
```

末端任务 Frame 使用：

```text
left_palm_Link
right_palm_Link
```

官方 Pinocchio 示例只演示单臂单末端。本项目的实现已将左右掌的 6D 误差和雅可比纵向拼接，并只更新上述 14 个关节；官方示例本身不能直接用于控制真机。

## 双臂 IK 接口

`a3_ultra_interface.py` 中的 `A3UltraInterface` 已继承 `BaseRobotInterface`，并实现 `solve_inverse_kinematics()`：

- 使用 `Object Tracking` 模式读取两个独立 Motion Tracker；
- 首帧有效头显及左右 Tracker 同时标定水平朝向和双手零点，机器人保持当前双掌参考位姿；
- XRoboToolkit 实际输出（X 右、Y 上、-Z 前）转换为机器人坐标系（X 前、Y 左、Z 上）；
- 左右掌的 6D 误差和雅可比组成一个 12D 阻尼最小二乘问题；
- 接近雅可比奇异位形时增大阻尼，减小关节解对输入误差的放大；
- 以较弱的上一帧关节姿态正则项改善冗余解连续性，不设置帧间速度硬上限；
- 每次迭代重新计算碰撞梯度，并通过最多 6 次回溯只接受末端误差下降、未越过碰撞距离边界的候选姿态；
- 零空间姿态代价使肘部优先保持向后弯曲的初始分支；
- 自碰撞代价默认权重为 `100`，距离 `10 cm` 时开始排斥，`2 cm` 作为硬安全边界；
- 关节进入靠近上下限的 5% 区域后，关节限位代价开始增大；
- 左右掌的位置和旋转分别应用 `2 mm` / `0.5°` 死区，与上次接受的目标比较；
- 只更新上面列出的 14 个双臂关节，并应用 URDF 关节限位；
- 返回 `A3UltraIKResult`，其中包含 14 个关节目标、完整模型配置、收敛状态、迭代次数和残差。

直接传入 `TeleoperationInput` 调用：

```python
result = robot.solve_inverse_kinematics(tracking_input)
if result is not None and result.converged:
    print(dict(zip(result.joint_names, result.joint_positions)))
```

需要重新建立 Tracker 与机器人双掌的相对零点时调用：

```python
robot.reset_ik_reference()
```

`send_to_robot()` 目前会明确抛出 `NotImplementedError`。在确认 A3 Ultra SDK 或通信协议前，不要调用继承自基类的完整 `run()` 控制循环。接入真机时还需补充速度限制、输入超时保护、初始姿态对齐和急停逻辑。

IK 设计参考 [XRoboToolkit 基础控制器](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/blob/main/xrobotoolkit_teleop/common/base_teleop_controller.py)
中的末端软任务和可操作性任务思路。本项目仍使用 Pinocchio 阻尼最小二乘，
没有迁移到 Placo QP，也没有实现与可操作性最大化等价的任务；自适应阻尼仅用于改善奇异处的数值稳定性。
上游当前基础控制器中的动能正则化调用被注释，本项目没有启用惯量加权的动能正则化。
碰撞检查针对离散候选姿态，不提供两帧之间的连续扫掠碰撞保证。

## MuJoCo 双臂预览

`sim.py` 使用 `sim/` 中从官方低层控制压缩包提取的 A3 Ultra T3D0 模型，
实时执行“读取 Motion Tracker → 双臂 IK → 更新 MuJoCo 关节”的循环：

```bash
python -m pico_teleop_interface.robot_interfaces.a3_ultra.sim
```

PICO 端同时开启 `Head`、`Object Tracking` 和 `Send`。恰好连接两个 Tracker
时，程序按首帧头显水平朝向校正后的左右位置绑定设备；启动倒计时请头显正视
身体前方，双臂向前平举、肘部微弯，并且不要交叉。也可以显式指定序列号：

```bash
python -m pico_teleop_interface.robot_interfaces.a3_ultra.sim \
  --left-tracker-serial <LEFT_SERIAL> \
  --right-tracker-serial <RIGHT_SERIAL>
```

也可以在 Python 中启动：

```python
from pico_teleop_interface.robot_interfaces.a3_ultra import A3UltraInterface

robot = A3UltraInterface()
robot.run_sim()
```

倒计时后首帧有效头显和双 Tracker 数据会建立朝向标定和相对零点；之后转头
不会改变坐标转换，短时断连恢复也不会重新标定。缺少有效头显方向时会提示
`Waiting for heading calibration`，并等待，不使用默认方向继续运行。
`robot.reset_ik_reference()` 会清除朝向和零点，但保留左右设备序列号绑定。
程序会用两个独立进程显示
`IK + PD` 和 `SONIC` 两个 MuJoCo 窗口；关闭任一窗口或按 `Ctrl+C` 会结束两者。
两个窗口使用同一份最新 IK 结果，不会积压历史命令。两边均由
`SonicUpperBodySimulator` 加载本项目的 `sim/model/mjcf/a3_ultra_t3d0.xml`，
各自拥有独立的
MuJoCo 模型和数据实例。初始关节角、固定身体约束、PD 增益、力矩限幅和物理
步长完全一致；控制链路是唯一的区别：

- 左侧：IK 关节角 → PD → MuJoCo。
- 右侧：IK 关节角 → SONIC → PD → MuJoCo。

两边的 PD 都执行 `tau = Kp * (q_target - q) - Kd * dq`，使用 `A3UltraSimulator`
的同一组增益和 Ultra 力矩限幅，不额外叠加 `qfrc_bias`。按单臂顺序
肩 pitch/roll/yaw、肘、腕 roll/pitch/yaw：

- `Kp = [40, 40, 30, 30, 30, 20, 20]`。
- `Kd = [3, 3, 2, 2, 2, 2, 2]`。
- 力矩限幅（Nm）：`[60, 60, 24, 24, 24, 6, 6]`。

SONIC 的 29 维输入保留 checkpoint 的关节排列、归一化基准和动作缩放，
但关节角、速度及骨盆姿态直接取自 Ultra 模型，不再调用 A3 T2.5 的闭链求解器。
`run_sim(model_path=...)` 若显式指定模型路径，会同时应用到两个实例，模型需
兼容 Ultra 的关节、执行器命名及 SONIC 的 29 维关节映射。
策略权重仍是现有 SONIC 预训练权重；仿真可以运行不代表已验证 Ultra 真机适配。

这用于观察双臂 PD 跟踪效果，不包含全身平衡控制。
