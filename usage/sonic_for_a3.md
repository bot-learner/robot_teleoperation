# 尝试 SONIC for A3

## 直接使用 a3_teleop（2026-10-18）

现在已在本机 `a3_teleop` 中补齐 CPU 推理依赖，不必切换 `.venv_cpu`。
此前默认入口改成 SONIC，但 PyTorch 只安装在独立 venv 中，所以原环境会报缺少 `torch`。
`usage/setup_conda_env.sh` 也已加入以下安装命令：

```bash
conda activate a3_teleop
python -m pip install 'torch==2.14.1+cpu' --index-url https://download.pytorch.org/whl/cpu
python -m pip install 'scipy==1.17.1' 'PyYAML==6.0.3'
```

在项目根目录启动：

```bash
python pico_teleop_interface/robot_interfaces/a3_ultra/a3_ultra_interface.py
```

VS Code 选择 `/home/aheidabai/miniconda3/envs/a3_teleop/bin/python` 即可。
先用模拟数据验证（不连接 PICO、不打开窗口）：

```bash
python -m pico_teleop_interface.robot_interfaces.a3_ultra.sonic_sim --mock --headless --steps 100
```

以下章节保留早期独立 venv 的搭建和测试记录；其中 `.venv_cpu/bin/python`
命令也可在激活 `a3_teleop` 后替换成 `python`。源码、权重和闭链求解器仍需准备好。

## 本机验证结果（2026-10-16）

已完成源码检出、所需 Git LFS 模型/闭链求解器下载、隔离 CPU 环境安装，
以及官方 PT 的大小与 SHA-256 校验。未修改上游控制代码，也未连接真机或 PICO。
本机 NVIDIA 驱动不可用，本次使用 CPU PyTorch 推理。

已通过 10 步短测试，并跑完整个 `001_walk_front_slow.csv`：

| 项目 | 结果 |
| --- | --- |
| 策略步数 / 仿真时长 | 1652 / 约 33.04 秒 |
| 官方跌倒启发式 `fall` | `false` |
| 29 关节跟踪 RMSE | 0.06631 rad |
| 局部人体参考点 MPJPE（`mpjpe_l`） | 15.49 mm |
| 根部位置平均 / 最大误差 | 0.1765 / 0.4966 m |

结果文件：源码目录下 `.cache/sonic_cpu_smoke_metrics.json` 和
`.cache/sonic_cpu_walk_metrics.json`。本次是零执行器延迟、无窗口测试，
未检测到跌倒不等于长期稳定或真机可用；根部位置存在明显漂移。
策略时钟设置为 50 Hz，不表示已测得 CPU 实时性能保证。

源码：`pico_teleop_interface/third_party/sonic_for_a3/`。
本次检出提交：`c9df753d31ef63c930b825b1b6767cabd37c1549`。

先使用官方预训练策略和配套模型运行离线动作跟踪，再考虑接入 PICO。
它控制 29 个全身关节，不能直接替换当前固定身体的双臂 IK。
官方说明策略兼容 A3 Ultra，但公开 sim2sim 使用 A3 配套的被动脚模型；
这里不是 Ultra T3D0 专用模型验证。当前发布的模型仍需上游实时重定向，且手腕跟踪有限。

## 1. 准备（本机已完成源码和 CPU 环境安装）

从项目根目录开始，进入源码：

```bash
cd pico_teleop_interface/third_party/sonic_for_a3
```

本机使用 `.venv_cpu/bin/python`，Python 3.11，CPU PyTorch。
该 venv 通过 `--system-site-packages` 复用 `a3_teleop` 中的 MuJoCo / NumPy，
新增包安装在 venv 内；未向原 conda 环境安装 SONIC 依赖。
它依赖当前机器的原环境路径，不是可直接复制到另一台机器的环境。

新机器可在检出仓库后按下面步骤创建：

```bash
git lfs pull --include='gear_sonic/data/assets/robot_description/**' --exclude=''
python3.11 -m venv .venv_cpu
.venv_cpu/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
.venv_cpu/bin/python -m pip install numpy scipy pyyaml mujoco huggingface_hub
```

闭链求解器的桥接编译还需要系统已有 `g++` 和 Eigen 头文件。
仿真测试不需要安装完整 Isaac Lab 训练环境，也不要求可用 NVIDIA GPU。

## 2. 下载官方 PT（约 402 MB，脚本校验大小和 SHA-256）

```bash
.venv_cpu/bin/python download_from_hf.py --component pt
```

只下载 PT 组即可进行本次实验，不需要 RKNN、交叉编译 sysroot 或重新训练。

## 3. 无窗口测试

仍在 SONIC 仓库根目录执行：

```bash
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv_cpu/bin/python \
  gear_sonic/scripts/sim2sim_a3_mujoco.py \
  --checkpoint checkpoints/035_step200000/model_step_200000.pt \
  --motion a3_data/agibot_a3/001_walk_front_slow.csv \
  --encoder-mode a3_fast \
  --batch-once \
  --max-policy-steps 10 \
  --metrics-out .cache/sonic_cpu_smoke_metrics.json
```

不加 `--output-video`，避免引入离屏图形渲染依赖。
短测试只检查加载、推理和物理步进，不足以证明长期稳定跟踪。
去掉 `--max-policy-steps 10` 可运行完整动作；将输出文件改为
`.cache/sonic_cpu_walk_metrics.json` 可保存完整动作指标。

## 4. 打开窗口观察

```bash
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv_cpu/bin/python \
  gear_sonic/scripts/sim2sim_a3_mujoco.py \
  --checkpoint checkpoints/035_step200000/model_step_200000.pt \
  --motion a3_data/agibot_a3/001_walk_front_slow.csv \
  --encoder-mode a3_fast
```

空格暂停，`R` 重置，关闭窗口退出。不要直接换成现有双臂仿真的 MJCF，
预训练策略的关节顺序、执行器和脚部动力学必须与训练配置匹配。

官方依据：[README](https://github.com/AgibotTech/sonic_for_a3)、
[Training → Sim2Sim → Deploy](https://github.com/AgibotTech/sonic_for_a3/blob/main/docs/a3_training2sim2deploy.md)。

## 5. 本项目的 SONIC 双臂实验入口

新增代码：`pico_teleop_interface/robot_interfaces/a3_ultra/sonic_sim.py`。
以下命令都从**本项目根目录**运行，不是在 SONIC 仓库内运行。

先用模拟 Tracker 数据测试（不需要头显或 PC Service）：

```bash
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  pico_teleop_interface/third_party/sonic_for_a3/.venv_cpu/bin/python \
  -m pico_teleop_interface.robot_interfaces.a3_ultra.sonic_sim \
  --mock --headless --steps 500 \
  --metrics pico_teleop_interface/third_party/sonic_for_a3/.cache/sonic_upper_mock_metrics.json
```

本机 500 步 / 10 秒模拟 Tracker 前伸测试：数值警告为 0，双臂参考关节角
RMSE 为 `0.08691 rad`（约 5.0°），最大绝对误差 `0.15762 rad`（约 9.0°）。
这些是策略执行后的实际关节角相对 IK 参考的误差，不是手掌空间误差。
固定身体运行的策略并非专门训练的双臂策略，不能据此断言优于原 IK + PD。

打开窗口观察模拟输入：

```bash
pico_teleop_interface/third_party/sonic_for_a3/.venv_cpu/bin/python \
  -m pico_teleop_interface.robot_interfaces.a3_ultra.sonic_sim --mock --steps 500
```

连接真实 PICO：在头显启用两个 Tracker 的 Object Tracking 和 Send，然后执行：

```bash
pico_teleop_interface/third_party/sonic_for_a3/.venv_cpu/bin/python \
  -m pico_teleop_interface.robot_interfaces.a3_ultra.sonic_sim \
  --left-tracker-serial <左侧序列号> \
  --right-tracker-serial <右侧序列号>
```

序列号可省略，由首帧头显水平朝向校正后的左右位置绑定。PICO 同时开启
`Head`、`Object Tracking` 和 `Send`；倒计时 6 秒内正视身体前方，双臂向前、
肘部微弯。首个有效头显及双 Tracker 帧标定水平朝向和零点，之后转头不改变
映射，短时断连也不会重新标定。
实时 PICO 联动尚未验证，启动命令可能调用现有脚本启动 PC Service。

数据链路：Tracker → 现有滤波 / 相对标定 / 双臂 IK → 补齐 29 关节参考
→ 官方 640 维编码输入 + 930 维实际状态历史 → A3-fast 策略
→ 只取双臂输出 → 官方 PD 增益及力矩限幅 → MuJoCo。
动作缩放、观测排列、历史缓存直接复用上游实现；不要将动作原值当成关节角。

实时参考采用未来 10 帧保持最新目标、速度为零，不进行动作预测。
腰、腿、头、脚部被动铰链用约束固定；闭链连杆的球关节保留原约束。
身体执行器命令清零，观测中的上一帧动作也只保留实际发送的双臂动作。
本项目的实时入口使用 `sim/model/mjcf/a3_ultra_t3d0.xml`。两个实例复用 Ultra
的初始化、身体约束、PD 增益和力矩限幅，执行纯 PD，不额外叠加 `qfrc_bias`。
SONIC 观测按 checkpoint 所需的 29 关节顺序，从 Ultra 的实际 `qpos/qvel`
读取；骨盆角速度和重力方向也从 Ultra 读取，不经过 A3 T2.5 闭链电机换算。
IK 参考的避碰不能保证策略输出及实际执行轨迹无碰撞；本入口仅用于仿真。

`a3_ultra_interface.py` 的 `__main__` 现在调用 `run_sim()`，同时打开
`IK + PD` 和 `SONIC` 两个独立进程的 MuJoCo 窗口。两者加载同一个 Ultra
T3D0 模型，具有相同的初始状态、身体约束、PD 参数及物理步长，并接收同一份
最新 IK 目标。左侧直接 IK → PD，右侧 IK → SONIC → PD；队列只保留
最新结果，不会积压历史帧。策略权重未经本项目重新训练，仍需验证实际跟踪质量；
仿真能运行不代表已验证 Ultra 真机控制。
在 VS Code 中直接运行该文件时，可选择已补齐依赖的 `a3_teleop/bin/python`，
或上述 `.venv_cpu/bin/python`。
如只需 SONIC，可在 Python 中显式调用 `robot.run_sonic_sim()`。
