"""A3 Ultra 双臂遥操中需要反复调整的参数。"""

# 主控循环频率；50 Hz 对应每帧 20 ms。
CONTROL_RATE_HZ = 50.0

# PICO 位置和姿态二阶低通滤波器的截止频率。
LOW_PASS_CUTOFF_HZ = 10.0

# 是否默认显示右下角的遥操帧率和状态监视窗口。
SHOW_MONITOR = True

# 观察到的 PICO 新数据帧率低于该值时标红；重复缓存不计数，且受主循环采样速度限制。
MONITOR_WARNING_FPS = 25.0

# 是否将程序生命周期、跟踪状态、数据快照、异常和性能指标写入文件。
ENABLE_FILE_LOG = True

# 日志目录；None 表示使用 ~/.local/state/pico_teleop/。
LOG_DIRECTORY = None

# 文件日志等级；INFO 记录必要运行事件和性能指标。
LOG_LEVEL = "INFO"

# 每隔多少秒记录一次帧率、Tracker 数量和 IK 耗时。
LOG_METRICS_INTERVAL_SECONDS = 1.0

# 每隔多少秒记录一次 PICO Tracker 数据和双臂 IK 目标关节角。
LOG_DATA_INTERVAL_SECONDS = 1.0

# 单个日志文件最大 40 MiB；当前文件加 4 个备份合计约 200 MiB。
LOG_MAX_BYTES = 40 * 1024 * 1024

# 轮转时保留的历史日志文件数量。
LOG_BACKUP_COUNT = 4

# 人手平移到机器人掌根平移的比例；1.0 表示等尺度映射。
POSITION_SCALE = 1.0

# 位置变化小于该值时保持上一目标，用于抑制 Tracker 静止抖动。
POSITION_DEADBAND_M = 0.002

# 旋转变化小于该角度时保持上一目标。
ROTATION_DEADBAND_DEG = 0.5

# Tracker 数据不可用或设备时间戳停止推进超过该时间后恢复时，重置滤波历史。
# IK 或仿真计算变慢不会触发此条件。
RECONNECT_JUMP_AFTER_SECONDS = 1.0

# 每帧 IK 允许的最大迭代次数；越大越容易收敛，但最坏耗时更长。
IK_MAX_ITERATIONS = 100

# 单帧 IK 的软时间上限，单位为 ms；到时返回已经找到的最佳安全姿态。
IK_MAX_SOLVE_TIME_MS = 15.0

# 任一掌根的位置误差都不超过该值时，位置部分视为收敛，单位为 m。
IK_POSITION_TOLERANCE_M = 0.001

# 任一掌根的旋转误差都不超过该值时，旋转部分视为收敛，单位为度。
IK_ROTATION_TOLERANCE_DEG = 0.1

# 阻尼最小值，用于抑制奇异位形附近的数值放大。
IK_DAMPING = 1e-6

# 每次 IK 迭代首先尝试的速度比例；失败时求解器会逐次减半回退。
IK_INTEGRATION_STEP = 0.4

# 单次 IK 内部迭代的最大关节变化量，单位为 rad；它不是帧间速度限制。
IK_MAX_JOINT_STEP_RAD = 0.2

# 默认双臂姿态代价权重，用于稳定肘部弯曲分支。
IK_ELBOW_PREFERENCE_WEIGHT = 0.2

# 自碰撞软代价权重；越大时 IK 越倾向远离障碍。
IK_SELF_COLLISION_WEIGHT = 100.0

# 靠近关节上下限时的软代价权重。
IK_JOINT_LIMIT_WEIGHT = 0.05

# 相邻控制帧间保持上一关节姿态的软代价权重。
IK_CONTINUITY_WEIGHT = 0.05

# 最小碰撞距离低于该值时开始计算避碰梯度，单位为 m。
COLLISION_ACTIVATION_DISTANCE_M = 0.20

# IK 回溯不允许继续侵入的碰撞硬边界，单位为 m。
COLLISION_MIN_DISTANCE_M = 0.02

# 低于该距离时每次 IK 迭代都刷新碰撞梯度，保证危险区内及时避让。
COLLISION_DANGER_DISTANCE_M = 0.10

# 碰撞对低于该距离时进入活跃集合；只为集合中的最近碰撞对计算梯度。
COLLISION_ACTIVE_ENTER_DISTANCE_M = 0.15

# 活跃碰撞对超过该距离后才移出集合，避免在边界附近频繁切换。
COLLISION_ACTIVE_EXIT_DISTANCE_M = 0.20

# 非危险区每隔多少次 IK 迭代刷新一次碰撞梯度，中间复用上次结果。
COLLISION_GRADIENT_REFRESH_INTERVAL = 3

# 0.10～0.20 m 预警区的避碰强度比例；危险区内仍使用完整强度。
COLLISION_WARNING_WEIGHT_SCALE = 0.05

# 精确碰撞前的保守包围球预检余量；用于覆盖浮点误差，单位为 m。
COLLISION_BROADPHASE_MARGIN_M = 0.01

# 掌根目标到对应肩关节的最小允许距离，单位为 m。
IK_WORKSPACE_MIN_RADIUS_M = 0.08

# 掌根目标到对应肩关节的最大允许距离，略小于机械臂完全伸直长度。
IK_WORKSPACE_MAX_RADIUS_M = 0.60

# PC 使用标准 JointState；HDU 转发状态，MDU 将指令转换为 Iceoryx joint_msgs。
REAL_ROBOT_COMMAND_TOPIC = "/teleop/arm_joint_command"
REAL_ROBOT_STATE_TOPIC = "/teleop/arm_joint_state"
# MDU 已启用指令且 Iceoryx 状态正常时才发布 True；禁发测试模式始终为 False。
REAL_ROBOT_BRIDGE_READY_TOPIC = "/teleop/arm_bridge_ready"
# MDU 心跳超过该时间时监视窗标红；当前不据此中止运动，单位秒。
REAL_ROBOT_BRIDGE_READY_TIMEOUT_SECONDS = 1.0

# A3 Ultra 机器人 ROS 2 网络使用的固定 Domain ID。
REAL_ROBOT_ROS_DOMAIN_ID = 232

# 机器人 DDS XML 使用 Fast DDS，不允许 ROS 自动选择其他 RMW。
REAL_ROBOT_RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# 第三方 PC 有线直连时，从 HDU 复制 ros_dds_configuration.xml 后在此填写路径。
# 若为 None，则沿用启动程序前设置的 FASTRTPS_DEFAULT_PROFILES_FILE。
REAL_ROBOT_DDS_PROFILE = None

# 底层双臂目标发送频率。
REAL_ROBOT_COMMAND_RATE_HZ = 100.0

# 实时 IK 目标超过该时间不更新就保持最后发送角度；机器人反馈超时另行检查。
REAL_ROBOT_STREAM_TIMEOUT_SECONDS = 0.2

# PICO 实机实时控制的关节速度上限，单位 rad/s。
REAL_ROBOT_TELEOP_SPEED_RAD_S = 0.5

# 实机规划路径执行时的关节速度上限；GUI 测试另设为 0.2 rad/s。
REAL_ROBOT_MAX_JOINT_SPEED_RAD_S = 0.5

# 真机关节的启动加速度上限；默认用 1 秒从静止增加到 0.5 rad/s。
REAL_ROBOT_MAX_JOINT_ACCELERATION_RAD_S2 = 0.5

# 启动桥接后等待第一帧真机反馈的最长时间。
REAL_ROBOT_CONNECT_TIMEOUT_SECONDS = 5.0

# PC 反馈年龄超过该值时监视窗标红；当前不据此中止运动，单位秒。
REAL_ROBOT_STATE_TIMEOUT_SECONDS = 0.25

# 离散目标的路径规划超时，含贴身脱离阶段，单位秒。
REAL_ROBOT_PLANNING_TIMEOUT_SECONDS = 10.0
# 规划期间反馈相对起点最多允许变化的角度；超过后拒绝执行旧路径，单位 rad。
REAL_ROBOT_PLANNING_START_TOLERANCE_RAD = 0.01

# 默认姿态归位：所有双臂关节与目标的误差均小于 25 度时才开始观察稳定性。
REAL_ROBOT_HOME_TOLERANCE_RAD = 25.0 * 3.141592653589793 / 180.0
# 默认归位需要连续稳定的时间，单位秒。
REAL_ROBOT_HOME_SETTLE_SECONDS = 2.0
# 稳定观察期间，各关节相对观察起点最多变化 1 度；超过则重新计时。
REAL_ROBOT_HOME_STABLE_RANGE_RAD = 1.0 * 3.141592653589793 / 180.0

