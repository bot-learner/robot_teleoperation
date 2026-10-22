# A3 Ultra 双臂碰撞对离线分析

## 修改结果

- `torso_Link` 的 17 个碰撞几何体合并为 2 个保守包围盒。
- 碰撞几何体组合由 1380 组降至 884 组。
- 修正 SRDF 中 3 个与 URDF 不一致的 link 名称。
- 对 SRDF 过滤后剩余的 136 组真实 link 组合进行离线姿态扫描。
- 其中 8 组髋部—肩部组合加入 SRDF，实时求解剩余 128 组 link 组合。

> 之前的“122 组”按关节父级做了合并，不是碰撞几何体实际所属 link 的数量；重新按 `parentFrame` 统计后是 136 组。

## 腰部碰撞体

原来 17 个几何体的整体边界为：

```text
x: -0.094 ～ 0.167 m
y: -0.180 ～ 0.180 m
z: -0.079 ～ 0.420 m
```

现在使用两个相互重叠的盒体覆盖该范围：

```text
          上盒体
       +----------+
       |          |
       |----------|  <- 重叠区，避免缝隙
       |          |
       +----------+
          下盒体
```

盒体比原始形状略大，因此会更早触发避碰，不会因为简化留下空洞。

## 离线扫描方法

执行命令：

```bash
python -m pico_teleop_interface.robot_interfaces.a3_ultra.analyze_collision_pairs \
  --samples 8192 --exclusion-margin 0.35
```

固定随机种子为 `20261018`，使用 Sobol 序列覆盖双臂 14 个关节的完整限位范围，并额外加入当前初始姿态。对采样中距离大于 0.35 m 的组合，再使用数值优化搜索更小距离。

加入 8 组 SRDF 排除项之前的结果如下：

```text
总姿态数：          8193
真实 link 组合：      136
观察到接触：           86
距离 0～0.10 m：       21
距离 0.10～0.35 m：    21
距离大于 0.35 m：       8
```

写入 SRDF 的 8 组组合，其搜索到的最小间距为：

| link 1 | link 2 | 最小距离 |
|---|---|---:|
| `left_hip_yaw_Link` | `left_shoulder_roll_Link` | 0.430 m |
| `right_hip_yaw_Link` | `right_shoulder_roll_Link` | 0.430 m |
| `left_hip_yaw_Link` | `right_shoulder_roll_Link` | 0.484 m |
| `left_shoulder_roll_Link` | `right_hip_yaw_Link` | 0.484 m |
| `left_hip_yaw_Link` | `left_shoulder_pitch_Link` | 0.511 m |
| `right_hip_yaw_Link` | `right_shoulder_pitch_Link` | 0.511 m |
| `left_hip_yaw_Link` | `right_shoulder_pitch_Link` | 0.546 m |
| `left_shoulder_pitch_Link` | `right_hip_yaw_Link` | 0.546 m |

离线采样不是形式化证明。更换 URDF、碰撞体或关节限位后，需要重新运行分析脚本；0.35 m 以下的组合仍保留在实时碰撞检查中。

IK 优化后，软碰撞激活距离由 0.10 m 扩大到 0.20 m。使用当前 SRDF 重新运行同一批采样的结果为：

```text
真实 link 组合：       128
观察到接触：            86
距离 0～0.20 m：        35
距离 0.20～0.35 m：      7
距离大于 0.35 m：        0
```
