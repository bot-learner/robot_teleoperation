# A3 / A3 Ultra Robot Models

[English](./README_EN.md)

本仓库提供 A3 系列人形机器人模型资源，包含多套机器人描述与可视化文件，适合用于：

- ROS / ROS 2 中的 `URDF` 机器人描述与可视化
- MuJoCo 中的 `MJCF` 仿真模型加载
- RViz 模型显示
- MoveIt 等运动规划场景中的语义模型加载（`A3 Ultra` 版本提供 `SRDF`）

仓库内容以静态模型资源为主，不包含控制器、驱动、启动脚本或完整工作区配置。

## 仓库内容

仓库当前包含 4 套模型：

| 目录 | 说明 | 主要文件 |
| --- | --- | --- |
| `a3_t2d5/` | A3 T2.5 基础版本 | `urdf/model.urdf`, `mjcf/a3_t2d5.xml`, `rviz/display.rviz` |
| `a3_t3d0/` | A3 T3.0 基础版本 | `urdf/model.urdf`, `mjcf/a3_t3d0.xml`, `rviz/display.rviz` |
| `a3_ultra_t2d5/` | A3 Ultra T2.5 | `urdf/model.urdf`, `urdf/model_planning.urdf`, `urdf/model_planning.srdf`, `mjcf/a3_ultra_t2d5.xml` |
| `a3_ultra_t3d0/` | A3 Ultra T3.0，带灵巧手相关模型 | `urdf/model.urdf`, `urdf/model_planning.urdf`, `urdf/model_planning.srdf`, `mjcf/a3_ultra_t3d0.xml` |

每个模型目录通常包含以下子目录：

- `meshes/`: STL 网格模型
- `urdf/`: URDF 机器人描述
- `mjcf/`: MuJoCo XML 模型
- `rviz/`: RViz 显示配置


