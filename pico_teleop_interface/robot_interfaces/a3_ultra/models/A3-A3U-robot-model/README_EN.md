# A3 / A3 Ultra Robot Models

[中文说明](./README.md)

This repository contains model assets for the A3 humanoid robot family. It includes multiple robot descriptions and visualization resources that are suitable for:

- `URDF` robot description in ROS / ROS 2
- `MJCF` model loading in MuJoCo
- RViz visualization
- Motion planning integration such as MoveIt (`SRDF` is provided for `A3 Ultra` variants)

The repository mainly contains static model assets. It does not include controllers, hardware drivers, launch files, or a complete workspace setup.

## Contents

The repository currently includes 4 model variants:

| Directory | Description | Main files |
| --- | --- | --- |
| `a3_t2d5/` | Base A3 T2.5 model | `urdf/model.urdf`, `mjcf/a3_t2d5.xml`, `rviz/display.rviz` |
| `a3_t3d0/` | Base A3 T3.0 model | `urdf/model.urdf`, `mjcf/a3_t3d0.xml`, `rviz/display.rviz` |
| `a3_ultra_t2d5/` | A3 Ultra T2.5 | `urdf/model.urdf`, `urdf/model_planning.urdf`, `urdf/model_planning.srdf`, `mjcf/a3_ultra_t2d5.xml` |
| `a3_ultra_t3d0/` | A3 Ultra T3.0 with dexterous-hand-related assets | `urdf/model.urdf`, `urdf/model_planning.urdf`, `urdf/model_planning.srdf`, `mjcf/a3_ultra_t3d0.xml` |

Each model directory usually contains:

- `meshes/`: STL mesh assets
- `urdf/`: URDF robot descriptions
- `mjcf/`: MuJoCo XML models
- `rviz/`: RViz display configuration
