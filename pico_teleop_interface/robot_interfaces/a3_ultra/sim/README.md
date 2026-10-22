# A3 Ultra MuJoCo resources

本目录保存 `sim.py` 运行所需的 A3 Ultra T3D0 MJCF、网格和地面场景。

资源来自仓库根目录的加密包：

```text
a3_ultra/low_level_motion_control.zip
└── low_level_motion_control/aimrt_mujoco_sim_a3/src/models/bin/cfg/model/
    ├── a3_ultra_t3d0/{mjcf,meshes}
    └── terrain/mujoco/flat.xml
```

这里只保留 Python MuJoCo 运动学预览需要的模型资源；没有复制 AimRT、ROS2、
低层控制器、ONNX 模型或 C++ 仿真框架。
