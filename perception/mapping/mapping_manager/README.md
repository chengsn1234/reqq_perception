# Mapping Manager

这个包管理elevation mapping在不同场景下的配置和启动。

## 目录结构

```
mapping_manager/
├── config/
│   ├── robots/
│   │   └── simple_scene_robot.yaml          # 机器人配置参数
│   └── postprocessing/
│       └── postprocessor_pipeline.yaml      # 后处理流水线配置
├── launch/
│   └── simple_scene_elevation_mapping.launch # 主启动文件
├── rviz/
│   └── simple_scene_elevation_mapping.rviz   # RViz可视化配置
└── README.md
```

## 使用方法

### 启动simple scene elevation mapping

```bash
# 切换到工作空间
cd /home/chengsn/reqq_ws

# 编译工作空间
catkin_make

# 设置环境变量
source devel/setup.bash

# 启动elevation mapping演示
roslaunch mapping_manager simple_scene_elevation_mapping.launch
```

### 可选参数

```bash
# 指定雷达类型（vlp16 或 hdl32e）
roslaunch mapping_manager simple_scene_elevation_mapping.launch sensor_type:=hdl32e

# 指定机器人初始位置
roslaunch mapping_manager simple_scene_elevation_mapping.launch spawn_x:=5.0 spawn_y:=3.0 spawn_z:=2.0

# 不启动RViz
roslaunch mapping_manager simple_scene_elevation_mapping.launch use_rviz:=false
```

## 配置说明

### 机器人配置 (simple_scene_robot.yaml)

- `map_frame_id`: 地图坐标系 (map)
- `robot_base_frame_id`: 机器人基座坐标系 (base_link)
- `input_sources`: 点云输入源配置
  - `topic`: 点云话题 (/velodyne_points_downsampled)
  - `sensor_processor`: 传感器处理器类型
- `length_in_x/y`: 地图尺寸 (20m x 20m)
- `resolution`: 地图分辨率 (0.1m)

### 后处理配置 (postprocessor_pipeline.yaml)

- `inpaint`: 填补地图空洞
- `surface_normals`: 计算表面法向量
- `slope`: 计算坡度图
- `traversability`: 可通行性分析

## 主要功能

1. **环境仿真**: 自动启动simple_scene Gazebo环境
2. **点云处理**: 对Velodyne点云进行下采样滤波
3. **elevation mapping**: 生成实时高程地图
4. **后处理**: 进行表面法向量计算、坡度分析等
5. **可视化**: 在RViz中显示机器人、点云和elevation map

## 依赖包

- elevation_mapping
- elevation_mapping_demos
- env_gazebo
- pcl_ros
- nodelet
- rviz

## 注意事项

1. 确保所有依赖包已正确编译
2. 启动前确保没有其他Gazebo实例正在运行
3. 如果遇到问题，可以查看各节点的日志输出