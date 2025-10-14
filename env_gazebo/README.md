# 雷达传感器仿真环境

这个包提供了完整的雷达传感器仿真环境，支持VLP-16和HDL-32E两种雷达类型。

**新增功能**：
- 在机体前方安装了独立的云台相机，可独立俯仰调节（与云台雷达分离）
- 相机默认朝前，初始下俯角约-20度，可通过dynamic_reconfigure调节
- 相机发布标准的ROS图像话题和相机内参话题

## 文件结构

```
env_gazebo/
├── cfg/
│   ├── gimbal_controllers.yaml        # 云台和相机控制器配置
│   ├── GimbalControl.cfg              # Dynamic reconfigure配置
│   └── RobotPose.cfg                  # 机器人位姿配置
├── launch/
│   ├── robot_spawn.launch             # 机器人生成启动文件
│   ├── simple_scene.launch            # 完整场景启动文件
│   └── simple_world.launch            # 基础世界启动文件
├── rviz/
│   └── lidar_visualization.rviz       # RViz配置文件
├── scripts/
│   ├── gazebo_tf_publisher.py         # TF发布器
│   └── gimbal_controller.py           # 云台和相机控制器
├── urdf/
│   ├── chassis.urdf.xacro             # 基础机器人chassis
│   ├── full_robot.urdf.xacro          # 完整机器人模型（云台+雷达+相机）
│   ├── sensor_HDL32E.xacro            # HDL-32E雷达URDF
│   ├── sensor_VLP16.xacro             # VLP-16雷达URDF  
│   ├── sensor_VLP_64.xacro            # VLP-64雷达URDF
│   └── robot_with_VLP.urdf.xacro      # 带VLP雷达的机器人
└── worlds/
    └── plane_with_objects.world       # 包含平面、瓶子、箱子的Gazebo世界
```

## 架构设计

### 模块化设计

**基础机器人** (`chassis.urdf.xacro`):
- 包含基础的base_link和前方标记
- 提供机器人的基础结构

**完整机器人模型** (`full_robot.urdf.xacro`):
- 集成云台、雷达、相机的完整机器人
- 云台支持yaw和pitch两轴运动，安装VLP-16/HDL-32E雷达
- 相机独立安装在机体前方，可单独俯仰调节
- 通过xacro参数选择雷达类型

**传感器组件**:
- `sensor_VLP16.xacro` / `sensor_HDL32E.xacro`: 对应雷达传感器定义
- 相机集成在主URDF中，包含Gazebo传感器插件

### Launch文件结构

**simple_scene.launch**:
- 完整的场景启动文件，包含世界、机器人、控制器
- 启动Gazebo世界、生成机器人、加载控制器配置
- 启动云台控制器脚本和TF发布器
- 可选启动RViz可视化

**robot_spawn.launch**:
- 负责机器人模型定义和在Gazebo中生成
- 支持传感器类型选择和生成位置配置

**simple_world.launch**:
- 仅启动Gazebo世界环境

### 控制系统

**控制器配置** (`gimbal_controllers.yaml`):
- `gimbal_yaw_position_controller`: 云台偏航轴控制
- `gimbal_pitch_position_controller`: 云台俯仰轴控制  
- `gimbal_camera_position_controller`: 相机俯仰轴控制
- `joint_state_controller`: 关节状态发布

**Dynamic Reconfigure** (`GimbalControl.cfg`):
- `lidar_gimbal_yaw`: 雷达云台偏航角度调节 (-180° 到 +180°)
- `lidar_gimbal_pitch`: 雷达云台俯仰角度调节 (-90° 到 +90°)
- `camera_gimbal_pitch`: 相机云台俯仰角度调节 (-90° 到 +90°)
- `robot_x/y/z`: 机器人在世界中的位置调节
- `robot_yaw`: 机器人在世界中的偏航角调节

**控制脚本** (`gimbal_controller.py`):
- 控制三个云台关节角度（雷达yaw/pitch + 相机pitch）
- 控制机器人在Gazebo世界中的位置和姿态
- 通过dynamic_reconfigure实时调节所有参数

### sensor_config.yaml

简化的雷达配置文件，只包含必要参数：

```yaml
lidar:
  sensor_type: vlp16           # 雷达选型: vlp16 或 hdl32e
  position:                    # 雷达位置和姿态
    x: 0.0                     # X坐标 (m)
    y: 0.0                     # Y坐标 (m)  
    z: 1.8                     # Z坐标 (m)
    roll: 0.0                  # 翻滚角 (rad)
    pitch: 0.0                 # 俯仰角 (rad)
    yaw: 0.0                   # 偏航角 (rad)
  parent_link: base_link       # 安装父链接
  topic_name: /velodyne_points # 点云话题名称
  frame_id: velodyne           # 坐标系ID
```

### 支持的传感器类型

**雷达传感器**:
- **vlp16**: Velodyne VLP-16 (16线激光雷达) - 安装在云台上可俯仰偏航
- **hdl32e**: Velodyne HDL-32E (32线激光雷达) - 安装在云台上可俯仰偏航

**相机传感器**:
- **gimbal_camera**: 云台相机，独立安装在机体前方
- 支持俯仰调节（-90°到+90°）
- 发布标准ROS图像话题和相机内参
- 默认分辨率640x480，视场角60度

### 配置说明

**机器人基座**:
- 尺寸: 0.2m x 0.2m x 0.1m（白色方块机体）
- 前方标记：红色细条，便于识别机体朝向
- 禁用重力：机器人在空中不会下落
- 默认生成高度：2.0m（避免与地面物体碰撞）

**云台雷达配置**:
- 云台安装在机体中心，支持±180度偏航，±90度俯仰
- 雷达安装在云台俯仰臂下方0.1m处
- 雷达可随云台进行全方位扫描

**独立相机配置**:
- 相机安装在机体前方0.15m，高度0.05m
- 独立俯仰关节，范围±90度，默认0度（水平朝前）
- 通过dynamic_reconfigure可实时调节俯仰角

## 使用方法

### 1. 启动完整场景（推荐）

```bash
# 使用默认VLP-16雷达
roslaunch env_gazebo simple_scene.launch

# 使用HDL-32E雷达
roslaunch env_gazebo simple_scene.launch sensor_type:=hdl32e

# 启动RViz可视化
roslaunch env_gazebo simple_scene.launch use_rviz:=true

# 自定义机器人在Gazebo世界中的生成位置
roslaunch env_gazebo simple_scene.launch spawn_x:=5.0 spawn_y:=3.0 spawn_yaw:=1.57

# 不显示Gazebo GUI
roslaunch env_gazebo simple_scene.launch gui:=false
```

**启动后发布的话题**：

*雷达话题*：
- `/velodyne_points`: 点云数据 (sensor_msgs/PointCloud2)

*相机话题*：
- `/gimbal_camera/image_raw`: 相机图像 (sensor_msgs/Image)  
- `/gimbal_camera/camera_info`: 相机内参 (sensor_msgs/CameraInfo)

*控制话题*：
- `/gimbal_yaw_position_controller/command`: 云台偏航控制 (std_msgs/Float64)
- `/gimbal_pitch_position_controller/command`: 云台俯仰控制 (std_msgs/Float64)  
- `/gimbal_camera_position_controller/command`: 相机俯仰控制 (std_msgs/Float64)

*TF坐标系*：
- `map -> base_link -> gimbal_base_link -> gimbal_pitch_link -> roLIDAR_Link -> velodyne`
- `base_link -> gimbal_camera_link -> gimbal_camera_optical_frame`

**查看相机图像**：
```bash
# 方式1: 使用rqt_image_view
rosrun rqt_image_view rqt_image_view

# 方式2: 查看话题列表
rostopic list | grep gimbal_camera

# 方式3: 查看相机信息
rostopic echo /gimbal_camera/camera_info
```

### 2. 使用dynamic_reconfigure实时控制

启动场景后，在另一个终端运行：
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

在rqt_reconfigure界面中可以找到：

**GimbalControl** - 云台和机器人控制：
- `lidar_gimbal_yaw`: 雷达云台偏航角 (-180° 到 +180°)
- `lidar_gimbal_pitch`: 雷达云台俯仰角 (-90° 到 +90°)  
- `camera_gimbal_pitch`: 相机云台俯仰角 (-90° 到 +90°)
- `robot_x/y/z`: 机器人在世界中的位置
- `robot_yaw`: 机器人在世界中的偏航角

**直接命令行控制（可选）**：
```bash
# 控制云台偏航到45度
rostopic pub /gimbal_yaw_position_controller/command std_msgs/Float64 "data: 0.785"

# 控制云台俯仰到-30度
rostopic pub /gimbal_pitch_position_controller/command std_msgs/Float64 "data: -0.524"

# 控制相机俯仰到-20度
rostopic pub /gimbal_camera_position_controller/command std_msgs/Float64 "data: -0.349"
```

### 3. 仅启动世界环境（不含机器人）
```bash
roslaunch env_gazebo simple_world.launch
```

### 4. 独立生成机器人（需要先启动Gazebo）
```bash
# 先启动Gazebo
roslaunch env_gazebo simple_world.launch

# 在另一个终端生成机器人
roslaunch env_gazebo robot_spawn.launch
```

## Launch文件参数

### simple_scene.launch 参数

**场景参数**:
- `world_name`: 世界文件名称 (默认: plane_with_objects)
- `gui`: 是否显示Gazebo GUI (默认: true)
- `use_rviz`: 是否启动RViz (默认: true)

**机器人生成位置参数**:
- `spawn_x`: 机器人在世界中的X位置 (默认: 0.0)
- `spawn_y`: 机器人在世界中的Y位置 (默认: 0.0)
- `spawn_z`: 机器人在世界中的Z位置 (默认: 2.0)
- `spawn_yaw`: 机器人在世界中的偏航角 (默认: 0.0)

**雷达配置参数**:
- `sensor_type`: 雷达类型 vlp16/hdl32e (默认: vlp16)
- `parent_link`: 雷达安装的父链接 (默认: base_link)
- `topic_name`: 点云话题名称 (默认: /velodyne_points)
- `frame_id`: 雷达坐标系ID (默认: velodyne)

**注意**: 
- 云台和相机的位置在URDF中已固定配置，无需外部参数
- 通过dynamic_reconfigure可实时调节云台和相机角度
- TF变换会自动根据spawn位置正确设置

## 话题和坐标系

### 发布的话题

**传感器数据**:
- `/velodyne_points`: 雷达点云数据 (sensor_msgs/PointCloud2)
- `/gimbal_camera/image_raw`: 相机图像 (sensor_msgs/Image)
- `/gimbal_camera/camera_info`: 相机内参 (sensor_msgs/CameraInfo)

**控制接口**:
- `/gimbal_yaw_position_controller/command`: 云台偏航控制 (std_msgs/Float64)
- `/gimbal_pitch_position_controller/command`: 云台俯仰控制 (std_msgs/Float64)
- `/gimbal_camera_position_controller/command`: 相机俯仰控制 (std_msgs/Float64)

**状态信息**:
- `/joint_states`: 关节状态信息 (sensor_msgs/JointState)
- `/tf`: 坐标变换信息

### 坐标系层级

**雷达相关坐标系**:
```
map
└── base_link
    └── gimbal_base_link (云台yaw轴)
        └── gimbal_pitch_link (云台pitch轴)
            └── roLIDAR_Link (雷达安装点)
                └── velodyne_base_link
                    └── velodyne (雷达扫描坐标系)
```

**相机相关坐标系**:
```
map  
└── base_link
    └── gimbal_camera_link (相机机身)
        └── gimbal_camera_optical_frame (ROS相机光学坐标系)
```

## 修改雷达配置

1. 编辑 `config/sensor_config.yaml` 文件
2. 修改以下参数：
   - `sensor_type`: 雷达类型 (vlp16/hdl32e)
   - `position`: 雷达位置和姿态 (x,y,z,roll,pitch,yaw)
   - `parent_link`: 安装父链接名称
   - `topic_name`: 点云发布话题名称
   - `frame_id`: 雷达坐标系ID
3. 重新启动launch文件

## 故障排除

### 常见问题

1. **找不到包**: 确保已经编译工作空间并source了setup.bash
2. **雷达不显示点云**: 检查Gazebo中是否有物体供雷达探测
3. **RViz显示异常**: 确保话题名称和坐标系配置正确
4. **控制器加载失败**: 重启Gazebo和控制器，确保没有重复加载

### 编译工作空间
```bash
cd ~/reqq_ws
catkin build
source devel/setup.bash
```

## 扩展使用

### 添加新的雷达类型
1. 在config/sensor_config.yaml中添加新雷达配置
2. 在configurable_lidar.urdf.xacro中添加对应的条件分支
3. 更新launch文件以支持新雷达类型

### 自定义世界文件
1. 在worlds/目录下创建新的.world文件
2. 使用world_name参数指定自定义世界文件