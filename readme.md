### 安装配置
(1) 下载代码后运行catkin build
```bash
git clone ***.git
catkin build
```

(2) source工作空间
```bash
source {Workspace}/devel/setup.bash
```

(3) 运行demo场景
启动带有简易机器人的仿真环境与高度建图模块：
```bash
roslaunch mapping_manager simple_scene_elevation_mapping.launch
```
在Rviz上点击Publish Point并点击高度图，终端会发布Rviz显示高度，机器人高度和相对高度差。


(4) 进行动态控制
调整机器人位置，相机云台，激光雷达云台：
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
