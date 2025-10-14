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
```bash
roslaunch env_gazebo simple_scene.launch
```
这会启动一个带有简易机器人和雷达的仿真环境。

(4) 进行动态控制
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
调整机器人位置 相机云台 激光雷达云台