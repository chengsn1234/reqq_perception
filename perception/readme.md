### ETH elevation mapping安装配置
(1)安装grid_map
link: https://github.com/anybotics/grid_map
```bash
sudo apt-get install ros-$ROS_DISTRO-grid-map
```
(2)下载代码并编译
```bash
cd catkin_workspace/src
git clone https://github.com/anybotics/kindr.git
git clone https://github.com/anybotics/kindr_ros.git
git clone https://github.com/ANYbotics/message_logger.git
git clone https://github.com/anybotics/elevation_mapping.git
cd ../
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
(3)从mapping_manager启动mapping节点和仿真节点
```bash
roslaunch mapping_manager simple_scene_elevation_mapping.launch 
```