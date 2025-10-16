# Mapping Dependencies Guide

这个目录包含了elevation mapping相关的依赖库，使用Git submodules管理。

## Git Submodules依赖库

本项目使用Git submodules来管理以下第三方依赖：

- **elevation_mapping**: ANYbotics开源的elevation mapping库
- **kindr**: 运动学和动力学表示库  
- **kindr_ros**: kindr的ROS接口库
- **message_logger**: 消息日志库

## 初始化和更新子模块

### 首次克隆仓库后，初始化所有子模块：

```bash
cd reqq_perception
git submodule init
git submodule update
```

### 或者一次性克隆包含子模块的仓库：

```bash
git clone --recursive https://github.com/chengsn1234/reqq_perception.git
```

### 更新子模块到最新版本：

```bash
git submodule update --remote
```

## 编译

初始化子模块后，回到工作空间根目录编译：

```bash
cd ~/reqq_ws
catkin_make
source devel/setup.bash
```