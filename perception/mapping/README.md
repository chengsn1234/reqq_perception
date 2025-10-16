# Mapping Dependencies Installation Guide

这个目录包含了elevation mapping相关的依赖库。为了保持仓库轻量，第三方库不包含在版本控制中，需要单独克隆。

## 第三方依赖库

### 1. elevation_mapping
```bash
cd perception/mapping/
git clone https://github.com/ANYbotics/elevation_mapping.git
```

### 2. kindr
```bash
cd perception/mapping/
git clone https://github.com/ANYbotics/kindr.git
```

### 3. kindr_ros
```bash
cd perception/mapping/
git clone https://github.com/ANYbotics/kindr_ros.git
```

### 4. message_logger
```bash
cd perception/mapping/
git clone https://github.com/ANYbotics/message_logger.git
```

## 一键安装脚本

你可以运行以下命令来安装所有依赖：

```bash
cd perception/mapping/
git clone https://github.com/ANYbotics/elevation_mapping.git
git clone https://github.com/ANYbotics/kindr.git
git clone https://github.com/ANYbotics/kindr_ros.git
git clone https://github.com/ANYbotics/message_logger.git
```

## 编译

安装完所有依赖后，回到工作空间根目录编译：

```bash
cd ~/reqq_ws
catkin_make
source devel/setup.bash
```