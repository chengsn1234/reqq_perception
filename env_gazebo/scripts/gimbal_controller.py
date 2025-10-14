#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from env_gazebo.cfg import GimbalControlConfig

class GimbalController:
    def __init__(self):
        rospy.init_node('gimbal_controller', anonymous=False)
        
        # 云台关节控制发布器
        self.lidar_yaw_pub = rospy.Publisher('/gimbal_yaw_position_controller/command', Float64, queue_size=1)
        self.lidar_pitch_pub = rospy.Publisher('/gimbal_pitch_position_controller/command', Float64, queue_size=1)
        # 相机俯仰控制发布器
        self.camera_pitch_pub = rospy.Publisher('/gimbal_camera_position_controller/command', Float64, queue_size=1)
        
        # Gazebo 模型状态设置服务
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 当前状态
        self.current_lidar_yaw = 0.0
        self.current_lidar_pitch = 0.0
        self.current_camera_pitch = -0.3491  # 初始相机下俯角约-20度
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 2.0
        self.current_robot_yaw = 0.0
        
        # 动态参数配置服务器
        self.server = Server(GimbalControlConfig, self.config_callback)
        
        rospy.loginfo("Gimbal Controller started. Use rqt_reconfigure to control lidar/camera gimbals and robot position.")
        
    def update_robot_pose(self, x, y, z, yaw):
        """更新 Gazebo 中机器人的位姿"""
        try:
            model_state = ModelState()
            model_state.model_name = 'robot'
            model_state.pose = Pose()
            model_state.pose.position = Point(x, y, z)
            
            # 转换欧拉角到四元数
            quaternion = quaternion_from_euler(0, 0, yaw)
            model_state.pose.orientation = Quaternion(*quaternion)
            
            # 设置参考坐标系
            model_state.reference_frame = 'world'
            
            # 调用服务
            resp = self.set_model_state(model_state)
            if not resp.success:
                rospy.logwarn("Failed to set model state: %s", resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Gimbal controller interrupted")
        except KeyboardInterrupt:
            rospy.loginfo("Gimbal controller stopped by user")
        finally:
            rospy.loginfo("Gimbal controller shutdown complete")
        
    def signal_handler(self, signum, frame):
        """优雅处理关闭信号"""
        rospy.loginfo("Received shutdown signal, stopping gimbal controller...")
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)
        
    def config_callback(self, config, level):
        """动态参数配置回调"""
        # 转换角度到弧度
        lidar_yaw_rad = math.radians(config.lidar_gimbal_yaw)
        lidar_pitch_rad = math.radians(config.lidar_gimbal_pitch)
        camera_pitch_rad = math.radians(config.camera_gimbal_pitch)
        robot_yaw_rad = math.radians(config.robot_yaw)
        
        # 发布雷达云台关节角度
        if abs(lidar_yaw_rad - self.current_lidar_yaw) > 0.01:
            self.lidar_yaw_pub.publish(Float64(lidar_yaw_rad))
            self.current_lidar_yaw = lidar_yaw_rad
            rospy.loginfo("Lidar Gimbal Yaw: %.2f degrees", config.lidar_gimbal_yaw)
            
        if abs(lidar_pitch_rad - self.current_lidar_pitch) > 0.01:
            self.lidar_pitch_pub.publish(Float64(lidar_pitch_rad))
            self.current_lidar_pitch = lidar_pitch_rad
            rospy.loginfo("Lidar Gimbal Pitch: %.2f degrees", config.lidar_gimbal_pitch)

        # 发布相机俯仰角度
        if abs(camera_pitch_rad - self.current_camera_pitch) > 0.01:
            self.camera_pitch_pub.publish(Float64(camera_pitch_rad))
            self.current_camera_pitch = camera_pitch_rad
            rospy.loginfo("Camera Gimbal Pitch: %.2f degrees", config.camera_gimbal_pitch)
        
        # 更新机器人位置
        if (abs(config.robot_x - self.current_x) > 0.01 or 
            abs(config.robot_y - self.current_y) > 0.01 or 
            abs(config.robot_z - self.current_z) > 0.01 or
            abs(robot_yaw_rad - self.current_robot_yaw) > 0.01):
            
            self.update_robot_pose(config.robot_x, config.robot_y, config.robot_z, robot_yaw_rad)
            self.current_x = config.robot_x
            self.current_y = config.robot_y
            self.current_z = config.robot_z
            self.current_robot_yaw = robot_yaw_rad
            rospy.loginfo("Robot Position: (%.2f, %.2f, %.2f), Yaw: %.2f degrees", 
                         config.robot_x, config.robot_y, config.robot_z, config.robot_yaw)
        
        return config
    
    def update_robot_pose(self, x, y, z, yaw):
        """更新 Gazebo 中机器人的位姿"""
        try:
            model_state = ModelState()
            model_state.model_name = 'robot'
            model_state.pose = Pose()
            model_state.pose.position = Point(x, y, z)
            
            # 转换欧拉角到四元数
            quaternion = quaternion_from_euler(0, 0, yaw)
            model_state.pose.orientation = Quaternion(*quaternion)
            
            # 设置参考坐标系
            model_state.reference_frame = 'world'
            
            # 调用服务
            resp = self.set_model_state(model_state)
            if not resp.success:
                rospy.logwarn("Failed to set model state: %s", resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = GimbalController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
