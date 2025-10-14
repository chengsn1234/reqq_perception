#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

class GazeboTFPublisher:
    def __init__(self):
        rospy.init_node('gazebo_tf_publisher', anonymous=False)
        
        # TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 机器人模型名称
        self.robot_model_name = rospy.get_param('~robot_model_name', 'robot')
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # 用于跟踪上次发布的时间戳，避免重复
        self.last_time = rospy.Time(0)
        
        # 订阅 Gazebo 模型状态
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        
        rospy.loginfo("Gazebo TF Publisher started. Publishing TF for model: %s", self.robot_model_name)
        rospy.loginfo("TF chain: %s -> %s -> %s", self.world_frame, self.odom_frame, self.base_frame)
    
    def model_states_callback(self, msg):
        try:
            # 查找机器人模型的索引
            if self.robot_model_name not in msg.name:
                return
            
            index = msg.name.index(self.robot_model_name)
            pose = msg.pose[index]
            
            # 获取当前仿真时间
            current_time = rospy.Time.now()
            
            # 避免发布相同时间戳的TF
            if current_time <= self.last_time:
                return
            
            self.last_time = current_time
            
            # 发布 map -> odom (固定为单位变换)
            self.tf_broadcaster.sendTransform(
                (0, 0, 0),
                (0, 0, 0, 1),
                current_time,
                self.odom_frame,
                self.world_frame
            )
            
            # 发布 odom -> base_link (从 Gazebo 获取实际位姿)
            self.tf_broadcaster.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                current_time,
                self.base_frame,
                self.odom_frame
            )
            
        except ValueError:
            rospy.logwarn_throttle(5.0, "Model '%s' not found in /gazebo/model_states", self.robot_model_name)
        except Exception as e:
            rospy.logerr("Error in model_states_callback: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = GazeboTFPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
