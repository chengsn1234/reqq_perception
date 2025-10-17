#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Height Query Node
响应RViz中的点击事件，显示点击位置的高度信息
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

class HeightQueryNode:
    def __init__(self):
        rospy.init_node('height_query_node', anonymous=True)
        
        # 参数
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅RViz中的点击点 (Publish Point tool)
        self.clicked_point_sub = rospy.Subscriber(
            '/clicked_point', 
            PointStamped, 
            self.clicked_point_callback
        )
        
        # 发布者
        self.info_pub = rospy.Publisher(
            '/height_query/info', 
            String, 
            queue_size=10
        )
        
        rospy.loginfo("Height Query Node initialized")
        rospy.loginfo("Click a point in RViz using 'Publish Point' tool to query height")
        

    def get_base_link_height(self):
        """获取base_link相对于map的高度"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_frame, 
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            return transform.transform.translation.z
            
        except Exception as e:
            rospy.logwarn(f"Failed to get base_link transform: {e}")
            return None
    def clicked_point_callback(self, msg):
        """处理RViz中点击的点"""
        rospy.loginfo(f"点击位置: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})")
        
        # 确保点在正确的坐标系中
        if msg.header.frame_id != self.map_frame:
            try:
                transformed_point = self.tf_buffer.transform(msg, self.map_frame)
                x, y = transformed_point.point.x, transformed_point.point.y
            except Exception as e:
                rospy.logerr(f"坐标转换失败: {e}")
                return
        else:
            x, y = msg.point.x, msg.point.y
            
        # 获取机器人高度
        base_height = self.get_base_link_height()
        
        # 创建信息消息
        clicked_z = msg.point.z  # RViz中点击的z值
        
        info_parts = [f"点击位置: ({x:.2f}, {y:.2f})"]
        info_parts.append(f"RViz显示高度: {clicked_z:.3f}m")
        
        if base_height is not None:
            info_parts.append(f"机器人高度: {base_height:.3f}m")
            height_diff = base_height - clicked_z
            info_parts.append(f"相对高度差: {height_diff:.3f}m")
        else:
            info_parts.append("机器人高度: 获取失败")
        
        info_msg = " | ".join(info_parts)
        
        rospy.loginfo(info_msg)
        self.info_pub.publish(String(data=info_msg))
        
def main():
    try:
        node = HeightQueryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Height Query Node error: {e}")

if __name__ == '__main__':
    main()