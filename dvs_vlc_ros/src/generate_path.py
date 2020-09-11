#!/usr/bin/python
# -*- coding: utf-8 -*-

# from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/

PKG = 'dvs_vlc_ros'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

import sys
import json
import time
sys.path.append('/home/koji/dvs/localization/src')
from localize import localize

class GeneratePath():
    def __init__(self):
        self.got_new_msg = False
        self.localizing = False
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'base'

        sub_odom = rospy.Subscriber("/odometry", Odometry, self.odometry_callback)
        pub_path = rospy.Publisher("/path", Path, queue_size=10)
        while not rospy.is_shutdown():
            if self.got_new_msg:
                pub_path.publish(self.path_msg)
                self.got_new_msg = False

    def odometry_callback(self, msg):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.pose.position.x = -msg.pose.pose.position.z
        pose_stamped_msg.pose.position.y = msg.pose.pose.position.x
        pose_stamped_msg.pose.position.z = -msg.pose.pose.position.y
        pose_stamped_msg.pose.orientation.x = msg.pose.pose.orientation.x
        pose_stamped_msg.pose.orientation.y = msg.pose.pose.orientation.y
        pose_stamped_msg.pose.orientation.z = msg.pose.pose.orientation.z
        pose_stamped_msg.pose.orientation.w = msg.pose.pose.orientation.w
        pose_stamped_msg.header.frame_id = 'base'
        pose_stamped_msg.header.stamp = msg.header.stamp
        # path_msg = Path()
        self.path_msg.poses.append(pose_stamped_msg)
        self.path_msg.header.stamp = msg.header.stamp
        self.got_new_msg = True

if __name__ == '__main__':
    rospy.init_node('generate_path')
    try:
        generate_path = GeneratePath()
    except rospy.ROSInterruptException: pass