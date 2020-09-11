#!/usr/bin/python
# -*- coding: utf-8 -*-

# from https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/

PKG = 'dvs_vlc_ros'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from dvs_msgs.msg import EventArray
from nav_msgs.msg import Odometry

import sys
import json
import time
sys.path.append('/home/koji/dvs/localization/src')
from localize import localize

class DVSLocalize():
    def __init__(self):
        # const
        self.buffer_time = 0.8

        self.got_new_msg = False
        self.odom_msg = False
        self.localizing = False
        self.event_array_buffer = [None for _ in range(int(25000*self.buffer_time))]
        self.current_idx = 0
        self.buffer_start_time = -1

        with open('/home/koji/dvs/localization/cfg/config.json') as f:
            self.config = json.load(f)
        # Create subsc and pub
        sub_eventarray = rospy.Subscriber("prophesee/camera/cd_events_buffer", EventArray, self.eventarray_callback)
        pub_odom = rospy.Publisher("odometry", Odometry, queue_size=10)
        while not rospy.is_shutdown():
            if self.got_new_msg:
                pub_odom.publish(self.odom_msg)
                self.got_new_msg = False

    def eventarray_callback(self, msg):
        if not self.localizing:
            if self.event_array_buffer[0] is None:
                self.buffer_start_time = msg.header.stamp.to_time()
            self.event_array_buffer[self.current_idx] = msg
            self.current_idx += 1
            # rospy.loginfo('Time {}'.format(msg.header.stamp.to_time() - self.buffer_start_time))
            if msg.header.stamp.to_time() - self.buffer_start_time > self.buffer_time:
                rospy.loginfo('IMPORTANT: localization start!')
                self.localizing = True
                self.localize_callback()

    def localize_callback(self):
        # time.sleep(1)

        event_list, event_time_list = [], []
        start_time = -1
        for event_array in self.event_array_buffer:
            if event_array is None:
                break
            if start_time<0:
                start_time = event_array.events[0].ts.to_time()
            for event in event_array.events:
                event_list.append([int(event.x), int(event.y), event.ts.to_time() - start_time, event.polarity])
                event_time_list.append(event.ts.to_time() - start_time)
        
        rospy.loginfo('length: {0}, {1}'.format(len(event_list), len(event_time_list)))
        R, T = localize(event_list, event_time_list, self.config)

        # initialize
        self.event_array_buffer = [None for _ in range(int(25000*self.buffer_time))]
        self.current_idx = 0
        rospy.loginfo('IMPORTANT: localized!')
        # self.odom_msg = Odometry()
        # # update odometry

        # self.got_new_msg = True
        self.localizing = False

if __name__ == '__main__':
    rospy.init_node('dvs_vlc')
    try:
        dvs_localize = DVSLocalize()
    except rospy.ROSInterruptException: pass