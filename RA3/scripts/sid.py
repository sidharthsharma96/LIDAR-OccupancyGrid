#!/usr/bin/env python
import numpy as np
import rosbag
import rospy

bag = rosbag.Bag('/home/sidharth/catkin_ws/src/RA3/src/3.bag')

for topic, msg, t in bag.read_messages(topics=['/velodyne_obstacles', '/trajectory']):
    if topic == '/velodyne_obstacles':
          print msg

    if topic == '/trajectory':
        print msg

    
bag.close()

