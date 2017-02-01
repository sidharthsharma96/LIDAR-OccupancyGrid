#!/usr/bin/env python
import numpy as np
import rosbag
import rospy

bag = rosbag.Bag('/home/sidharth/catkin_ws/src/RA3/src/3.bag')
count = 0
for topic, msg, t in bag.read_messages(topics=['/map']):
	b = []
	for i in range(101):
		a = []
		for j in range(101):
			a.append(int(msg.data[101*i + j]))
		b.append(a)
	fo = open("matrix" + str(count) + ".txt", "w")
	count+=1
	for i in range(101):
		for j in range(101):
			fo.write("%d "%b[i][j])
		if i != 100:
			fo.write("\n");
	fo.close()
bag.close()