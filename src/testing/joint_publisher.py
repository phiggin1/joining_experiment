#! /usr/bin/env python3
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from std_msgs.msg import Float64MultiArray,Float64
from sensor_msgs.msg import JointState

import time
import numpy as np

class ControlJoints:
	def __init__(self):
		rospy.init_node('Controljoints', anonymous=True)

		self.joint_pubs = []
		for i in range(6):
			topic = '/j2n6s300/joint_'+str(i)+'_position_controller/command'
			self.joint_pubs.append(rospy.Publisher(topic, Float64, queue_size=10))

		self.joint_array_sub = rospy.Subscriber('/j2n6s300/joint_states', JointState, self.state_cb)

		self.joint_array_sub = rospy.Subscriber('/j2n6s300/joint_group_position_controller/command', Float64MultiArray, self.joints_cb)

		rospy.spin()

	def state_cb(self, state):
		a = np.around(np.asarray(state.position[0:6]), decimals=2)
		pi = np.around(2*np.pi, decimals=2)

		print("STATE:"+np.array2string((a%pi)*(180/pi), precision=2, separator=',', suppress_small=True))


	def joints_cb(self, joints):
		a = np.around(np.asarray(joints.data), decimals=2)
		pi = np.around(2*np.pi, decimals=2)

		print("CMD  :"+np.array2string((a%pi)*(180/pi), precision=2, separator=',', suppress_small=True))
		i = 0
		for j in joints.data:
			self.joint_pubs[i].publish(j%pi)
			i += 1

if __name__ == '__main__':
    control = ControlJoints()

