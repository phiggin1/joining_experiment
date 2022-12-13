#!/usr/bin/env python3

import rospy
from kortex_driver.msg import Base_JointSpeeds, JointSpeed
from trajectory_msgs.msg import JointTrajectory

class KortexHack:
    def __init__(self):
        rospy.init_node('kortex_hacked_fix', anonymous=True)
        self.servo_sub = rospy.Subscriber('/my_gen3/gen3_joint_trajectory_controller/command', JointTrajectory, self.servo_cb)
        self.joint_vel_pub = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size=10)
        rospy.spin()

    def servo_cb(self, joint_traj):
        point = joint_traj.points[0]

        joint_speeds = Base_JointSpeeds()
        for i, v in enumerate(point.velocity):
            speed = JointSpeed()
            speed.joint_identifier = i
            speed.value = v
            speed.duration = 0
            joint_speeds.joint_speeds.append(speed)

        self.joint_vel_pub.publish(joint_speeds)

if __name__ == '__main__':
    track = KortexHack()
