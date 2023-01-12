#!/usr/bin/env python3

import rospy
from kortex_driver.msg import TwistCommand, Twist
from geometry_msgs.msg import TwistStamped, PoseStamped

class KortexHack:
    def __init__(self):
        rospy.init_node('kortex_hacked_fix', anonymous=True)
        self.safe = False
        self.servo_sub = rospy.Subscriber('/my_gen3/servo_server/delta_twist_cmds', TwistStamped, self.delta_twist_cmds_cb)
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=10)
        self.finger_sub = rospy.Subscriber('finger_pose', PoseStamped, self.get_finger_pose)

        rospy.spin()

    def get_finger_pose(self, pose):
        if (pose.pose.position.z < 0.1):
            self.safe = False
        else:
            self.safe = True

    def delta_twist_cmds_cb(self, delta_twist):
        twist = TwistCommand()
        twist.reference_frame = 0
        twist.duration = 0

        if self.safe:
            twist.twist.linear_x = delta_twist.twist.linear.x
            twist.twist.linear_y = delta_twist.twist.linear.y
            twist.twist.linear_z = delta_twist.twist.linear.z
            twist.twist.angular_x = delta_twist.twist.angular.x
            twist.twist.angular_y = delta_twist.twist.angular.y
            twist.twist.angular_z = delta_twist.twist.angular.z
        else:
            twist.twist.linear_x = 0.0
            twist.twist.linear_y = 0.0
            twist.twist.linear_z = 0.0
            twist.twist.angular_x = 0.0
            twist.twist.angular_y = 0.0
            twist.twist.angular_z = 0.0

        self.cart_vel_pub.publish(twist)

if __name__ == '__main__':
    physical_arm = KortexHack()
