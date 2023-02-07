#!/usr/bin/env python3

import rospy
import tf
from kortex_driver.msg import TwistCommand, Twist
from geometry_msgs.msg import TwistStamped, PoseStamped

def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

class KortexHack:
    def __init__(self):
        rospy.init_node('kortex_hacked_fix', anonymous=True)

        self.listener = tf.TransformListener()
        self.safe = False
        self.base_frame = 'base_link'
        self.servo_sub = rospy.Subscriber('/my_gen3/servo_server/delta_twist_cmds', TwistStamped, self.delta_twist_cmds_cb)
        self.cart_vel_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=10)
        self.finger_sub = rospy.Subscriber('/my_gen3/finger_pose', PoseStamped, self.get_finger_pose)
        self.min_linear_vel = -0.025
        self.max_linear_vel =  0.025
        self.min_angular_vel = -0.2
        self.max_angular_vel =  0.2
        rospy.spin()

    def get_finger_pose(self, pose):
        t = rospy.Time.now()
        pose.header.stamp = t

        self.listener.waitForTransform(pose.header.frame_id, self.base_frame, t, rospy.Duration(4.0) )
        self.finger_pose = self.listener.transformPose(self.base_frame, pose)
        
        if (self.finger_pose.pose.position.z < 0.1):
            self.safe = False
        else:
            self.safe = True

    def delta_twist_cmds_cb(self, delta_twist):

        twist = TwistCommand()
        twist.reference_frame = 0
        twist.duration = 0

        if self.safe:
            twist.twist.linear_x = clamp(delta_twist.twist.linear.x, self.min_linear_vel, self.max_linear_vel)
            twist.twist.linear_y = clamp(delta_twist.twist.linear.y, self.min_linear_vel, self.max_linear_vel)
            twist.twist.linear_z = clamp(delta_twist.twist.linear.z, self.min_linear_vel, self.max_linear_vel)
            twist.twist.angular_x = clamp(delta_twist.twist.angular.x, self.min_angular_vel, self.max_angular_vel)
            twist.twist.angular_y = clamp(delta_twist.twist.angular.y, self.min_angular_vel, self.max_angular_vel)
            twist.twist.angular_z = clamp(delta_twist.twist.angular.z, self.min_angular_vel, self.max_angular_vel)
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
