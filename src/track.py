#!/usr/bin/env python2

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from kinova_msgs.msg import PoseVelocity

def get_direction(p1, p2):
    x = p1.pose.position.x - p2.pose.position.x
    y = p1.pose.position.y - p2.pose.position.y
    z = p1.pose.position.z - p2.pose.position.z

    return (x,y,z)

class Tracker:
    def __init__(self):
        rospy.init_node('track', anonymous=True)

        self.target_pose = None
        self.finger_pose = None

        self.listener = tf.TransformListener()

        self.tolerance = 0.01
        self.pub_rate = 10 #hz
        self.servo_speed = 0.5

        self.finger_sub = rospy.Subscriber('/test/finger_pose', PoseStamped, self.get_finger_pose)
        self.target_sub = rospy.Subscriber('/target/target', PoseStamped, self.get_target_pose)
        self.cart_vel_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)

    def get_finger_pose(self, pose):
        t = rospy.Time.now()
        pose.header.stamp = t
        self.listener.waitForTransform(pose.header.frame_id, 'root', t, rospy.Duration(4.0) )
        self.finger_pose = self.listener.transformPose('root', pose)


    def get_target_pose(self, pose):
        t = rospy.Time.now()
        pose.header.stamp = t
        self.listener.waitForTransform(pose.header.frame_id, 'root', t, rospy.Duration(4.0) )
        self.target_pose = self.listener.transformPose('root', pose)

    def experiment(self):
        distance = 99999.9
        rate = rospy.Rate(self.pub_rate) # 10hz
        while distance > self.tolerance and not rospy.is_shutdown():
            if self.target_pose is not None and self.finger_pose is not None:
                d = get_direction(self.target_pose, self.finger_pose)
		print("tar", self.target_pose.pose.position)
		print("fin", self.finger_pose.pose.position)
                distance = math.sqrt(d[0]**2  + d[1]**2 + d[2]**2)
                print(d, distance)

                pose_vel = TwistStamped()
                pose_vel.header = self.finger_pose.header
                pose_vel.header.stamp = rospy.Time.now()
                #x = (a if b else 0)

                pose_vel.twist.linear.x = (self.servo_speed*d[0] if abs(d[0]) > self.tolerance else 0.0)
                pose_vel.twist.linear.y = (self.servo_speed*d[1] if abs(d[1]) > self.tolerance else 0.0)
                pose_vel.twist.linear.z = (self.servo_speed*d[2] if abs(d[2]) > self.tolerance else 0.0)

                print(pose_vel.twist.linear)
                self.cart_vel_pub.publish(pose_vel)
                rate.sleep()

if __name__ == '__main__':
    track = Tracker()
    while not rospy.is_shutdown():
        track.experiment()
