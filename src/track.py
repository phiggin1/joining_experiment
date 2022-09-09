#!/usr/bin/env python2

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from kinova_msgs.msg import PoseVelocity
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply
import numpy as np

def get_direction(p1, p2):
    x = p1.pose.position.x - p2.pose.position.x
    y = p1.pose.position.y - p2.pose.position.y
    z = p1.pose.position.z - p2.pose.position.z

    return (x,y,z)

def print_pose(p):
    pos = [p.position.x, 
            p.position.y, 
            p.position.z]
    
    rot = [p.orientation.x, 
            p.orientation.y,
            p.orientation.z,
            p.orientation.w]

    pos_str = np.array2string(np.asarray(pos),  precision=2, separator=',')

    quat_rot_str = np.array2string(np.asarray(rot),  precision=2, separator=',')
    euler_rot_str = np.array2string(np.asarray(euler_from_quaternion(rot)),  precision=2, separator=',')
    print("Position:\t%s\tOrientation:\t%s" % (pos_str, quat_rot_str))

def print_quat(q):
    quat__str = np.array2string(np.asarray(q),  precision=2, separator=',')
    print("Orientation:\t%s" %  quat__str)

def quat_from_orientation(orientation):
    q = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ]

    return q

def angle_axis(q):
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]

    print(q)

    sqrt_q = math.sqrt( qx**2 + qy**2 + qz**2 )

    ax = qx/sqrt_q
    ay = qy/sqrt_q
    az = qz/sqrt_q

    theta = math.atan2(sqrt_q, qw)

    return theta, ax, ay, az

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

                print("target pose")
                print_pose(self.target_pose.pose)
                print("finger pose")
                print_pose(self.finger_pose.pose)

                distance = math.sqrt(d[0]**2  + d[1]**2 + d[2]**2)
                print("d: %f\tx: %fd\ty: %f\tz: %f" % (distance, d[0],d[1],d[2]))

                q_t = quat_from_orientation(self.target_pose.pose.orientation)
                q_f = quat_from_orientation(self.finger_pose.pose.orientation)

                q_error = quaternion_multiply( q_t , q_f)
                print("angular dist")
                print_quat(q_error)
                print(angle_axis(q_error))

                pose_vel = TwistStamped()
                pose_vel.header = self.finger_pose.header
                pose_vel.header.stamp = rospy.Time.now()

                pose_vel.twist.linear.x = (self.servo_speed*d[0] if abs(d[0]) > self.tolerance else 0.0)
                pose_vel.twist.linear.y = (self.servo_speed*d[1] if abs(d[1]) > self.tolerance else 0.0)
                pose_vel.twist.linear.z = (self.servo_speed*d[2] if abs(d[2]) > self.tolerance else 0.0)

                print(pose_vel.twist.linear)
                #self.cart_vel_pub.publish(pose_vel)
                rate.sleep()

if __name__ == '__main__':
    track = Tracker()
    while not rospy.is_shutdown():
        track.experiment()
