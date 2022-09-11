#!/usr/bin/env python2

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from kinova_msgs.msg import PoseVelocity
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply
import numpy as np
from simple_pid import PID

def get_direction(p1, p2):
    x = p1.pose.position.x - p2.pose.position.x
    y = p1.pose.position.y - p2.pose.position.y
    z = p1.pose.position.z - p2.pose.position.z

    return (x,y,z)

def pose2sting(p):
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

    return "Position: %s\tOrientation: %s" % (pos_str, quat_rot_str)

def quat2string(q):
    quat__str = np.array2string(np.asarray(q),  precision=2, separator=',')

    return "Orientation: %s" %  quat__str

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
        self.base_frame = 'base_link'
        #self.base_frame = 'root'

        self.listener = tf.TransformListener()

        self.positional_tolerance = 0.01
        self.angular_tolerance = 0.1

        self.pub_rate = 10 #hz
        self.servo_speed = 0.5

        self.num_halt_msgs = 20

        self.time_out = 30.0

        self.x_pid = PID(Kp=1.5, Ki=0.0, Kd=0.0)
        self.y_pid = PID(Kp=1.5, Ki=0.0, Kd=0.0)
        self.z_pid = PID(Kp=1.5, Ki=0.0, Kd=0.0)
        self.theta_pid = PID(Kp=0.5, Ki=0.0, Kd=0.0)

        self.finger_sub = rospy.Subscriber('/test/finger_pose', PoseStamped, self.get_finger_pose)
        self.target_sub = rospy.Subscriber('/target/target', PoseStamped, self.get_target_pose)
        self.cart_vel_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)

    def get_finger_pose(self, pose):
        t = rospy.Time.now()
        pose.header.stamp = t
        self.listener.waitForTransform(pose.header.frame_id, self.base_frame, t, rospy.Duration(4.0) )
        self.finger_pose = self.listener.transformPose(self.base_frame, pose)

    def get_target_pose(self, pose):
        t = rospy.Time.now()
        pose.header.stamp = t
        self.listener.waitForTransform(pose.header.frame_id, self.base_frame, t, rospy.Duration(4.0) )
        self.target_pose = self.listener.transformPose(self.base_frame, pose)

    def satisfy_tolerance(self, angular_error, positional_error):
        x_err = positional_error[0]
        y_err = positional_error[1]
        z_err = positional_error[2]

        return (abs(x_err) < self.positional_tolerance and
                abs(y_err) < self.positional_tolerance and
                abs(z_err) < self.positional_tolerance and
                abs(angular_error) < self.angular_tolerance)

    def experiment(self):
        positional_error = [9999.9,9999.9,9999.9]
        angular_error = 9999.9
        last_time = None


        rate = rospy.Rate(self.pub_rate) # 10hz
        while (not self.satisfy_tolerance(angular_error, positional_error)):
            if (self.target_pose is not None and self.finger_pose is not None):
                time = rospy.Time.now().to_sec()

                if last_time is None:
                    dt = 0.1
                else:
                    dt = abs(time - last_time)
                last_time = time

                rospy.loginfo('dt: %f' % dt)
                positional_error = get_direction(self.finger_pose, self.target_pose)

                q_t = quat_from_orientation(self.target_pose.pose.orientation)
                q_f = quat_from_orientation(self.finger_pose.pose.orientation)
                q_r = quaternion_multiply( q_t , quaternion_inverse(q_f))
                angular_error, ax, ay, az =angle_axis(q_r)

                rospy.loginfo("Target  pose: " + pose2sting(self.target_pose.pose))
                rospy.loginfo("Current pose: " + pose2sting(self.finger_pose.pose))

                rospy.loginfo("Postional error x: %fd\ty: %f\tz: %f" % (positional_error[0],positional_error[1],positional_error[2]))
        
                rospy.loginfo("Max error           : %f" % (max(positional_error)))
                rospy.loginfo("positional tolerance: %f" % (self.positional_tolerance))

                rospy.loginfo("Angular error    : %f" % (angular_error))
                rospy.loginfo("angular tolerance: %f" % (self.angular_tolerance))

                t_l_x = self.x_pid(positional_error[0], dt)
                t_l_y = self.x_pid(positional_error[1], dt)
                t_l_z = self.x_pid(positional_error[2], dt)

                ang_vel_magnitude = self.theta_pid(angular_error, dt)
                t_a_x = ang_vel_magnitude * ax
                t_a_y = ang_vel_magnitude * ay
                t_a_z = ang_vel_magnitude * az

                pose_vel = TwistStamped()
                pose_vel.header = self.finger_pose.header
                pose_vel.header.stamp = rospy.Time.now()

                pose_vel.twist.linear.x = t_l_x
                pose_vel.twist.linear.y = t_l_y
                pose_vel.twist.linear.z = t_l_z

                pose_vel.twist.angular.x = t_a_x
                pose_vel.twist.angular.y = t_a_y
                pose_vel.twist.angular.z = t_a_z 

                rospy.loginfo(pose_vel.twist)
                self.cart_vel_pub.publish(pose_vel)
                rate.sleep()


        rospy.loginfo("Servoing fininshed")

        #send zero twist to halt servoing
        pose_vel = TwistStamped()
        pose_vel.header = self.finger_pose.header
        pose_vel.twist.linear.x = 0.0
        pose_vel.twist.linear.y = 0.0
        pose_vel.twist.linear.z = 0.0
        pose_vel.twist.angular.x = 0.0
        pose_vel.twist.angular.y = 0.0
        pose_vel.twist.angular.z = 0.0 

        rate = rospy.Rate(self.pub_rate*10) # 10hz
        for i in range(self.num_halt_msgs):
            pose_vel.header.stamp = rospy.Time.now()
            self.cart_vel_pub.publish(pose_vel)
            rate.sleep()
        rospy.loginfo("Servoing halted")

if __name__ == '__main__':
    track = Tracker()
    #while not rospy.is_shutdown():
    track.experiment()
