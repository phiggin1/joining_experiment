#!/usr/bin/env python2

import rospy
import tf
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from joining_experiment.msg import JoinPose
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply
import numpy as np
import pandas as pd

import festival
import soundfile as sf
import json

from simple_pid import PID
from joining_experiment.srv import JoiningServo, JoiningServoResponse

def get_position_error(p1, p2):
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
    euler_rot_str = np.array2string(np.asarray(euler_from_quaternion(rot)),  precision=2, separator=',', suppress_small=True)

    return "Position: %s\tOrientation: %s" % (pos_str, euler_rot_str)

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

def zero_twist():
    zero_vel = TwistStamped()
    zero_vel.twist.linear.x = 0.0
    zero_vel.twist.linear.y = 0.0
    zero_vel.twist.linear.z = 0.0
    zero_vel.twist.angular.x = 0.0
    zero_vel.twist.angular.y = 0.0
    zero_vel.twist.angular.z = 0.0

    return zero_vel

class Tracker:
    def __init__(self):
        rospy.init_node('joining_pose_tracking', anonymous=True)

        self.target_pose = None
        self.finger_pose = None
        self.base_frame = 'base_link'

        self.listener = tf.TransformListener()

        self.positional_tolerance = 0.01
        self.angular_tolerance = 0.1

        self.pub_rate = 10 #hz

        #number of zero twist halt msgs to send to get servo_server to halt
        self.num_halt_msgs = 20

        self.time_out = 25.0

        self.is_sim = rospy.get_param("~rivr", True)
        if self.is_sim:
            rospy.loginfo("virtual robot")
        else:
            rospy.loginfo("physical robot")

        #proportional gains  
        self.cart_x_kp = rospy.get_param("~cart_x_kp", 10.0)
        self.cart_y_kp = rospy.get_param("~cart_y_kp", 10.0)
        self.cart_z_kp = rospy.get_param("~cart_z_kp", 100.0)
        self.angular_kp = rospy.get_param("~angular_kp", 0.5)

        #integral gains
        self.cart_x_ki = rospy.get_param("~cart_x_ki", 0.0)
        self.cart_y_ki = rospy.get_param("~cart_y_ki", 0.0)
        self.cart_z_ki = rospy.get_param("~cart_z_ki", 0.0)
        self.angular_ki = rospy.get_param("~angular_ki", 0.0)

        #derivative gains
        self.cart_x_kd = rospy.get_param("~cart_x_kd", 0.0)
        self.cart_y_kd = rospy.get_param("~cart_y_kd", 0.0)
        self.cart_z_kd = rospy.get_param("~cart_z_kd", 0.0)
        self.angular_kd = rospy.get_param("~angular_kd", 0.0)

        self.near = True
        self.see_cathode = True
        self.see_anode = True 
        self.speech_delay = 5.0

        self.finger_sub = rospy.Subscriber('/test/finger_pose', PoseStamped, self.get_finger_pose)
        self.target_sub = rospy.Subscriber('/target/target', JoinPose, self.get_target_pose)
        self.cart_vel_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.rivr_robot_speech = rospy.Publisher('/robotspeech', String, queue_size=10)

        self.service = rospy.Service('JoiningServo', JoiningServo, self.experiment)
        rospy.spin()

    def get_finger_pose(self, pose):
        t = rospy.Time.now()
        pose.header.stamp = t
        self.listener.waitForTransform(pose.header.frame_id, self.base_frame, t, rospy.Duration(4.0) )
        self.finger_pose = self.listener.transformPose(self.base_frame, pose)

    def get_target_pose(self, tar_pose):
        pose = PoseStamped()
        pose.header = tar_pose.header
        pose.pose = tar_pose.pose

        self.near = tar_pose.near.data
        self.see_cathode = tar_pose.see_cathode.data 
        self.see_anode = tar_pose.see_anode.data 

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

    def talk(self, str):
        if self.is_sim:
            print("Saying rivr: " + str)
            #wav = festival.textToWav(str)
            #data = sf.read(wav)
            #string_msg =json.dumps(list(data[0]))
            #self.rivr_robot_speech.publish(string_msg)
        else:
            #festival.sayText(str)
            print("Saying real: " + str)

    def experiment(self, req):
        positional_error = [9999.9,9999.9,9999.9]
        angular_error = 9999.9

        x_pid = PID(Kp=self.cart_x_kp, Ki=self.cart_x_ki, Kd=self.cart_x_kd)
        y_pid = PID(Kp=self.cart_y_kp, Ki=self.cart_y_ki, Kd=self.cart_y_kd)
        z_pid = PID(Kp=self.cart_z_kp, Ki=self.cart_z_ki, Kd=self.cart_z_kd)
        theta_pid = PID(Kp=self.angular_kp, Ki=self.angular_ki, Kd=self.angular_kd)

        x_pid.output_limits = (-1.0, 1.0)    # Output value will be between -1.0 and 1.0
        y_pid.output_limits = (-1.0, 1.0)    # Output value will be between -1.0 and 1.0
        z_pid.output_limits = (-10.0, 10.0)    # Output value will be between -10.0 and 10.0
        theta_pid.output_limits = (-1.0, 1.0)    # Output value will be between -1.0 and 1.0

        total_time = 0.0
        timed_out = False
        last_time_spoke = None

        rate = rospy.Rate(self.pub_rate) # 10hz
        while (not self.satisfy_tolerance(angular_error, positional_error) and total_time < self.time_out):  
            if (not self.near or not self.see_cathode or not self.see_anode):
                now = rospy.Time.now().to_sec()
                if not self.see_anode and not self.see_cathode and (last_time_spoke is None or now > last_time_spoke+self.speech_delay):
                    print(last_time_spoke, now)
                    self.talk("Can you please move the red and green putty to where I can see tem?")
                    last_time_spoke = rospy.Time.now().to_sec()

                if not self.see_anode and (last_time_spoke is None or now > last_time_spoke+self.speech_delay):
                    print(last_time_spoke, now)
                    self.talk("Can you please move the red putty where I can see it?")
                    last_time_spoke = rospy.Time.now().to_sec()

                if not self.see_cathode and (last_time_spoke is None or now > last_time_spoke+self.speech_delay):
                    self.talk("Can you please move the green putty where I can see it?")
                    last_time_spoke = rospy.Time.now().to_sec()

                if not self.near and (last_time_spoke is None or now > last_time_spoke+self.speech_delay):
                    self.talk("Can you please move the putty closer together?")
                    last_time_spoke = rospy.Time.now().to_sec()

                self.cart_vel_pub.publish(zero_twist())


            elif (self.target_pose is not None and self.finger_pose is not None):
                time = rospy.Time.now().to_sec()
                dt = 1.0/self.pub_rate
                
                positional_error = get_position_error(self.finger_pose, self.target_pose)

                q_t = quat_from_orientation(self.target_pose.pose.orientation)
                q_f = quat_from_orientation(self.finger_pose.pose.orientation)
                q_r = quaternion_multiply( q_t , quaternion_inverse(q_f))
                angular_error, ax, ay, az = angle_axis(q_r)

                rospy.loginfo('Elapsed time: %f' % total_time)
                rospy.loginfo("Target  pose: " + pose2sting(self.target_pose.pose))
                rospy.loginfo("Current pose: " + pose2sting(self.finger_pose.pose))

                rospy.loginfo("Postional error    x: %.3f\ty: %.3f\tz: %.3f" % (positional_error[0],positional_error[1],positional_error[2]))
                rospy.loginfo("positional tolerance: %f" % (self.positional_tolerance))

                rospy.loginfo("Angular error       : %f" % (angular_error))
                rospy.loginfo("angular tolerance   : %f" % (self.angular_tolerance))

                #get twist linear values from PID controllers
                t_l_x = x_pid(positional_error[0], dt)
                t_l_y = y_pid(positional_error[1], dt)
                t_l_z = z_pid(positional_error[2], dt)

                #get twist angular values
                #   get euler angles from axis angles of quaternion
                ang_vel_magnitude = theta_pid(angular_error, dt)
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

                rospy.loginfo("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f"%(pose_vel.twist.linear.x,pose_vel.twist.linear.y,pose_vel.twist.linear.z,pose_vel.twist.angular.x,pose_vel.twist.angular.y,pose_vel.twist.angular.z))
                self.cart_vel_pub.publish(pose_vel)
                
                total_time += dt
                if total_time > self.time_out:
                    timed_out = True

                rate.sleep()

        #send zero twist to halt servoing
        pose_vel = zero_twist()
        rate = rospy.Rate(self.pub_rate) # 10hz
        for i in range(self.num_halt_msgs):
            pose_vel.header.stamp = rospy.Time.now()
            self.cart_vel_pub.publish(pose_vel)
            rate.sleep()

        if timed_out:
            rospy.loginfo("Servoing timed out")
        else:
            rospy.loginfo("Servoing took %f seconds" % total_time)

        return JoiningServoResponse(not timed_out)

if __name__ == '__main__':
    track = Tracker()
