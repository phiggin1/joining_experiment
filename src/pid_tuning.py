#! /usr/bin/env python
import numpy as np
import pandas as pd
import rospy
import message_filters
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

class PIDTuning:
    def moveJoint(self, jointcmds):
        print(jointcmds)
        
        rate = rospy.Rate(20)
        count = 0
        while (count < 20*20):
            jointCmd = JointTrajectory()  
            point = JointTrajectoryPoint()

            jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
            point.time_from_start = rospy.Duration.from_sec(5.0)

            mod = float(count)/100.0

            for i in range(0, 6):
                jointCmd.joint_names.append('j2n6s300_joint_'+str(i+1))
                point.positions.append(jointcmds[i]+mod)
                point.velocities.append(0)
                point.accelerations.append(0)
                point.effort.append(0) 

            print(mod)
            print(point.positions)

            jointCmd.points.append(point)


            self.pub.publish(jointCmd)
            count = count + 1
            rate.sleep()     
    
    def command_cb(self, command):
        if self.moveing:
            d = {} 
            target = np.asarray(command.points[0].positions)
            now = rospy.Time.now().to_sec()

            d['timestamp'] = now
            for i in range(0, 6):
                joint = 'j2n6s300_joint_'+str(i+1)
                d[joint+'_target'] = target[i]

            self.dataframe.append(d)

    def state_cb(self, state):
        if self.moveing:
            d = {} 
            actual = np.asarray(state.actual.positions)
            now = state.header.stamp.to_sec()

            d['timestamp'] = now
            for i in range(0, 6):
                joint = 'j2n6s300_joint_'+str(i+1)
                d[joint+'_actual'] = actual[i]

            self.dataframe.append(d)

    def shutdown(self):
        rospy.loginfo('begin quiting')
        self.moveing = False
        data_csv = pd.DataFrame(self.dataframe)
        data_csv.to_csv('test_servo.csv')
        rospy.loginfo('wrote to test_servo.csv')
        rospy.sleep(5)

    def __init__(self):
        rospy.init_node('pid_tuning')
        rospy.on_shutdown(self.shutdown)

        self.moveing = False
        #self.pub = rospy.Publisher('/j2n6s300/effort_joint_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.state_sub = rospy.Subscriber('/j2n6s300/effort_joint_trajectory_controller/state', JointTrajectoryControllerState, self.state_cb)
        self.command_sub = rospy.Subscriber('/j2n6s300/effort_joint_trajectory_controller/command', JointTrajectory, self.command_cb)
        self.dataframe = []
        self.moveing = True
        
        rospy.spin()
        '''
        self.large = 1.0
        self.small = 0.0005

        self.home_pose = [0.0,2.9,1.3,4.2,1.4,0.0]
        self.home_pose_small_move = [0.0+self.small,2.9+self.small,1.3+self.small,4.2+self.small,1.4+self.small,0.0+self.small]
        self.home_pose_large_move = [0.0+self.large,2.9+self.large,1.3+self.large,4.2+self.large,1.4+self.large,0.0+self.large]


        rospy.loginfo('home_pose')
        self.dataframe = []
        self.target = self.home_pose
        self.moveing = True
        self.moveJoint(self.home_pose)
        rospy.sleep(5.)
        self.moveing = False
        data_csv = pd.DataFrame(self.dataframe)
        data_csv.to_csv('test_servo.csv')
        '''
        '''
        rospy.loginfo('home_pose')
        self.moveJoint(self.home_pose)
        rospy.sleep(5.)
        self.dataframe = []
        self.target = self.home_pose
        self.moveing = True
        rospy.sleep(5.)
        rospy.loginfo('home_pose_small_move')
        self.target = self.home_pose_small_move
        self.moveJoint(self.home_pose_small_move)
        rospy.sleep(10.)
        self.moveing = False
        data_csv = pd.DataFrame(self.dataframe)
        data_csv.to_csv('test_small.csv')


        rospy.loginfo('home_pose')
        self.moveJoint(self.home_pose)
        rospy.sleep(5.)
        self.dataframe = []
        self.target = self.home_pose
        self.moveing = True
        rospy.sleep(5.)
        rospy.loginfo('home_pose_large_move')
        self.target = self.home_pose_large_move
        self.moveJoint(self.home_pose_large_move)
        rospy.sleep(10.)
        self.moveing = False
        data_csv = pd.DataFrame(self.dataframe)
        data_csv.to_csv('test_large.csv')
        '''


if __name__ == '__main__':
    tune = PIDTuning()