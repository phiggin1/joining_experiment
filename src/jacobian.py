#!/usr/bin/env python2
import sys
import os
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def print_pose(p):


    np.set_printoptions(precision=3, linewidth=120)
    print(np.asarray(pos))
    print(np.asarray(rot))



class GetJacobian:
    def __init__(self):
        self.stop = False

        rospy.init_node('jacobian', anonymous=True)

        self.is_sim = rospy.get_param("~rivr", False)
        if self.is_sim:
            joint_state_topic = ['joint_states:=/joint_states']
        else:
            joint_state_topic = ['joint_states:=/j2n6s300_driver/out/joint_state']



        moveit_commander.roscpp_initialize(joint_state_topic)


        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group_name = "right_arm"
        #self.arm_group_name = "arm"
        self.arm_move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
      
        self.planning_frame = self.arm_move_group.get_planning_frame()
        self.eef_link = self.arm_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        r = rospy.Rate(10) # 10hz 
        while not rospy.is_shutdown():
            self.get_jacobian()
            r.sleep()

    def get_jacobian(self):
        joint_angles = self.arm_move_group.get_current_joint_values()
        matrix_moveit = np.array(self.arm_move_group.get_jacobian_matrix(joint_angles))
        p = self.arm_move_group.get_current_pose().pose
        pos = [p.position.x, 
              p.position.y, 
              p.position.z]
        
        rot = [p.orientation.x, 
               p.orientation.y,
               p.orientation.z,
               p.orientation.w]

        joint_str = np.array2string(np.asarray(joint_angles),  precision=2, separator=',', suppress_small=True)

        ee_pos_str = np.array2string(np.asarray(pos),  precision=2, separator=',')

        ee_quat_rot_str = np.array2string(np.asarray(rot),  precision=2, separator=',')
        ee_euler_rot_str = np.array2string(np.asarray(euler_from_quaternion(rot)),  precision=2, separator=',')

        det = np.linalg.det(matrix_moveit)
        cond = np.linalg.cond(matrix_moveit)
        '''
        print("Time:\t%.2f\tJoint Angles:\t%s\tEE Position:\t%s\tEE Orientation:\t%s\tDeterminant:\t%.2f\tCondition:\t%.2f" 
            % (rospy.Time.now().to_sec(), joint_str, ee_pos_str, ee_rot_str, det, cond))
        
        print("Time:\t%.2f\tJoint Angles:\t%s\tEE Position:\t%s\tEE Orientation:\t%s\tCondition:\t%.2f" 
            % (rospy.Time.now().to_sec(), joint_str, ee_pos_str, ee_rot_str, cond))
        
        print("Time:\t%.2f\tEE Position:\t%s\tEE Orientation:\t%s\tCondition:\t%.2f" 
            % (rospy.Time.now().to_sec(), ee_pos_str, ee_rot_str, cond))
        '''

        print("Time:\t%.2f\tJoint Angles:\t%s\t \n\t EE Position:\t%s\tEE Orientation:\t%s\tCondition:\t%.2f" 
            % (rospy.Time.now().to_sec(), joint_str, ee_pos_str, ee_quat_rot_str, cond))
            
if __name__ == '__main__':
    jac = GetJacobian()
