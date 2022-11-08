#!/usr/bin/env python3

import numpy as np
import rospy
import moveit_commander
from sensor_msgs.msg import JointState


def direction(velocity):
    limit = 0.00001
    dir = np.zeros_like(array)
    for (i,d) in enumerate(velocity):
        if (-limit < d < limit):
            dir[i] = 0.0
        elif d > 0.0:
            dir[i] = 1.0
        else:
            dir[i] = -1.0

    return dir



class GetJacobian:
    def __init__(self):
        self.stop = False
        rospy.init_node('jacobian', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group_name = "arm"
        self.arm_move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
      
        self.planning_frame = self.arm_move_group.get_planning_frame()
        self.eef_link = self.arm_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        self.prev_joint_angles = np.array(0.0,0.0,0.0,0.0,0.0,0.0)
        self.prev_joint_velocity = np.array(0.0,0.0,0.0,0.0,0.0,0.0)
        self.prev_ee_pose = np.array(0.0,0.0,0.0,0.0,0.0,0.0)

        self.joint_sub = rospy.Subscriber('/my_gen3/joint_states', JointState, self.joint_cb)

        home_goal = [0.0, 0.26, -2.27, 0.0, 0.96, 1.57]
        down_goal = [0.0, 0.57, -2.3, 0.0, 1.3, 0.0]
        rospy.sleep(5)

        #move to home
        self.arm_move_group.go(home_goal, wait=True)
        rospy.sleep(5)

        #move ee down
        self.arm_move_group.go(down_goal, wait=True)
        rospy.sleep(5)

        #move to home
        self.arm_move_group.go(home_goal, wait=True)
        rospy.sleep(5)

        #rospy.spin()

    def joint_cb(self, joint_state):
        joint_angles = np.array(joint_state.position)
        joint_velocity = np.array(joint_state.velocity)

        jacobian = np.array(self.arm_move_group.get_jacobian_matrix(joint_angles))
        ee_pose = self.arm_move_group.get_current_pose().pose

        rospy.loginfo(ee_pose)
        rospy.loginfo(joint_angles)
        rospy.loginfo(joint_velocity)
        rospy.loginfo(jacobian*joint_velocity)

        self.prev_joint_angles = joint_angles
        self.prev_joint_velocity = joint_velocity
        self.prev_ee_pose = ee_pose

if __name__ == '__main__':
    jac = GetJacobian()