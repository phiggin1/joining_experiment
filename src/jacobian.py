#!/usr/bin/env python2
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Empty

def print_pose(p):
    pos = np.asarray(
        (p.position.x, 
        p.position.y, 
        p.position.z)
    )
    rot = np.asarray(
        (p.orientation.x, 
        p.orientation.y,
        p.orientation.z,
        p.orientation.w)
    )

    np.set_printoptions(precision=3, linewidth=120)
    print(np.asarray(pos))
    print(np.asarray(rot))

class GetJacobian:
    def __init__(self):
        self.stop = False
        rospy.init_node('jacobian', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group_name = "right_arm"
        self.arm_move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
      
        self.planning_frame = self.arm_move_group.get_planning_frame()
        self.eef_link = self.arm_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        print(self.arm_move_group.get_current_joint_values())
        print_pose(self.arm_move_group.get_current_pose().pose)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

        self.joint_sub = rospy.Subscriber('/j2n6s300_driver/out/joint_state', JointState, self.joint_state_cb)

        rospy.spin()

    def joint_state_cb(self, states):
        current = list(states.position[0:6])
        matrix_joint_sub = np.array(self.arm_move_group.get_jacobian_matrix(current))
        matrix_moveit = np.array(self.arm_move_group.get_jacobian_matrix(self.arm_move_group.get_current_joint_values()))
        np.set_printoptions(precision=2, linewidth=120)
        print(rospy.Time.now().to_sec())

        print("Joint Sub")
        print(np.asarray(current))
        print(matrix_joint_sub)
        print("Determinant: %f" % np.linalg.det(matrix_joint_sub))
        print("Condition: %f" % np.linalg.cond(matrix_joint_sub))

        print("MoveIT")
        print(np.asarray(self.arm_move_group.get_current_joint_values()))
        print(matrix_moveit)
        print("Determinant: %f" % np.linalg.det(matrix_moveit))
        print("Condition: %f" % np.linalg.cond(matrix_moveit))
        print('========================================')
        
if __name__ == '__main__':
    jac = GetJacobian()
