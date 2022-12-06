#!/usr/bin/env python3
import sys
import math
import numpy as np
import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import pandas as pd


def direction(velocity):
    limit = 0.005
    dir = np.zeros_like(velocity)
    for (i,d) in enumerate(velocity):
        if (-limit <= d <= limit):
            #print(limit, d, 0)
            dir[i] = 0.0
        elif d > limit:
            #print(limit, d, 1.0)
            dir[i] = 1.0
        elif d < -limit:
            #print(limit, d, -1.0)
            dir[i] = -1.0
        else:
            dir[i] = 999

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


        self.servo_pub = rospy.Publisher('/my_gen3/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.joint_sub = rospy.Subscriber('/my_gen3/joint_states', JointState, self.joint_cb)

        home_goal = [0.0, 0.1775, -1.3784, 0.0, -1.57, 1.57]
        down_goal = [0.0, 0.6946, -2.0211, 0.0, -0.4115, 1.57]

        self.prev_joint_angles = np.asarray(home_goal)
        self.prev_joint_velocity = np.asarray([0.0,0.0,0.0,0.0,0.0,0.0])
        self.prev_ee_pose = np.asarray([0.3909,0.0014,0.4056,1.57,1.57,0.0])
        self.last_time = rospy.Time.now()

        self.data = []
        self.state = "home"

        rospy.sleep(0.1)

        #move to home
        self.state = "moving home"
        rospy.loginfo(self.state)
        self.arm_move_group.go(home_goal, wait=True)
        self.state = "home"
        rospy.loginfo(self.state)
        rospy.sleep(0.1)

        #move ee down
        self.state = "moving down"
        rospy.loginfo(self.state)
        self.arm_move_group.go(down_goal, wait=True)
        self.state = 'down'
        rospy.loginfo(self.state)
        rospy.sleep(0.1)

        #move to home
        self.state = 'moving home'
        rospy.loginfo(self.state)
        self.arm_move_group.go(home_goal, wait=True)
        self.state =  'home'
        rospy.loginfo(self.state)
        rospy.sleep(0.1)

        publish_rate = 100
        time_to_move = 10
        rate = rospy.Rate(publish_rate)
        count = 0
        twist_stamped = TwistStamped()
        twist_stamped.header.frame_id = 'base_link'

        twist_stamped.twist.linear.x = 0.0 
        twist_stamped.twist.linear.y = 0.0 
        twist_stamped.twist.linear.z = -1.0

        twist_stamped.twist.angular.x = 0.0 
        twist_stamped.twist.angular.y = 0.0 
        twist_stamped.twist.angular.z = 0.0 

        self.state = 'servoing drown'
        rospy.loginfo(self.state)
        while count < publish_rate*time_to_move:
            twist_stamped.header.stamp = rospy.Time.now()
            self.servo_pub.publish(twist_stamped)
            count += 1
            rate.sleep()


        twist_stamped.twist.linear.z = 0.0
        self.state = 'servoing zero twist'
        count = 0
        rospy.loginfo(self.state)
        while count < publish_rate*time_to_move:
            twist_stamped.header.stamp = rospy.Time.now()
            self.servo_pub.publish(twist_stamped)
            count += 1
            rate.sleep()

        #move to home
        self.state = 'moving home'
        rospy.loginfo(self.state)
        self.arm_move_group.go(home_goal, wait=True)
        self.state =  'home'
        rospy.loginfo(self.state)
        rospy.sleep(0.1)

        #rospy.spin()
        '''rospy.loginfo('pre writing')
        data_csv = pd.DataFrame(self.data)
        data_csv.to_csv('/home/iral/'+str(rospy.Time.now().to_sec())+'jacobian.csv')
        rospy.loginfo('post writing')
        rospy.sleep(1)'''

        #ax1 = data_csv.plot.scatter(x='timestamp', y='joint_vel_1')


    def joint_cb(self, joint_state):
        joint_angles = list(joint_state.position[1:7])
        joint_velocity = list(joint_state.velocity[1:7])
        jacobian = np.array(self.arm_move_group.get_jacobian_matrix(joint_angles))

        inv_jacobian = np.linalg.inv(jacobian)

        cond = np.linalg.cond(jacobian)
        ee_pose = self.arm_move_group.get_current_pose().pose

        dt = joint_state.header.stamp.to_sec() - self.last_time.to_sec()
        x = ee_pose.position.x
        y = ee_pose.position.y
        z = ee_pose.position.z
        roll, pitch, yaw = euler_from_quaternion([ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w])

        ee_pose = np.asarray([x,y,z,roll,pitch,yaw])
        ee_vel = (ee_pose-self.prev_ee_pose)/dt

        jacobian_ee_vel = np.matmul(jacobian, np.asarray(joint_velocity))
        inv_jacobian_joint_vel = np.matmul(inv_jacobian, np.asarray(ee_vel))

        np.set_printoptions(precision=3, linewidth=120)

        entry = {}
        entry['timestamp'] = joint_state.header.stamp.to_sec()

        '''entry['ee_pos_x'] = ee_pose[0]
        entry['ee_pos_y'] = ee_pose[1]
        entry['ee_pos_z'] = ee_pose[2]
        entry['ee_pos_roll'] = ee_pose[3]
        entry['ee_pos_pitch'] = ee_pose[4]
        entry['ee_pos_yaw'] = ee_pose[5]

        entry['ee_vel_x'] = ee_vel[0]
        entry['ee_vel_y'] = ee_vel[1]
        entry['ee_vel_z'] = ee_vel[2]
        entry['ee_vel_roll'] = ee_vel[3]
        entry['ee_vel_pitch'] = ee_vel[4]
        entry['ee_vel_yaw'] = ee_vel[5]

        entry['jacobian_ee_vel_x'] = jacobian_ee_vel[0]
        entry['jacobian_ee_vel_y'] = jacobian_ee_vel[1]
        entry['jacobian_ee_vel_z'] = jacobian_ee_vel[2]
        entry['jacobian_ee_vel_roll'] = jacobian_ee_vel[3]
        entry['jacobian_ee_vel_pitch'] = jacobian_ee_vel[4]
        entry['jacobian_ee_vel_yaw'] = jacobian_ee_vel[5]'''

        entry['joint_vel_1'] = joint_velocity[0]
        entry['joint_vel_2'] = joint_velocity[1]
        entry['joint_vel_3'] = joint_velocity[2]
        entry['joint_vel_4'] = joint_velocity[3]
        entry['joint_vel_5'] = joint_velocity[4]
        entry['joint_vel_6'] = joint_velocity[5]


        entry['inv_jacobian_joint_vel_1'] = inv_jacobian_joint_vel[0]
        entry['inv_jacobian_joint_vel_2'] = inv_jacobian_joint_vel[1]
        entry['inv_jacobian_joint_vel_3'] = inv_jacobian_joint_vel[2]
        entry['inv_jacobian_joint_vel_4'] = inv_jacobian_joint_vel[3]
        entry['inv_jacobian_joint_vel_5'] = inv_jacobian_joint_vel[4]
        entry['inv_jacobian_joint_vel_6'] = inv_jacobian_joint_vel[5]

        entry['state'] = self.state
        #entry['condition'] = cond

        self.data.append(entry)

        #rospy.loginfo(np.array2string(ee_pose,  precision=4, separator=',')+'\t'+np.array2string(ee_vel,  precision=4, separator=',') +'\t'+np.array2string(jacobian_ee_vel,  precision=4, separator=',')+'\t'+str(cond))

        #rospy.loginfo(abs(ee_vel - jacobian_ee_vel))

        self.prev_joint_angles = joint_angles
        self.prev_ee_pose = ee_pose
        self.last_time = joint_state.header.stamp

if __name__ == '__main__':
    jac = GetJacobian()