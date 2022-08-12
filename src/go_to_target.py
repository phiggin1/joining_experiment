#!/usr/bin/env python2
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import moveit_commander
import moveit_msgs.msg
from std_srvs.srv import Empty
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import actionlib
import kinova_msgs.msg
#import pyttsx3

def quaternion_from_msg(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]

def get_marker(p):
    marker = Marker()

    marker.id = 0
    marker.header.frame_id = p.header.frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = marker.SPHERE

    marker.action = marker.ADD

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0        
    marker.color.a = 0.5

    marker.scale.x = 0.025
    marker.scale.y = 0.025
    marker.scale.z = 0.025

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = p.pose.position.x
    marker.pose.position.y = p.pose.position.y
    marker.pose.position.z = p.pose.position.z

    return marker

class GoToTarget:
    def __init__(self):
        self.stop = False
        rospy.init_node('move_to', anonymous=True)

        self.pose_pub = rospy.Publisher('/target/target_pos_base', PoseStamped, queue_size=10)

        self.target_offset_pub = rospy.Publisher('/target/offset_target_pos', PoseStamped, queue_size=10)
        self.grab = rospy.Publisher('buttons', String, queue_size=10)

        #self.speech = pyttsx3.init()


        #action_address = '/j2n6s300_driver/fingers_action/finger_positions'

        #self.client = actionlib.SimpleActionClient(action_address,
        #                                    kinova_msgs.msg.SetFingersPositionAction)
        #self.client.wait_for_server()

        self.interactive = True

        self.target_topic = "/target/target"
        self.retry_times = 10           
        self.finger_full_open = 0.0
        self.finger_open = 5750.0
        self.finger_Full_closed = 6460.0

        self.hand_open = [self.finger_open, self.finger_open, self.finger_full_open]
        self.hand_closed = [self.finger_Full_closed, self.finger_Full_closed, self.finger_full_open]

        self.standoff_distance = 0.20       #m
        self.hand_finger_offset_x = 0.0#0.035   #m
        self.hand_finger_offest_y = 0.0#0.01    #m
        self.hand_finger_offest_z = 0.15   #m
        self.goal_tolerance = 0.0025        #m

        self.hand_over_pose = PoseStamped()
        self.hand_over_pose.header.frame_id = "base_link"
        self.hand_over_pose.pose.position.x =  0.55
        self.hand_over_pose.pose.position.y = -0.08
        self.hand_over_pose.pose.position.z =  0.95
        self.hand_over_pose.pose.orientation.x = -0.5
        self.hand_over_pose.pose.orientation.y = -0.5
        self.hand_over_pose.pose.orientation.z =  0.5
        self.hand_over_pose.pose.orientation.w =  0.5

        self.listener = tf.TransformListener()
   
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group_name = "right_arm"
        self.arm_move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.arm_move_group.set_max_velocity_scaling_factor(0.25)
        self.arm_move_group.set_goal_position_tolerance(self.goal_tolerance)
        
        self.hand_group_name = "right_hand"
        self.hand_move_group = moveit_commander.MoveGroupCommander(self.hand_group_name)
        self.hand_move_group.set_max_velocity_scaling_factor(1.0)
        self.hand_move_group.set_goal_position_tolerance(self.goal_tolerance)

        self.planning_frame = self.arm_move_group.get_planning_frame()
        self.eef_link = self.arm_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        #rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        #self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        
    def talk(self, str):
        print(str)
        #self.speech.say(str)
        #self.speech.runAndWait()

    def get_target(self):
        count = 0
        target = None
        while target is None:
            try:
                target = rospy.wait_for_message(self.target_topic, PoseStamped, timeout=5.0)
            except rospy.exceptions.ROSException:
                rospy.loginfo("Timeout waiting for target")
                count+=1
                if count < self.retry_times:
                    pass
                else:
                    i = raw_input("Continue? (y/n) ")
                    if i == 'n':
                        rospy.loginfo("no targets found after "+str(self.retry_times)+" attempts, exiting")
                        exit()
                    else:
                        if self.interactive:
                            self.talk("Can you move the two pieces of putty closer together?")
                        count = 0
                        pass

        t = rospy.Time.now()
        target.header.stamp = t
        self.listener.waitForTransform(target.header.frame_id, self.planning_frame, t, rospy.Duration(4.0) )
        goal_pose = self.listener.transformPose(self.planning_frame, target)
        '''
        p, q = self.listener.lookupTransform(self.planning_frame, target.header.frame_id, t)

        goal_pose.pose.orientation.x = target.pose.orientation.x
        goal_pose.pose.orientation.y = target.pose.orientation.z
        goal_pose.pose.orientation.z = target.pose.orientation.y
        goal_pose.pose.orientation.w = target.pose.orientation.w
        '''
        
        print( goal_pose.pose.orientation.z )
        print('------------')
        if goal_pose.pose.orientation.z > 0:
            print('green robot right')
            sign = -1.0
        else:
            print('green robot left')
            sign = 1.0

        print(goal_pose.pose.position.y)
        print(sign*self.hand_finger_offest_y)

        goal_pose.pose.position.x -= self.hand_finger_offset_x
        goal_pose.pose.position.y += (sign*self.hand_finger_offest_y)
        goal_pose.pose.position.z += self.hand_finger_offest_z

        print(goal_pose.pose.position.y)
        print('============')
        
        goal_pose.pose.position.z += self.hand_finger_offest_z
        return goal_pose

    def move_arm(self, pose, speed):
        #clear = self.clear_octomap()
        self.arm_move_group.set_max_velocity_scaling_factor(speed)
        self.arm_move_group.set_pose_target(pose.pose)
        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()
        current_pose = self.arm_move_group.get_current_pose()
        
        #rospy.loginfo("move arm goal pose")
        #rospy.loginfo(pose)

    def gripper_client(self, finger_positions):
        """Send a gripper goal to the action server."""

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions[0])
        goal.fingers.finger2 = float(finger_positions[1])
        goal.fingers.finger3 = float(finger_positions[2])

        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(5.0)):
            return self.client.get_result()
        else:
            self.client.cancel_all_goals()
            rospy.logwarn('the gripper action timed-out')
            return None

    def experiment(self):
        self.failures = 0
        
        #move to handover position
        self.move_arm(self.hand_over_pose, 1.0)
        '''
        self.gripper_client(self.hand_open)

        #LED = l e d
        if self.interactive:
            self.talk("Can you please put the l e d between my fingers?")
            self.talk("The shorter lead should be on your left.")
        '''
        #wait for LED to be given then close hand
        if self.interactive:
            raw_input("Hand over led, Press Enter to continue...")
        self.grab.publish("grabbed")
        #self.gripper_client(self.hand_closed)
                
        '''
        #give instructions
        if self.interactive:
            s = "connect the red clip to the positive terminal of the battery"
            self.talk(s)

            raw_input("Press Enter to continue...")

            s = "connect the black clip to the negative terminal of the battery"
            self.talk(s)
            raw_input("Press Enter to continue...")

            s = "place the red clip into the red putty"
            self.talk(s)
            raw_input("Press Enter to continue...")

            s = "place the black clip into the green putty"
            self.talk(s)
            raw_input("Press Enter to continue...")

            s = "can you hold the two pieces of putty up in front of me?"
            self.talk(s)
            raw_input("Press Enter to continue...")
        '''

        reached_target = False
        while not reached_target:
            goal_pose = self.get_target()
            print('goal')
            print(goal_pose.pose.position)
            print(goal_pose.pose.orientation)

            goal_pose.pose.position.z += self.standoff_distance
            print('goal standoff')
            print(goal_pose.pose.position)
            
            if self.interactive:
                raw_input("Move to standoff, Press Enter to continue...")
            self.move_arm(goal_pose, 1.0)
            
            #use servoing if this dosent work well enough
            #check target -> update goal
            final_pose = self.get_target()
            print('goal final')
            print(final_pose.pose.position)
            self.target_offset_pub.publish(final_pose)
            
            if self.interactive:
                i = raw_input("Move to final, Press Enter to continue...")
            
            self.move_arm(final_pose, 1.0)
            
            #check if should open hand
            if self.interactive:
                i = raw_input("Open hand (y/n) ")
                if i == 'y':
                    print('open hand')
                    self.grab.publish("released")
                    #self.gripper_client(self.hand_open)
                    reached_target = True
                else:
                    self.failures += 1
            
            print('goal standoff pose')
            print(goal_pose.pose.position)
            self.move_arm(goal_pose, 1.0)
            
        
        #print('goal hand over backoff pose')
        #print(self.hand_over_pose.pose.position)
        #self.move_arm(self.hand_over_pose, 1.0)
        
        
        #re home the arm
        self.arm_move_group.set_max_velocity_scaling_factor(1.0)
        self.arm_move_group.set_named_target('test_home')
        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()
        

        print("Experienced "+str(self.failures)+" failures")

if __name__ == '__main__':
    move = GoToTarget()
    move.experiment()