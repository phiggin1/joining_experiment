#!/usr/bin/env python2
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import festival
import soundfile as sf
import json
from joining_experiment.srv import JoiningServo, JoiningServoResponse

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
        self.rivr_robot_speech = rospy.Publisher('/robotspeech', String, queue_size=10)

        self.interactive = True

        self.target_topic = "/target/target"
        self.retry_times = 10           
        self.finger_full_open = 0.0
        self.finger_open = 1.1
        self.finger_Full_closed = 1.3

        self.hand_open = [self.finger_open, self.finger_open, self.finger_full_open]
        self.hand_closed = [self.finger_Full_closed, self.finger_Full_closed, self.finger_full_open]

        self.standoff_distance = 0.15      #m
        self.hand_finger_offset_x = 0.0#0.035   #m
        self.hand_finger_offest_y = 0.0#0.01    #m
        self.hand_finger_offest_z = 0.2   #m
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

    def talk(self, str):
        print(str)
        wav = festival.textToWav(str)
        data = sf.read(wav)
        string_msg =json.dumps(list(data[0]))
        self.rivr_robot_speech.publish(string_msg)

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

        print('init goal')
        print(goal_pose.pose.position.z)
        print(self.hand_finger_offest_z)        
        goal_pose.pose.position.z += self.hand_finger_offest_z
        print('finger offset goal')
        print(goal_pose.pose.position.z)
        print('============')
        
        return goal_pose

    def move_arm(self, pose, speed):
        self.arm_move_group.set_max_velocity_scaling_factor(speed)
        self.arm_move_group.set_pose_target(pose.pose)
        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()


    def move_fingers(self, finger_positions):
        current_joints = self.hand_move_group.get_current_joint_values()
        print(current_joints)
        print(finger_positions)
        self.hand_move_group.go(finger_positions, wait=True)

    def servo(self):
        rospy.wait_for_service('JoiningServo')
        try:
            joining_servo = rospy.ServiceProxy('JoiningServo', JoiningServo)
            resp = joining_servo()
            return resp.resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def experiment(self):
        self.failures = 0
        
        
        #move to handover position
        self.move_arm(self.hand_over_pose, 1.0)
        
        self.move_fingers(self.hand_open)

        #LED = l e d
        if self.interactive:
            self.talk("Can you please put the l e d between my fingers? The shorter lead should be on your left.")
        
        #wait for LED to be given then close hand
        if self.interactive:
            raw_input("Hand over led, Press Enter to continue...")
        self.grab.publish("grabbed")
        self.move_fingers(self.hand_closed)
                

        #re home the arm
        self.arm_move_group.set_max_velocity_scaling_factor(1.0)
        self.arm_move_group.set_named_target('test_home')
        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()
        
        #give instructions
        if self.interactive:
            s = "place the lead of the red wire into the red putty"
            self.talk(s)
            raw_input("Press Enter to continue...")

            s = "place the lead of the black wire into the green putty"
            self.talk(s)
            raw_input("Press Enter to continue...")

            s = "can you hold the two pieces of putty up in front of me?"
            self.talk(s)
            raw_input("Press Enter to continue...")
        

        reached_target = False
        while not reached_target:

            if self.interactive:
                raw_input("waiting for user to present, Press Enter to continue...")

            goal_pose = self.get_target()
            print('goal')
            print(goal_pose.pose.position)

            goal_pose.pose.position.z += self.standoff_distance
            print('goal standoff')
            print(goal_pose.pose.position)
            
            if self.interactive:
                raw_input("Move to standoff, Press Enter to continue...")
            self.move_arm(goal_pose, 1.0)
            


            if self.interactive:
                i = raw_input("Move to final, Press Enter to continue...")
            self.servo()

            
            #check if should open hand
            if self.interactive:
                i = raw_input("Open hand (y/n) ")
                if i == 'y':
                    print('open hand')
                    self.grab.publish("released")
                    self.move_fingers(self.hand_open)
                    reached_target = True
                else:
                    self.failures += 1
            
            print('goal standoff pose')
            print(goal_pose.pose.position)
            self.move_arm(goal_pose, 1.0)
            
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