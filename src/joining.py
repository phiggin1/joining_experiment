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
from joining_experiment.msg import JoinPose
from joining_experiment.srv import JoiningServo, JoiningServoResponse
import time

def quaternion_from_msg(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]

class GoToTarget:
    def __init__(self):
        self.stop = False
        rospy.init_node('joining_exp')

        self.grab = rospy.Publisher('buttons', String, queue_size=10)

        self.is_sim = rospy.get_param("~rivr", False)
        print(self.is_sim)

        self.rivr_robot_speech = rospy.Publisher('/robotspeech', String, queue_size=10)

        self.interactive = True

        self.target_topic = "/target/target"

        self.retry_times = 10      

        self.finger_full_open = 0.0
        self.finger_open = 1.1
        self.finger_Full_closed = 1.3

        self.hand_open = [self.finger_open, self.finger_open, self.finger_full_open]
        self.hand_closed = [self.finger_Full_closed, self.finger_Full_closed, self.finger_full_open]

        self.standoff_distance = 0.15       #m
        self.hand_finger_offset_x = 0.0     #0.035   #m
        self.hand_finger_offest_y = 0.0     #0.01    #m
        self.hand_finger_offest_z = 0.2     #m
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

        self.intial_pose = PoseStamped()
        self.intial_pose.header.frame_id = "base_link"
        self.intial_pose.pose.position.x =  0.55
        self.intial_pose.pose.position.y = -0.08
        self.intial_pose.pose.position.z =  0.95
        self.intial_pose.pose.orientation.x = 0.0
        self.intial_pose.pose.orientation.y = 0.0
        self.intial_pose.pose.orientation.z = -0.707
        self.intial_pose.pose.orientation.w = 0.707

        self.listener = tf.TransformListener()
        
        if self.is_sim:
            joint_state_topic = ['joint_states:=/joint_states']
        else:
            joint_state_topic = ['joint_states:=/j2n6s300/joint_states']
        
        moveit_commander.roscpp_initialize(joint_state_topic)

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
        print("Saying: " + str)
        if self.is_sim:
            wav = festival.textToWav(str)
            data = sf.read(wav)
            string_msg =json.dumps(list(data[0]))
            self.rivr_robot_speech.publish(string_msg)
        else:
            festival.sayText(str)

    def get_target(self):
        count = 0
        target = None
        near = False
        see_anode = False
        see_cathode = False
        last_time_spoke = None
        while (target is None or not near or not see_anode or not see_cathode) and count < self.retry_times:
            try:
                target = rospy.wait_for_message(self.target_topic, JoinPose, timeout=5.0)
                near = target.near.data
                see_anode = target.see_anode.data
                see_cathode = target.see_cathode.data

                if not near and (last_time_spoke is None or rospy.Time.now().to_sec() > last_time_spoke+5.0):
                    self.talk("Can you please move the putty closer together?")
                    last_time_spoke = rospy.Time.now().to_sec()
                    
                if not see_anode and (last_time_spoke is None or rospy.Time.now().to_sec() > last_time_spoke+5.0):
                    self.talk("Can you please move the green putty where I can see it?")
                    last_time_spoke = rospy.Time.now().to_sec()

                if not see_cathode and (last_time_spoke is None or rospy.Time.now().to_sec() > last_time_spoke+5.0):
                    self.talk("Can you please move the red putty where I can see it?")
                    last_time_spoke = rospy.Time.now().to_sec()

            except rospy.exceptions.ROSException:
                rospy.loginfo("Timeout waiting for target")
                count+=1
                if count>= self.retry_times:
                    if raw_input("Keep retrying (y/n)?") == 'y':
                        count = 0
        
        if target is not None:
            t = rospy.Time.now()
            target.header.stamp = t
            self.listener.waitForTransform(target.header.frame_id, self.planning_frame, t, rospy.Duration(4.0) )
            goal_pose = self.listener.transformPose(self.planning_frame, target)

            goal_pose.pose.position.z += self.hand_finger_offest_z

            return goal_pose

    def move_arm(self, pose, speed):
        self.arm_move_group.set_max_velocity_scaling_factor(speed)
        self.arm_move_group.set_pose_target(pose.pose)
        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()

    def move_fingers(self, finger_positions):
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
        print('open fingers')
        self.move_fingers(self.hand_open)

        
        #LED = l e d
        if self.interactive:
            self.talk("Can you please put the l e d between my fingers? The shorter lead should be on your left.")
        
        #wait for LED to be given then close hand
        if self.interactive:
            raw_input("Hand over led, Press Enter to continue...")
        
        self.grab.publish("grabbed")
        print('closed fingers')
        self.move_fingers(self.hand_closed)
              

        #move to intial positon above
        self.move_arm(self.intial_pose, 1.0)

        
        #give instructions
        if self.interactive:
            self.talk("place the lead of the red wire into the red putty")
            raw_input("Press Enter to continue...")

            self.talk("place the lead of the black wire into the green putty")
            raw_input("Press Enter to continue...")

            self.talk("can you hold the two pieces of putty up in front of me?")
            raw_input("Press Enter to continue...")
        
        
        reached_target = False
        while not reached_target:
            if self.interactive:
                raw_input("waiting for user to present, Press Enter to continue...")

            standoff_pose = self.get_target()
            standoff_pose.pose.position.z += self.standoff_distance

            print('standoff')
            print(standoff_pose.pose.position)
            
            #if self.interactive:
            #    raw_input("Move to standoff, Press Enter to continue...")

            self.move_arm(standoff_pose, 1.0)
               
            #if self.interactive:
            #    i = raw_input("Move to final, Press Enter to continue...")
            
            goal_pose = self.get_target()
            self.move_arm(goal_pose, 1.0)
            print('standoff')
            print(goal_pose.pose.position)

            #print(self.servo())

            
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
            

            #time.sleep(10.0)

            print('standoff')
            print(standoff_pose.pose.position)
            self.move_arm(standoff_pose, 1.0)

            #time.sleep(10.0)
            
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