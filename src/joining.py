#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from visualization_msgs.msg import Marker

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

        self.rivr_robot_speech = rospy.Publisher('/text_to_speech', String, queue_size=10)

        self.interactive = True

        self.target_topic = "/target/target"
        self.target_sub = rospy.Subscriber(self.target_topic , JoinPose, self.get_target)

        self.retry_times = 10      

        self.finger_open = 1.0
        self.finger_partial_closed = 0.68
        self.finger_closed = 0.74

        self.speech_delay = 8.0

        self.hand_open = [self.finger_partial_closed, self.finger_partial_closed, self.finger_full_open]
        self.hand_closed = [self.finger_full_closed, self.finger_full_closed, self.finger_full_open]

        self.standoff_distance = 0.35       #m
        self.goal_tolerance = 0.0025        #m

        self.target = None
        self.last_time_spoke = rospy.Time.now().to_sec()
        self.presented = False

        self.hand_over_pose = PoseStamped()
        self.hand_over_pose.header.frame_id = "base_link"
        self.hand_over_pose.pose.position.x =  0.55
        self.hand_over_pose.pose.position.y = -0.08
        self.hand_over_pose.pose.position.z =  0.95
        self.hand_over_pose.pose.orientation.x = -0.5
        self.hand_over_pose.pose.orientation.y = -0.5
        self.hand_over_pose.pose.orientation.z =  0.5
        self.hand_over_pose.pose.orientation.w =  0.5

        self.hand_over_pose_retreat = PoseStamped()
        self.hand_over_pose_retreat.header.frame_id = "base_link"
        self.hand_over_pose_retreat.pose.position.x =  0.50
        self.hand_over_pose_retreat.pose.position.y = -0.08
        self.hand_over_pose_retreat.pose.position.z =  0.95
        self.hand_over_pose_retreat.pose.orientation.x = -0.5
        self.hand_over_pose_retreat.pose.orientation.y = -0.5
        self.hand_over_pose_retreat.pose.orientation.z =  0.5
        self.hand_over_pose_retreat.pose.orientation.w =  0.5

        self.intial_pose = PoseStamped()
        self.intial_pose.header.frame_id = "base_link"
        self.intial_pose.pose.position.x =  0.48
        self.intial_pose.pose.position.y = -0.18
        self.intial_pose.pose.position.z =  1.19
        self.intial_pose.pose.orientation.x = 0.0
        self.intial_pose.pose.orientation.y = 0.0
        self.intial_pose.pose.orientation.z = 0.707
        self.intial_pose.pose.orientation.w = 0.707

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

        '''
        rospy.sleep(2)
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.13
        table_pose.pose.orientation.w = 1.0
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(5.0, 5.0, 1.0))
        print(table_name, self.wait_for_scene_update(table_name, 4))

        person_pose = PoseStamped()
        person_pose.header.frame_id = "base_link"
        person_pose.pose.position.x = 1.6
        person_pose.pose.position.y = 0.0
        person_pose.pose.position.z = 0.0
        person_pose.pose.orientation.w = 1.0
        person_name = "person"
        self.scene.add_box(person_name, person_pose, size=(1.0, 5.0, 5.0))
        print(person_name, self.wait_for_scene_update(person_name, 4))
        '''

    def wait_for_scene_update(self, name, timeout):
        start = rospy.get_time()

        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
                is_known = name in self.scene.get_known_object_names()
                if is_known:
                    return True
                rospy.sleep(0.1)
                seconds = rospy.get_time()

        return False

    def talk(self, text):
            print("Saying rivr: " + text)
            self.rivr_robot_speech.publish(text)
            repeat = input("Repeat (y/n): ")
            while (repeat == 'y'):
                self.rivr_robot_speech.publish(string_msg)
                repeat = input("Repeat (y/n): ")

    def get_target(self, target):
        near = target.near.data
        too_close = target.too_close.data
        see_anode = target.see_anode.data
        see_cathode = target.see_cathode.data
        in_workspace = target.in_workspace.data
        directions = target.move_direction.split(' ')

        now = rospy.Time.now().to_sec()
        if not see_anode and not see_cathode and self.presented and now > self.last_time_spoke+self.speech_delay: 
            self.valid_target = False
            self.talk("Can you please move the red and green putty to where I can see tem?")
            self.last_time_spoke = now
        elif not see_anode and self.presented and now > self.last_time_spoke+self.speech_delay: 
            self.valid_target = False
            self.talk("Can you please move the red putty where I can see it?")
            self.last_time_spoke = now
        elif not see_cathode and self.presented and now > self.last_time_spoke+self.speech_delay: 
            self.valid_target = False
            self.talk("Can you please move the green putty where I can see it?")
            self.last_time_spoke = now
        elif not near and self.presented and now > self.last_time_spoke+self.speech_delay: 
            self.valid_target = False
            self.talk("Can you please move the putty closer together?")
            self.last_time_spoke = now
        elif not too_close and self.presented and now > self.last_time_spoke+self.speech_delay: 
            self.valid_target = False
            self.talk("Can you please move the putty a little further apart?")
            self.last_time_spoke = now
        elif not in_workspace and self.presented and now > self.last_time_spoke+self.speech_delay: 
            self.valid_target = False
            text  = "Can you please move the putty "
            for i in len(directions):
                text += directions[i]
                if i < len(directions)-1:
                    text += " and "
            self.talk(text)
            self.last_time_spoke = now
        else:
            t = rospy.Time.now()
            target.header.stamp = t
            self.listener.waitForTransform(target.header.frame_id, self.planning_frame, t, rospy.Duration(4.0) )
            self.target = self.listener.transformPose(self.planning_frame, target)  
            self.last_valid_target = rospy.time.now()  
            self.valid_target = True

    def get_init_target(self):
        count = 0
        rate = rospy.rate(1)
        while not self.valid_target and count < self.retry_times and not rospy.is_shutdown():
            count += 1
            rate.sleep()
        
        return self.target

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
        print('hand over pose')
        self.move_arm(self.hand_over_pose, 1.0)
        
        print('open hand')
        self.move_fingers(self.hand_open)


        self.talk("Can you please put the l e d between my fingers? The shorter lead should be on your right.")
        

        #give acknowledgement?
        self.grab.publish("grabbed")
        self.move_fingers(self.hand_closed)
        print('\nclosed hand')
        self.talk("Thank you")

        #move to retreat
        self.move_arm(self.hand_over_pose_retreat, 1.0)
    
        #move to intial positon above
        self.move_arm(self.intial_pose, 1.0)
        

        self.talk("Can you please place the lead of the red wire into the red putty")
        input("\nPress Enter to continue...")

        self.talk("Can you place the lead of the black wire into the green putty")
        input("\nPress Enter to continue...")

        self.talk("Can you hold the two pieces of putty up in front of me?")
        input("\nPress Enter to continue...")
        
        reached_target = False
        while not reached_target:
            input("\nWaiting for user to present, Press Enter to continue...")

            self.presented = True
            standoff_pose = self.get_init_target()
            self.presented = False
            standoff_pose.pose.position.z += self.standoff_distance

            print('standoff')
            print(standoff_pose.pose.position)
            
            self.move_arm(standoff_pose, 1.0)
            
            rospy.loginfo('pre servo')
            self.presented = True
            print(self.servo())
            self.presented = False
            rospy.loginfo('post servo')
            
            #check if should open hand
            self.talk("did the L E D light up")
            i = input("Open hand (y/n) ")
            if i == 'y':
                print('open hand')
                self.grab.publish("released")
                self.move_fingers(self.hand_open)
                #self.talk("thank you")
                reached_target = True
            else:
                self.talk("i will try again")
                self.failures += 1
            
            print('standoff')
            print(standoff_pose.pose.position)
            self.move_arm(standoff_pose, 1.0)
            
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
