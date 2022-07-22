#!/usr/bin/env python2
import sys
import rospy
import message_filters
import numpy as np
import math
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
from tf.transformations import quaternion_from_euler
import moveit_commander
import moveit_msgs.msg
from std_srvs.srv import Empty

from visualization_msgs.msg import Marker

def dot(a,b):
    return (a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2])

def dist(pa, pb):
    return math.sqrt((pa[0] - pb[0])**2+(pa[1] - pb[1])**2+(pa[2] - pb[2])**2)

def norm(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

def get_marker(p):
    marker = Marker()

    marker.id = 0
    marker.header.frame_id = p.header.frame_id
    marker.header.stamp = rospy.Time.now()

    marker.type = marker.SPHERE

    marker.action = marker.ADD

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0        
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


class ImageSegment:
    def __init__(self):
        self.stop = False
        rospy.init_node('image_segmentasda', anonymous=True)

        self.listener = tf.TransformListener()
        self.hand_finger_offest_height = 20.0/100.0 #cm to m
        self.hand_finger_offest_depth = 1.0/100.0   #cm to m
        
        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        arm_group_name = "right_arm"
        arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)
        
        hand_group_name = "right_hand"
        hand_move_group = moveit_commander.MoveGroupCommander(hand_group_name)

        planning_frame = arm_move_group.get_planning_frame()
        eef_link = arm_move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        self.goal_tolerance = 0.01


        self.robot = robot
        self.scene = scene

        self.arm_move_group = arm_move_group
        self.arm_move_group.set_max_velocity_scaling_factor(0.25)
        self.arm_move_group.set_goal_position_tolerance(self.goal_tolerance)

        self.hand_move_group = hand_move_group
        self.hand_move_group.set_max_velocity_scaling_factor(1.0)
        self.hand_move_group.set_goal_position_tolerance(self.goal_tolerance)

        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        
        self.planning_frame = 'base_link'
        self.anode_pc_sub = message_filters.Subscriber('/test/pc_a', PointCloud2, queue_size=10)
        self.cathode_pc_sub = message_filters.Subscriber('/test/pc_c', PointCloud2, queue_size=10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.anode_pc_sub, self.cathode_pc_sub,], 10, slop=2.0)
        self.ts.registerCallback(self.callback)

        self.target_pub = rospy.Publisher('/test/target_pos', Marker, queue_size=10)
        self.marker_pub = rospy.Publisher('/test/offset_target_pos', Marker, queue_size=10)
        rospy.spin()

    def callback(self, points_anode, points_cathode):
        if not self.stop:
            pa = self.get_centroid(points_anode)
            pc = self.get_centroid(points_cathode)
            #d, pa, pc = self.get_closest_points(points_anode, points_cathode)

            goal_point = PointStamped()
            goal_point.header = points_anode.header
            goal_point.point.x = (pa[0]+pc[0])/2.0
            goal_point.point.y = (pa[1]+pc[1])/2.0
            goal_point.point.z = (pa[2]+pc[2])/2.0


            a = (pa[0]-pc[0], pa[1]-pc[1], pa[2]-pc[2])
            b = (0,0,1)
            theta = np.arccos( dot(a,b)/(norm(a)*norm(b)) )

            print(theta)
            print(pa)
            print(pc)
            print(goal_point.point)

            self.move_arm(goal_point, theta)
            
            self.stop = True
            rospy.loginfo(self.stop)

    def get_centroid(self, points):
        cent_x = 0.0
        cent_y = 0.0
        max_z = 0.0
        cent_z = 0.0
        count = 0
        for p in pc2.read_points(points):
            cent_x += p[0]
            cent_y += p[1]
            cent_z += p[2]
            if p[2] > max_z:
                max_z = p[2]
            count += 1

        cent_x = cent_x/count
        cent_y = cent_y/count
        cent_z = max_z#cent_z/count
        return [cent_x,cent_y,cent_z]

    def get_closest_points(self, cloud_a, cloud_b):
        min_d = 99999.9
        closest_a = None
        closest_b = None
        for pa in pc2.read_points(cloud_a):
            for pb in pc2.read_points(cloud_b):
                d = dist(pa, pb)
                if d < min_d:
                    min_d = d
                    closest_a = pa
                    closest_b = pb

        return (min_d, closest_a, closest_b)
                    

    def move_arm(self, req, theta):
        
        clear = self.clear_octomap()
        current_pose = self.arm_move_group.get_current_pose().pose
        target = PoseStamped()

        target.header = req.header
        target.pose.position = req.point

        t = rospy.Time.now()
        req.header.stamp = t
        self.listener.waitForTransform(req.header.frame_id, self.planning_frame, t, rospy.Duration(20.0) )
        goal_pose = self.listener.transformPose(self.planning_frame, target)
        
        quat = quaternion_from_euler(0.0,0.0,theta)
        
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        
        m = get_marker(goal_pose)
        self.target_pub.publish(m)

        offset_m = get_marker(goal_pose)
        offset_m.color.b = 0.0
        offset_m.color.r = 1.0
        offset_m.pose.position.x -= 0.035
        offset_m.pose.position.y -= self.hand_finger_offest_depth
        offset_m.pose.position.z += self.hand_finger_offest_height+.05

        self.marker_pub.publish(offset_m)
        goal_pose.pose.position.x -= 0.035
        goal_pose.pose.position.y -= self.hand_finger_offest_depth
        goal_pose.pose.position.z += self.hand_finger_offest_height+.05
        print(goal_pose.pose)
        self.arm_move_group.set_max_velocity_scaling_factor(1.0)
        self.arm_move_group.set_pose_target(goal_pose.pose)
        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()


        goal_pose.pose.position.z -= .05
        print(goal_pose.pose)
        self.arm_move_group.set_max_velocity_scaling_factor(0.20)
        self.arm_move_group.set_pose_target(goal_pose.pose)
        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()
        
        #current_pose = self.arm_move_group.get_current_pose().pose


        
        

if __name__ == '__main__':
    segmenter = ImageSegment()
