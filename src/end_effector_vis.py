#!/usr/bin/env python2
import rospy
from visualization_msgs.msg import Marker


def get_marker():
    marker = Marker()

    marker.id = 0
    marker.header.frame_id = "j2n6s300_link_6"
    marker.header.stamp = rospy.Time.now()

    marker.type = marker.SPHERE

    marker.action = marker.ADD

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0        
    marker.color.a = 0.5

    marker.scale.x = 0.015
    marker.scale.y = 0.015
    marker.scale.z = 0.015

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    return marker

pub_end = rospy.Publisher('/test/j2n6s300_end_effector_marker', Marker, queue_size=10)
pub_fing = rospy.Publisher('/test/j2n6s300_finger_marker', Marker, queue_size=10)
rospy.init_node('j2n6s300_end_effector_marker')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    hand_finger_offset_x = 0.035   #m
    hand_finger_offest_y = 0.02    #m
    hand_finger_offest_z = 0.20    #m
    e = get_marker()
    f = get_marker()
    f.color.r = 0.0
    #f.pose.position.x += hand_finger_offest_y
    #f.pose.position.y += -hand_finger_offset_x
    f.pose.position.z += -hand_finger_offest_z
    pub_end.publish(e)
    pub_fing.publish(f)
    r.sleep()
