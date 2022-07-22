#!/usr/bin/env python2
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

def get_marker(frame):
    marker = Marker()

    marker.id = 0
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()

    marker.type = marker.SPHERE

    marker.action = marker.ADD

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0        
    marker.color.a = 1.0

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

rospy.init_node('j2n6s300_finger_marker')

marker_pub = rospy.Publisher('/test/finger_maker', Marker, queue_size=10)
pose_pub = rospy.Publisher('/test/finger_pose', PoseStamped, queue_size=10)

finger1 = "j2n6s300_link_finger_tip_1"
finger2 = "j2n6s300_link_finger_tip_2"
effector = "j2n6s300_link_6"
base = "base_link"
finger_tip_offset = 0.0425 

rospy.sleep(5)
tf = tf.TransformListener()
rospy.sleep(5)

while not rospy.is_shutdown():
    now = rospy.Time.now()
    tf.waitForTransform(effector, finger1, now, rospy.Duration(4.0))
    f1_tf = tf.lookupTransform(effector, finger1, now)

    now = rospy.Time.now()
    tf.waitForTransform(effector, finger2, now, rospy.Duration(4.0))
    f2_tf = tf.lookupTransform(effector, finger2, now)

    now = rospy.Time.now()
    tf.waitForTransform(base, effector, now, rospy.Duration(4.0))
    eff_tf = tf.lookupTransform(base, effector, now)    

    print(f1_tf[0])
    print(f2_tf[0])
    print(eff_tf[0])
    print(eff_tf[1])

    m = get_marker(effector)

    m.pose.position.x = (f1_tf[0][0]+f2_tf[0][0])/2.0 
    m.pose.position.y = (f1_tf[0][1]+f2_tf[0][1])/2.0 
    m.pose.position.z = (f1_tf[0][2]+f2_tf[0][2])/2.0 - finger_tip_offset

    print(m.pose.position)

    marker_pub.publish(m)

    pose = PoseStamped()

    pose.header.frame_id = base
    pose.pose.position = m.pose.position
    '''
    pose.pose.position.x -= eff_tf[0][0]
    pose.pose.position.y -= eff_tf[0][1]
    pose.pose.position.z -= eff_tf[0][2]
    '''
    pose.pose.orientation.x = eff_tf[1][0]
    pose.pose.orientation.y = eff_tf[1][1]
    pose.pose.orientation.z = eff_tf[1][2]
    pose.pose.orientation.w = eff_tf[1][3]

    pose_pub.publish(pose)