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
effector = "j2n6s300_end_effector"
base = "root"
finger_tip_offset = 0.0425 



rospy.sleep(5)
tf_listener = tf.TransformListener()
tf_broadcase = tf.TransformBroadcaster()
rospy.sleep(5)



while not rospy.is_shutdown():
    now = rospy.Time.now()
    tf_listener.waitForTransform(effector, finger1, now, rospy.Duration(4.0))
    f1_tf = tf_listener.lookupTransform(effector, finger1, now)
    f2_tf = tf_listener.lookupTransform(effector, finger2, now)

    pose = PoseStamped()
    pose.header.frame_id = effector
    pose.header.stamp = now
    pose.pose.position.x = (f1_tf[0][0]+f2_tf[0][0])/2.0 
    pose.pose.position.y = (f1_tf[0][1]+f2_tf[0][1])/2.0 
    pose.pose.position.z = (f1_tf[0][2]+f2_tf[0][2])/2.0 - finger_tip_offset
    pose.pose.orientation.w = 1.0
    #print(pose)

    pose_pub.publish(pose)

    translation = (pose.pose.position.x,
                   pose.pose.position.y,
                   pose.pose.position.z)
    rotation = (pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)

    tf_broadcase.sendTransform(translation, rotation, rospy.Time.now(), 'j2n6s300_link_finger_tip', effector)
