#!/usr/bin/env python2
import rospy
import tf
from geometry_msgs.msg import PoseStamped



rospy.init_node('j2n6s300_finger_pose')

pose_pub = rospy.Publisher('/test/finger_pose', PoseStamped, queue_size=10)

finger1 = "j2n6s300_link_finger_tip_1"
finger2 = "j2n6s300_link_finger_tip_2"
#effector = "j2n6s300_end_effector"
effector = "j2n6s300_link_6"
base = "base_link"
finger_tip_offset = 0.045

rospy.sleep(1)
tf_listener = tf.TransformListener()
tf_broadcase = tf.TransformBroadcaster()
rospy.sleep(1)

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    now = rospy.Time.now()
    tf_listener.waitForTransform(effector, finger1, now, rospy.Duration(4.0))
    #f1_tf = tf_listener.lookupTransform(effector, finger1, now)
    #f2_tf = tf_listener.lookupTransform(effector, finger2, now)

    pose = PoseStamped()
    pose.header.frame_id = effector
    pose.header.stamp = now
    pose.pose.position.x = 0.0 #(f1_tf[0][0]+f2_tf[0][0])/2.0 
    pose.pose.position.y = 0.0 #(f1_tf[0][1]+f2_tf[0][1])/2.0 
    pose.pose.position.z = -.2 #(f1_tf[0][2]+f2_tf[0][2])/2.0 - finger_tip_offset
    pose.pose.orientation.w = 1.0

    pose_pub.publish(pose)

    translation = (pose.pose.position.x,
                   pose.pose.position.y,
                   pose.pose.position.z)
    rotation = (pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)

    tf_broadcase.sendTransform(translation, rotation, rospy.Time.now(), 'j2n6s300_link_finger_tip', effector)

    rate.sleep()
