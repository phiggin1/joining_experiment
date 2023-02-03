#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped



rospy.init_node('gen3_finger_pose')

pose_pub = rospy.Publisher('finger_pose', PoseStamped, queue_size=10)

effector = "tool_frame"
base = "base_link"

rospy.sleep(1)
tf_listener = tf.TransformListener()
tf_broadcase = tf.TransformBroadcaster()
rospy.sleep(1)

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    now = rospy.Time.now()

    pose = PoseStamped()
    pose.header.frame_id = effector
    pose.header.stamp = now
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.015
    pose.pose.position.z = 0.055

    pose.pose.orientation.w = 1.0

    pose_pub.publish(pose)
    #if needed a transform can be published
    '''
    translation = (pose.pose.position.x,
                   pose.pose.position.y,
                   pose.pose.position.z)
    rotation = (pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)

    tf_broadcase.sendTransform(translation, rotation, rospy.Time.now(), 'j2n6s300_link_finger_tip', effector)
    '''
    rate.sleep()
