#!/bin/bash

#RIVR
#topics="/camera/unitydepth/camera_info /camera/unitydepth/image_raw /camera/unityrgb/camera_info /camera/unityrgb/image_raw /target/object_anode /target/object_cathode /my_gen3/finger_pose /target/pose_stamped /target/target /text_to_speech /tf /tf_static /my_gen3/in/cartesian_velocity /my_gen3/in/clear_faults /my_gen3/in/emergency_stop /my_gen3/in/joint_velocity /my_gen3/in/stop /my_gen3/joint_states"

#Real
topics="/camera/color/camera_info /camera/color/image_raw /camera/color/image_rect_color /camera/depth/camera_info /camera/depth/image /camera/depth_registered/sw_registered/camera_info /camera/depth_registered/sw_registered/image_rect_raw /target/object_anode /target/object_cathode /my_gen3/finger_pose /target/pose_stamped /target/target /text_to_speech /tf /tf_static /my_gen3/in/cartesian_velocity /my_gen3/in/clear_faults /my_gen3/in/emergency_stop /my_gen3/in/joint_velocity /my_gen3/in/stop /my_gen3/joint_states"

echo $topics

if [ -z "$1" ]
then
    rosbag record $topics
else
    prefix=$1
    rosbag record -o $prefix $topics
fi