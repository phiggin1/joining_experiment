#!/usr/bin/env python2
import sys
import rospy
import message_filters
import numpy as np
import cv2
from cv_bridge import CvBridge
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2




class ImageSegment:
    def __init__(self):
        rospy.init_node('image_segmentasda', anonymous=True)
        self.bridge = CvBridge()

        self.depth_cam_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo, timeout=None)

        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.depth_cam_info)

        self.img_pub = rospy.Publisher('/test/image_raw', Image, queue_size=10)
        self.anode_pc_pub = rospy.Publisher('/test/pc_a', PointCloud2, queue_size=10)
        self.cathode_pc_pub = rospy.Publisher('/test/pc_c', PointCloud2, queue_size=10)

        self.rgb_image_sub = message_filters.Subscriber('/kinect2/sd/image_color_rect', Image)
        self.depth_image_sub = message_filters.Subscriber('/kinect2/sd/image_depth_rect', Image)
        self.anode_image_sub = message_filters.Subscriber('/anode_filter/image', Image)
        self.cathode_image_sub = message_filters.Subscriber('/cathode_filter/image', Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub, self.anode_image_sub, self.cathode_image_sub], 10, slop=2.0)
        self.ts.registerCallback(self.callback)

        rospy.spin()

    def callback(self, rgb_ros_image, depth_ros_image, anode_ros, cathode_ros):
        anode_image = self.remove_isolated_pixels(np.asarray(self.bridge.imgmsg_to_cv2(anode_ros, desired_encoding="passthrough")))
        cathode_image = self.remove_isolated_pixels(np.asarray(self.bridge.imgmsg_to_cv2(cathode_ros, desired_encoding="passthrough")))
        rgb = np.asarray(self.bridge.imgmsg_to_cv2(rgb_ros_image, desired_encoding="passthrough"))
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_ros_image, desired_encoding="passthrough"))

        anode_masked = cv2.bitwise_and(rgb, rgb, mask=anode_image)
        cathode_masked = cv2.bitwise_and(rgb, rgb, mask=cathode_image)

        anode_depth_masked = cv2.bitwise_and(depth, depth, mask=anode_image)
        cathode_depth_masked = cv2.bitwise_and(depth, depth, mask=cathode_image)

        #depth_masked = anode_depth_masked+cathode_depth_masked
        img = anode_masked+cathode_masked

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="passthrough"))

        points_anode   = self.get_pointcloud(anode_depth_masked)
        points_cathode  = self.get_pointcloud(cathode_depth_masked)

        self.anode_pc_pub.publish(pc2.create_cloud_xyz32(depth_ros_image.header, points_anode))
        self.cathode_pc_pub.publish(pc2.create_cloud_xyz32(depth_ros_image.header, points_cathode))        

    def get_pointcloud(self, depth_masked):
        point_list = []
        cent_x = 0.0
        cent_y = 0.0
        cent_z = 0.0
        count = 0
        for r in range(depth_masked.shape[0]):
            for c in range(depth_masked.shape[1]):
                if depth_masked[r,c]>600 and depth_masked[r,c]<1700 :
                    d = depth_masked[r,c]/1000.0
                    cx = self.cam_model.cx()
                    cy = self.cam_model.cy()
                    fx = self.cam_model.fx()
                    fy = self.cam_model.fy()

                    x = (c -cx)*d/fx
                    y = (r -cy)*d/fy
                    z = d

                    cent_x += x
                    cent_y += y
                    cent_z += z
                    count += 1

                    point_list.append([x,y,z])

        #cent_x = cent_x/count
        #cent_y = cent_y/count
        #cent_z = cent_z/count

        return point_list#, [cent_x,cent_y,cent_z]

    def remove_isolated_pixels(self, image):
        connectivity = 8

        output = cv2.connectedComponentsWithStats(image, connectivity, cv2.CV_32S)

        num_stats = output[0]
        labels = output[1]
        stats = output[2]

        new_image = image.copy()

        for label in range(num_stats):
            if stats[label,cv2.CC_STAT_AREA] == 1:
                new_image[labels == label] = 0

        return new_image


   
if __name__ == '__main__':
    segmenter = ImageSegment()
