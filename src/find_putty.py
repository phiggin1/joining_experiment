#!/usr/bin/env python2

import rospy
import message_filters
import numpy as np
import math
import cv2
from cv_bridge import CvBridge
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from joining_experiment.msg import JoinPose
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def remove_isolated_pixels(image):
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

def dot(a,b):
    return (a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2])

def dist(pa, pb):
    return math.sqrt((pa[0] - pb[0])**2+(pa[1] - pb[1])**2+(pa[2] - pb[2])**2)

def norm(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

class GetTargetPose:
    def __init__(self):
        rospy.init_node('GetTargetPose', anonymous=True)
        self.bridge = CvBridge()


        self.type = rospy.get_param("~type", 'cathode')
        is_sim = rospy.get_param("~rivr", True)
        self.debug = rospy.get_param("~debug", True) 
        self.step = rospy.get_param("~step", 4)

        min_r = rospy.get_param("~min_r", 0)
        max_r = rospy.get_param("~max_r", 64)
        min_g = rospy.get_param("~min_g", 80)
        max_g = rospy.get_param("~max_g", 255)
        min_b = rospy.get_param("~min_b", 0)
        max_b = rospy.get_param("~max_b", 64)

        #Min and max distance to consider for anode/cathode positions in mm
        self.min_depth = rospy.get_param("min_depth", 0)
        self.max_depth = rospy.get_param("max_depth", 9999)

        # Threshold of anode (red) in BGR space
        self.min_color = np.array([min_b, min_g, min_r])
        self.max_color = np.array([max_b, max_g, max_r])

        rospy.loginfo("%s max r:%i max g:%i max b:%i" % (self.type, max_r, max_g, max_b))

        if is_sim:
            rospy.loginfo("virtual robot")
            self.depth_cam_info = rospy.wait_for_message("/camera/unityrgb/camera_info", CameraInfo, timeout=None)
            self.rgb_image_sub = message_filters.Subscriber('/camera/unityrgb/image_raw', Image)
            self.depth_image_sub = message_filters.Subscriber('/camera/unitydepth/image_raw', Image)
        else:
            rospy.loginfo("physical robot")
            self.depth_cam_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo, timeout=None)
            self.rgb_image_sub = message_filters.Subscriber('/kinect2/sd/image_color_rect', Image)
            self.depth_image_sub = message_filters.Subscriber('/kinect2/sd/image_depth_rect', Image)

        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.depth_cam_info)

        self.img_pub = rospy.Publisher('/target/image_'+self.type, Image, queue_size=10)
        self.pc_pub = rospy.Publisher('/target/pc_'+self.type, PointCloud2, queue_size=10)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 10, slop=2.0)
        self.ts.registerCallback(self.callback)

        rospy.spin()

    def callback(self, rgb_ros_image, depth_ros_image):
        rgb = np.asarray(self.bridge.imgmsg_to_cv2(rgb_ros_image, desired_encoding="passthrough"))
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_ros_image, desired_encoding="passthrough"))
        blur = cv2.GaussianBlur(rgb, (15, 15), 2)

        # preparing the mask to overlay
        image_mask = cv2.inRange(blur, self.min_color, self.max_color)
        depth_masked = cv2.bitwise_and(depth, depth, mask=image_mask)
        points = self.get_pointcloud(depth_masked)

        if len(points)>0:      
            x,y,z,w,h,d = self.get_centroid(points)
            rospy.loginfo("%s x: %.3f y: %.3f z: %.3f w: %.3f h: %.3f d: %.3f" % (self.type,x,y,z,w,h,d))

        else:
            rospy.loginfo("Empty "+self.type+" pointcloud")
        
        if self.debug:
            rgb_masked = cv2.bitwise_and(rgb, rgb, mask=image_mask)
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(rgb_masked, encoding="passthrough"))
            self.pc_pub.publish(pc2.create_cloud_xyz32(depth_ros_image.header, points))

    def get_pointcloud(self, depth_masked):
        point_list = []
        for r in range(0, depth_masked.shape[0], self.step):
            for c in range(0, depth_masked.shape[1], self.step):
                #if depth_masked[r,c]>self.min_depth and depth_masked[r,c]<self.max_depth:
                if depth_masked[r,c]>0.0:
                    d = depth_masked[r,c]/1000.0
                    cx = self.cam_model.cx()
                    cy = self.cam_model.cy()
                    fx = self.cam_model.fx()
                    fy = self.cam_model.fy()

                    x = (c - cx)*d/fx
                    y = (r - cy)*d/fy
                    z = d

                    point_list.append([x,y,z])

        return point_list

    def get_centroid(self, points):
        cent_x = 0.0
        min_x = 9999.0
        max_x = -9999.0

        cent_y = 0.0
        min_y = 9999.0
        max_y = -9999.0

        cent_z = 0.0
        min_z = 9999.0
        max_z = -9999.0

        count = 0
        for p in points:
            cent_x += p[0]
            cent_y += p[1]
            cent_z += p[2]

            if p[0] < min_x:
                min_x = p[0]
            if p[0] > max_x:
                max_x = p[0]

            if p[1] < min_y:
                min_y = p[1]
            if p[1] > max_y:
                max_y = p[1]

            if p[2] < min_z:
                min_z = p[2]
            if p[2] > max_z:
                max_z = p[2]

            count += 1


        cent_x = cent_x/count
        cent_y = cent_y/count 
        cent_z = cent_z/count

        w = abs(max_x - min_x)
        h = abs(max_y - min_y)
        d = abs(max_z - min_z)

        return [cent_x,cent_y,cent_z, w, h, d]

if __name__ == '__main__':
    get_targets = GetTargetPose()
