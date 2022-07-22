#!/usr/bin/env python2

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker

import numpy as np
from copy import deepcopy
from tf import TransformListener
import image_geometry
import math

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

    marker.pose.orientation.x = p.pose.orientation.x
    marker.pose.orientation.y = p.pose.orientation.y
    marker.pose.orientation.z = p.pose.orientation.z
    marker.pose.orientation.w = p.pose.orientation.w

    marker.pose.position.x = p.pose.position.x
    marker.pose.position.y = p.pose.position.y
    marker.pose.position.z = p.pose.position.z

    return marker

class TargetFilter:
    def __init__(self):
        self.min_val = 450
        self.max_val = 1200
        self.delta = self.max_val-self.min_val

        self.min_depth = 600
        self.max_depth = 1100

        self.finger1 = 'j2n6s300_link_finger_tip_1'
        self.finger2 = 'j2n6s300_link_finger_tip_2'
        self.finger3 = 'j2n6s300_link_finger_tip_3'

        rospy.init_node('detect', anonymous=True)

        self.depth_cam_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo, timeout=None)
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.depth_cam_info)

        self.tf = TransformListener()
        
        self.bridge = CvBridge()
        
        self.target_pub = rospy.Publisher("/testing/image_raw", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, self.callback)
        
        self.finger_marker_pub = rospy.Publisher("/target/finger_marker", PointStamped, queue_size=10)
        self.finger_pc_pub = rospy.Publisher('/target/finger', PointCloud2, queue_size=10)


        rospy.spin()    

    # Define a function to show the image in an OpenCV Window
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)

    def get_centroid(self, points):
        cent_x = 0.0
        cent_y = 0.0
        max_z = 0.0
        cent_z = 0.0
        count = 0
        for p in points:
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

    def get_pointcloud(self, depth_masked):
        point_list = []
        for r in range(depth_masked.shape[0]):
            for c in range(depth_masked.shape[1]):
                if depth_masked[r,c]>self.min_depth and depth_masked[r,c]<self.max_depth :
                    d = depth_masked[r,c]/1000.0
                    cx = self.cam_model.cx()
                    cy = self.cam_model.cy()
                    fx = self.cam_model.fx()
                    fy = self.cam_model.fy()

                    x = (c -cx)*d/fx
                    y = (r -cy)*d/fy
                    z = d

                    point_list.append([x,y,z])
                    

        return point_list

    def callback(self, ros_img):
        src = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")

        t = self.tf.getLatestCommonTime(ros_img.header.frame_id, self.finger1)
        fing_1, q = self.tf.lookupTransform(ros_img.header.frame_id, self.finger1,  t)
        fing_2, q = self.tf.lookupTransform(ros_img.header.frame_id, self.finger2,  t)
        #fing_3, q = self.tf.lookupTransform(ros_img.header.frame_id, self.finger3,  t)

        fing_1_uv = self.cam_model.project3dToPixel( fing_1 )
        fing_2_uv = self.cam_model.project3dToPixel( fing_2 )
        #fing_3_uv = self.cam_model.project3dToPixel( fing_3 )

        padding = 25

        min_u = np.min([fing_1_uv[0],fing_2_uv[0]])-padding#,fing_3_uv[0]])-padding
        min_v = np.min([fing_1_uv[1],fing_2_uv[1]])#-padding#,fing_3_uv[1]])-padding
        max_u = np.max([fing_1_uv[0],fing_2_uv[0]])+padding#,fing_3_uv[0]])+padding
        max_v = np.max([fing_1_uv[1],fing_2_uv[1]])+padding#,fing_3_uv[1]])+padding
        
        img = np.minimum(src, self.max_val)
        img = np.maximum(img, self.min_val)

        img_float = np.float32((img-self.min_val)/float(self.delta))
        dist = np.uint8(img_float*255)#dist.astype(np.uint8)
        color = cv2.applyColorMap(dist, cv2.COLORMAP_JET)

        mask = np.zeros(src.shape[:2], dtype="uint8")
        cv2.rectangle(mask, (int(min_u), int(min_v)), (int(max_u), int(max_v)), 255, -1)
        masked_color = cv2.bitwise_and(color, color, mask=mask)

        gray = cv2.cvtColor(masked_color, cv2.COLOR_BGR2GRAY) # convert to grayscale
        blur = cv2.blur(gray, (9, 9)) # blur the image
        ret, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)
       
        # Finding contours for the thresholded image
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        print(len(contours))
        if len(contours) < 1:
            return

        min_dists  = []
        for i in range(len(contours)):
            min_dists.append(float('inf'))
        fing_u = (fing_1_uv[0]+fing_2_uv[0])/2
        fing_v = (fing_1_uv[1]+fing_2_uv[1])/2
        for i in range(len(contours)):
            for p in contours[i]:
                (u,v) = p[0]
                d = math.sqrt((u-fing_u)**2 + (v-fing_v)**2)
                if d < min_dists[i]:
                    min_dists[i] = d

        min_indx = np.argmin(min_dists)

        print(min_dists)
        print(min_indx)

        # create an empty black image
        drawing_cont = deepcopy(color)#np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)
        drawing_mask = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)
        color_min = (0, 255, 0) # green - color for contours
        color_max = (0, 0, 255) # blue - color for convex hull
        # draw contours and hull points
        for i in range(len(contours)):
            if i == min_indx:
                c = color_min
            else:
                c = color_max
            # draw ith contour
            cv2.drawContours(drawing_cont, contours, i, c, 1, 8, hierarchy, 0)
            if i == min_indx:
                cv2.drawContours(drawing_mask, contours, i, (255,255,255), -1)
      
        # Radius of circle
        radius = 3
    
        # Line thickness of 2 px
        thickness = 2
        
        # Using cv2.circle() method
        # Draw a circle with blue line borders of thickness of 2 px
        
        color = cv2.circle(color, (int(fing_1_uv[0]), int(fing_1_uv[1])), radius, (255, 0, 255), thickness)
        color = cv2.circle(color, (int(fing_2_uv[0]), int(fing_2_uv[1])), radius, (0, 255, 255), thickness)
        #color = cv2.circle(color, (int(fing_3_uv[0]), int(fing_3_uv[1])), radius, (0, 0, 255), thickness)

        # concatenate image Horizontally
        h1 = np.concatenate((color, cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)), axis=1)
        
        # concatenate image Vertically
        h2 = np.concatenate((drawing_cont, drawing_mask), axis=1)

        final = np.concatenate((h1, h2), axis=0)
        
        self.show_image(final)
        
        img_msg = self.bridge.cv2_to_imgmsg(drawing_mask, encoding="passthrough")
        img_msg.header = ros_img.header
        self.target_pub.publish(img_msg)
        

        mask = cv2.cvtColor(drawing_mask, cv2.COLOR_BGR2GRAY)
        print(mask.shape)
        print(type(mask[0][0]))
        print(src.shape)
        print(type(src[0][0]))

        depth_masked = cv2.bitwise_and(src, src, mask=mask)
        points_finger = self.get_pointcloud(depth_masked)
        cent = self.get_centroid(points_finger)
        p = PointStamped()
        p.header = ros_img.header
        p.point.x = cent[0]
        p.point.y = cent[1]
        p.point.z = cent[2]
        self.finger_marker_pub.publish(p)

        self.finger_pc_pub.publish(pc2.create_cloud_xyz32(ros_img.header, points_finger))


if __name__ == '__main__':
    get_targets = TargetFilter()