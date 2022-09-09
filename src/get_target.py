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

        #cathode = negative black terminal (green putty)
        cath_min_r = rospy.get_param("cath_min_r", 0)
        cath_max_r = rospy.get_param("cath_max_r", 64)
        cath_min_g = rospy.get_param("cath_min_g", 80)
        cath_max_g = rospy.get_param("cath_max_g", 255)
        cath_min_b = rospy.get_param("cath_min_b", 0)
        cath_max_b = rospy.get_param("cath_max_b", 64)

        #anode = positive red terminal (red putty)
        an_min_r = rospy.get_param("an_min_r", 100)
        an_max_r = rospy.get_param("an_max_r", 255)
        an_min_g = rospy.get_param("an_min_g", 0)
        an_max_g = rospy.get_param("an_max_g", 64)
        an_min_b = rospy.get_param("an_min_b", 0)
        an_max_b = rospy.get_param("an_max_b", 64)

        #Min and max distance to consider for anode cathode positions in mm
        self.min_depth = rospy.get_param("min_depth", 400)
        self.max_depth = rospy.get_param("max_depth", 1200)

        #width of the led in m
        self.led_width = rospy.get_param("led_width", 15.0/1000.0)
        self.putty_width = rospy.get_param("putty_width", 50.0/1000.0)

        self.debug = rospy.get_param("debug", True) 

        # Threshold of anode (red) in BGR space
        self.anode_min = np.array([an_min_b, an_min_g, an_min_r])
        self.anode_max = np.array([an_max_b, an_max_g, an_max_r])

        # Threshold of cathode (green) in BGR space
        self.cathode_min = np.array([cath_min_b, cath_min_g, cath_min_r])
        self.cathode_max = np.array([cath_max_b, cath_max_g, cath_max_r])

        self.depth_cam_info = rospy.wait_for_message("/camera/unityrgb/camera_info", CameraInfo, timeout=None)
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.depth_cam_info)

        self.img_pub = rospy.Publisher('/target/image_raw', Image, queue_size=10)
        self.anode_pc_pub = rospy.Publisher('/target/pc_a', PointCloud2, queue_size=10)
        self.cathode_pc_pub = rospy.Publisher('/target/pc_c', PointCloud2, queue_size=10)

        self.target_pub = rospy.Publisher('/target/target', PoseStamped, queue_size=10)

        self.rgb_image_sub = message_filters.Subscriber('/camera/unityrgb/image_raw', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/unitydepth/image_raw', Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 10, slop=2.0)
        self.ts.registerCallback(self.callback)

        rospy.spin()

    # Define a function to show the image in an OpenCV Window
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)

    def callback(self, rgb_ros_image, depth_ros_image):
        rgb = np.asarray(self.bridge.imgmsg_to_cv2(rgb_ros_image, desired_encoding="passthrough"))
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_ros_image, desired_encoding="passthrough"))

        # preparing the mask to overlay
        anode_image = remove_isolated_pixels( cv2.inRange(rgb, self.anode_min, self.anode_max) )
        cathode_image = remove_isolated_pixels( cv2.inRange(rgb, self.cathode_min, self.cathode_max) )


        anode_depth_masked = cv2.bitwise_and(depth, depth, mask=anode_image)
        cathode_depth_masked = cv2.bitwise_and(depth, depth, mask=cathode_image)

        points_anode = self.get_pointcloud(anode_depth_masked)
        points_cathode = self.get_pointcloud(cathode_depth_masked)
        
        if self.debug:
            print(len(points_anode), len(points_cathode))
            anode_masked = cv2.bitwise_and(rgb, rgb, mask=anode_image)
            cathode_masked = cv2.bitwise_and(rgb, rgb, mask=cathode_image)
            img = anode_masked+cathode_masked
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="passthrough"))
            self.anode_pc_pub.publish(pc2.create_cloud_xyz32(depth_ros_image.header, points_anode))
            self.cathode_pc_pub.publish(pc2.create_cloud_xyz32(depth_ros_image.header, points_cathode))

            #self.show_image(img)
        
        try:
            an_cent = self.get_centroid(points_anode)
            cath_cent = self.get_centroid(points_cathode)
            a = (an_cent[0] - cath_cent[0], an_cent[1] - cath_cent[1], an_cent[2] - cath_cent[2])

            pa = an_cent
            pc = cath_cent
            d = dist(an_cent, cath_cent)

            '''
            d, pa, pc = self.get_closest_points(points_anode, points_cathode)
            #a = (pa[0]-pc[0], pa[1]-pc[1], pa[2]-pc[2])
            '''
            b = (0,0,1)

            #yaw (in kinect frame around y axis)
            theta = np.arccos( dot(a,b)/(norm(a)*norm(b)) )
            if (a[0] < 0):
                theta = -theta

            quat = quaternion_from_euler(math.pi/2.0, theta-(math.pi/2.0), 0.0 )

            target = PoseStamped()
            target.header = depth_ros_image.header
            target.pose.position.x = (pa[0]+pc[0])/2.0
            target.pose.position.y = (pa[1]+pc[1])/2.0 
            target.pose.position.z = (pa[2]+pc[2])/2.0
            target.pose.orientation.x = quat[0]
            target.pose.orientation.y = quat[1]
            target.pose.orientation.z = quat[2]
            target.pose.orientation.w = quat[3]
        
            rospy.loginfo('=====================')
            #rospy.loginfo(pa)
            #rospy.loginfo(pc)
            #rospy.loginfo(an_cent)
            #rospy.loginfo(cath_cent)
            #rospy.loginfo(theta*180/np.pi)
            rospy.loginfo("positon:     x:%.4f\ty:%.4f\tz:%.4f" % (target.pose.position.x, target.pose.position.y, target.pose.position.z) )
            rospy.loginfo("orientation: x:%.4f\ty:%.4f\tz:%.4f\tw:%.4f" % (target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w))

            self.target_pub.publish(target)

            if d > self.led_width+2*self.putty_width:
                rospy.loginfo(d)
                rospy.loginfo(self.led_width+2*self.putty_width)
                rospy.loginfo("Anode and cathode to far away")

        except (ZeroDivisionError, TypeError): 
            rospy.loginfo("Empty anode or cathode pointcloud")


    def get_pointcloud(self, depth_masked):
        point_list = []
        for r in range(0, depth_masked.shape[0], 4):
            for c in range(0, depth_masked.shape[1], 4):
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

    def get_centroid(self, points):
        cent_x = 0.0
        min_y = 9990.0
        cent_y = 0.0
        max_z = 0.0
        cent_z = 0.0
        count = 0
        for p in points:
            cent_x += p[0]
            cent_y += p[1]
            cent_z += p[2]
            if p[1] < min_y:
                min_y = p[1]
            if p[2] > max_z:
                max_z = p[2]
            count += 1

        cent_x = cent_x/count
        cent_y = min_y#cent_y/count
        cent_z = cent_z/count #max_z

        return [cent_x,cent_y,cent_z]

    def get_closest_points(self, cloud_a, cloud_b):
        min_d = 99999.9
        closest_a = None
        closest_b = None
        for pa in cloud_a:
            for pb in cloud_b:
                d = dist(pa, pb)
                if d < min_d:
                    min_d = d
                    closest_a = pa
                    closest_b = pb

        return (min_d, closest_a, closest_b)

if __name__ == '__main__':
    get_targets = GetTargetPose()