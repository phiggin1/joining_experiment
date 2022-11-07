#!/usr/bin/env python3

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
from joining_experiment.msg import Object
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


def get_marker(x,y,z,w,h,d,frame_id,type):
    m = Marker()
    m.header.frame_id = frame_id

    m.type = Marker.CUBE
    m.action = Marker.ADD

    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z

    m.pose.orientation.w = 1.0

    m.lifetime = rospy.Duration.from_sec(1.0)
    m.scale.x = w if w>0.0 else 0.01
    m.scale.y = h if h>0.0 else 0.01
    m.scale.z = d if d>0.0 else 0.01

    m.color.a = 0.50
    if type == "anode":
        m.color.r = 255
    if type == "cathode":
        m.color.g = 255

    return m

def dot(a,b):
    return (a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2])

def dist(pa, pb):
    return math.sqrt((pa[0] - pb[0])**2+(pa[1] - pb[1])**2+(pa[2] - pb[2])**2)

def norm(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

def reject_outliers(data, m=20):
    d = np.abs(data[:,0] - np.median(data[:,0]))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.)

    return data[s < m]

class FindPutty:
    def __init__(self):
        
        
        rospy.init_node('find_putty', anonymous=True)
        self.bridge = CvBridge()

        #Get type of putty (anode/cathode)
        # used to name published topics (marker, pointcloud, object)
        self.type = rospy.get_param("~type", 'cathode')

        #Virtual robot in RIVR or phyical robot
        #   determines which topics to subscribe too
        is_sim = rospy.get_param("~rivr", True)


        self.debug = rospy.get_param("~debug", True) 

        #step size to move through depth image
        #   iterating through whole image too slow
        self.step = rospy.get_param("~step", 4)

        #Range of colors to filter on
        min_r = rospy.get_param("~min_r", 0)
        max_r = rospy.get_param("~max_r", 64)
        min_g = rospy.get_param("~min_g", 80)
        max_g = rospy.get_param("~max_g", 255)
        min_b = rospy.get_param("~min_b", 0)
        max_b = rospy.get_param("~max_b", 64)

        #Min and max distance to consider for anode/cathode positions in mm
        self.min_depth = rospy.get_param("min_depth", 0)
        self.max_depth = rospy.get_param("max_depth", 1500.0)

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
        self.marker_pub = rospy.Publisher('/target/marker_'+self.type, Marker, queue_size=10)
        self.obj_pub = rospy.Publisher('/target/object_'+self.type, Object, queue_size=10)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 10, slop=2.0)
        self.ts.registerCallback(self.callback)

        rospy.spin()

    def callback(self, rgb_ros_image, depth_ros_image):
        rgb = np.asarray(self.bridge.imgmsg_to_cv2(rgb_ros_image, desired_encoding="passthrough"))
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_ros_image, desired_encoding="passthrough"))
        blur = cv2.GaussianBlur(rgb, (21, 21), 5)

        # preparing the mask to overlay
        image_mask = cv2.inRange(blur, self.min_color, self.max_color)
        depth_masked = cv2.bitwise_and(depth, depth, mask=image_mask)
        points = self.get_pointcloud(depth_masked)

        x,y,z,w,h,d = 0.0,0.0,0.0,0.0,0.0,0.0
        if len(points)>0:      
            x,y,z,w,h,d = self.get_centroid(points)
            #rospy.loginfo("%s x: %.3f y: %.3f z: %.3f w: %.3f h: %.3f d: %.3f" % (self.type,x,y,z,w,h,d))
            #rospy.loginfo("%s\tx: %.3f\ty: %.3f\tz: %.3f" % (self.type,x,y,z))
            obj = Object()
            obj.header = depth_ros_image.header
            obj.point.x = x
            obj.point.y = y
            obj.point.z = z
            obj.width.data = w
            obj.height.data = h
            obj.depth.data = d
            self.object_pub.publish(obj)

        else:
            rospy.loginfo("Empty "+self.type+" pointcloud")
        
        #debugging messages for visualization
        rgb_masked = cv2.bitwise_and(rgb, rgb, mask=image_mask)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(rgb_masked, encoding="passthrough"))
        self.pc_pub.publish(pc2.create_cloud_xyz32(depth_ros_image.header, points))
        self.marker_pub.publish(get_marker(x,y,z,w,h,d,depth_ros_image.header.frame_id,self.type))

        #generate the object
        #   x,y,z position and depth,width, and height
        obj = Object()
        obj.header = depth_ros_image.header
        obj.point.x = x
        obj.point.y = y
        obj.point.z = z
        obj.w.data = w
        obj.h.data = h
        obj.d.data = d
        self.obj_pub.publish(obj)
        

    #Iterate through the masked depth image
    #   return a list of points in 3d space
    def get_pointcloud(self, depth_masked):
        point_list = []
        distances = []
        for r in range(0, depth_masked.shape[0], self.step):
            for c in range(0, depth_masked.shape[1], self.step):
                d = depth_masked[r,c]
                if self.min_depth < d <  self.max_depth:
                    distances.append((d, r, c))

        #if no valid points are found return the empty list
        if len(distances) == 0:
            return point_list

        #filter out outliers and interate through the remaining points
        for dist in reject_outliers(np.asarray(distances), m=50):
            #the depth image pixel value is the distance in millimeters
            d = dist[0]/1000.0
            r = dist[1]
            c = dist[2]

            #get the camera center (cx,cy) and focal length (fx,fy)
            cx = self.cam_model.cx()
            cy = self.cam_model.cy()
            fx = self.cam_model.fx()
            fy = self.cam_model.fy()

            #project the point into 3d space
            x = (c - cx)*d/fx
            y = (r - cy)*d/fy
            z = d

            point_list.append([x,y,z])

        return point_list

    #from a list of points return the midpoint (x,y,z) and 
    #   width, height, depth
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

        cent_x = (max_x+min_x)/2.0
        cent_y = (max_y+min_y)/2.0 
        cent_z = (max_z+min_z)/2.0

        w = abs(max_x - min_x)
        h = abs(max_y - min_y)
        d = abs(max_z - min_z)

        return [cent_x,cent_y,cent_z, w, h, d]

if __name__ == '__main__':
    get_target = FindPutty()
