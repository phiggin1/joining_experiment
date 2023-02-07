#!/usr/bin/env python3

import rospy
import message_filters
import tf
import numpy as np
import math
from joining_experiment.msg import Object
from joining_experiment.msg import JoinPose
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def dot(a,b):
    return (a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2])

def dist(pa, pb):
    return math.sqrt((pa[0] - pb[0])**2+(pa[1] - pb[1])**2+(pa[2] - pb[2])**2)

def norm(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

class GetTargetPose:
    def __init__(self):
        rospy.init_node('GetTargetPose', anonymous=True)

        #width of the led and putty in m
        self.led_width = rospy.get_param("led_width", 15.0/1000.0)
        self.putty_width = rospy.get_param("putty_width", 50.0/1000.0)

        self.min_dist = rospy.get_param("min_dist", 20.0/1000.0)
        self.max_dist = rospy.get_param("max_dist", 75.0/1000.0)

        self.min_x = rospy.get_param("min_x", 0.0)
        self.max_x = rospy.get_param("max_x", 0.65)
        self.min_y = rospy.get_param("min_y", -0.5)
        self.max_y = rospy.get_param("max_y", 0.5)
        self.min_z = rospy.get_param("min_z", 0.0)
        self.max_z = rospy.get_param("max_z", 0.3)


        self.base_frame = 'base_link'

        self.listener = tf.TransformListener()

        self.anode = None
        self.cathode = None
        self.last_valid_anode = rospy.Time.now()
        self.last_valid_cathode = rospy.Time.now()
        self.wait = 0.50

        self.min_dist =  0.00750
        self.max_dist =  0.075

        self.debug = rospy.get_param("debug", True) 

        self.target_pub = rospy.Publisher('/target/target', JoinPose, queue_size=10)
        self.pose_stamped_pub = rospy.Publisher('/target/pose_stamped', PoseStamped, queue_size=10)

        self.cathode_sub = rospy.Subscriber('/target/object_cathode', Object, self.get_cathode)
        self.anode_sub = rospy.Subscriber('/target/object_anode', Object, self.get_anode)
        '''
        self.cathode_sub = message_filters.Subscriber('/target/object_cathode', Object)
        self.anode_sub = message_filters.Subscriber('/target/object_anode', Object)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.cathode_sub, self.anode_sub], 10, slop=2.0)
        self.ts.registerCallback(self.callback)
        '''
        rospy.sleep(5.0)
        rate =rospy.Rate(60)
        while not rospy.is_shutdown():
            #if self.anode is not None and self.cathode is not None:
            self.callback(self.cathode, self.anode)
            rate.sleep()
            

    def get_cathode(self, cathode):
        self.cathode = cathode
        self.last_valid_cathode = rospy.Time.now()

    def get_anode(self, anode):
        self.anode = anode
        self.last_valid_anode = rospy.Time.now()

    def callback(self, cathode, anode):       
        target = JoinPose()
        target.pose.position.x = 0.0
        target.pose.position.y = 0.0
        target.pose.position.z = 0.0
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0

        #error checking
        target.too_far = False     #cathode and anode close enough
        target.too_close = False   #cathode and anode far enough apart to not be touching
        target.see_cathode = True  #can see cathode
        target.see_anode = True    #can see anode
        target.in_workspace = True #is cathode&anode in workspace
        target.move_direction = ""      #   if not move right/left/up/down/closer/farther

        see = True
        now = rospy.Time.now()
        if now > (self.last_valid_anode + rospy.Duration(self.wait)) or self.anode is None:
            target.see_anode = False
            target.in_workspace = False 
            see = False
            rospy.loginfo("see_anode = False")
        if now > (self.last_valid_cathode + rospy.Duration(self.wait)) or self.cathode is None:
            target.see_cathode = False
            target.in_workspace = False 
            see = False
            rospy.loginfo("see_cathode = False")

        if not see:
            self.target_pub.publish(target)
            
            #rospy.loginfo("see_anode:%r see_cathode:%r" % (target.see_anode, target.see_cathode))
            
            return

        #vector from anode to cathode
        a = [anode.point.x - cathode.point.x,
             anode.point.y - cathode.point.y,
             anode.point.z - cathode.point.z]

        #forward vector in base_link space 
        b = (1, 0, 0)

        pa = [  anode.point.x,   anode.point.y,   (anode.point.z+0.5*anode.h.data)]
        pc = [cathode.point.x, cathode.point.y, (cathode.point.z+0.5*anode.h.data)]
        
        #pa = [  anode.point.x,   anode.point.y,   anode.point.z]
        #pc = [cathode.point.x, cathode.point.y, cathode.point.z]

        #check if to cathode or anode are too close/far
        d = dist(pa, pc)

        if d <= self.min_dist:
            target.too_close = True
        elif d > self.max_dist:
            target.too_far = True

        #yaw (in kinect frame around y axis)
        theta = np.arccos( dot(a,b)/(norm(a)*norm(b)) )

        #generate quaternion from the yaw
        #quat = quaternion_from_euler(math.pi, 0.0, theta)     
        quat = quaternion_from_euler(math.pi, 0.0, math.pi/2.0)          

        #the target position is between the anode and cathode
        target.header = cathode.header
        target.pose.position.x = (pa[0]+pc[0])/2.0
        target.pose.position.y = (pa[1]+pc[1])/2.0 
        target.pose.position.z = (pa[2]+pc[2])/2.0
        target.pose.orientation.x = quat[0]
        target.pose.orientation.y = quat[1]
        target.pose.orientation.z = quat[2]
        target.pose.orientation.w = quat[3]

        move_direction = []
        #check if in workspace
        if target.pose.position.x < self.min_x:
            target.in_workspace = False
            move_direction.append("farther from my base")
        elif target.pose.position.x > self.max_x:
            target.in_workspace = False
            move_direction.append("closer to my base")
        if target.pose.position.y < self.min_y:
            target.in_workspace = False
            move_direction.append("to my right")
        elif target.pose.position.y > self.max_y:
            target.in_workspace = False
            move_direction.append("to my left")

        if target.pose.position.z < self.min_z:
            move_direction.append("up")
            target.in_workspace = False
        elif target.pose.position.z > self.max_z:            
            target.in_workspace = False
            move_direction.append("down")

        target.move_direction = move_direction

        stamped_pose = PoseStamped()
        stamped_pose.header = cathode.header
        stamped_pose.pose.position.x = target.pose.position.x
        stamped_pose.pose.position.y = target.pose.position.y
        stamped_pose.pose.position.z = target.pose.position.z
        stamped_pose.pose.orientation.x = target.pose.orientation.x
        stamped_pose.pose.orientation.y = target.pose.orientation.y
        stamped_pose.pose.orientation.z = target.pose.orientation.z
        stamped_pose.pose.orientation.w = target.pose.orientation.w

        self.pose_stamped_pub.publish(stamped_pose)

        
        '''rospy.loginfo("positon:     x:%.4f\ty:%.4f\tz:%.4f" % (target.pose.position.x, target.pose.position.y, target.pose.position.z) )
        rospy.loginfo("orientation: x:%.4f\ty:%.4f\tz:%.4f\tw:%.4f" % (target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w))
        rospy.loginfo("dist:%.4f min_dist:%.4f max_dist:%.4f" %(d,self.min_dist,self.max_dist))
        rospy.loginfo("too_close:%r too_far:%r" % (target.too_close, target.too_far))
        rospy.loginfo(move_direction)'''
        
        
        self.target_pub.publish(target)

if __name__ == '__main__':
    get_targets = GetTargetPose()
