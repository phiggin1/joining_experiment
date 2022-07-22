#!/usr/bin/env python2

import numpy as np
from copy import deepcopy
import math
import rospy
from geometry_msgs.msg import PoseStamped

#from filterpy.kalman import KalmanFilter
#from filterpy.common import Q_discrete_white_noise

class TargetFilter:
    def __init__(self):
        rospy.init_node('move_to', anonymous=True)
        self.prev = None

        self.count = 0.0
        self.incorrect = 0.0

        self.sigma = 0.005
        self.dt = 1.0

        sig =self.sigma
        dt =self.dt


        self.x = np.zeros((6,1))

        self.u = np.zeros((3,1))

        self.A = np.matrix([[1, 0, 0, dt, 0,  0 ],
                            [0, 1, 0, 0,  dt, 0 ],
                            [0, 0, 1, 0,  0,  dt],
                            [0, 0, 0, 1,  0,  0 ],
                            [0, 0, 0, 0,  1,  0 ],
                            [0, 0, 0, 0,  0,  1 ]])

        self.B = np.matrix([[0.5*dt**2, 0,         0        ],
                            [0,         0.5*dt**2, 0,       ],
                            [0,         0,         0.5*dt**2],
                            [dt,        0,         0        ],
                            [0,         dt,        0        ],
                            [0,         0,         dt       ]])   

        self.H = np.eye(6,6)

        self.P = np.eye(6,6)

        self.Q = np.matrix([[(dt**4)/4, 0,         0,         (dt**3)/2, 0,         0        ],
                               [0,         (dt**4)/4, 0,         0,         (dt**3)/2, 0        ],
                               [0,         0,         (dt**4)/4, 0,         0,         (dt**3)/2],
                               [(dt**3)/2, 0,         0,         dt**2,     0,         0        ],
                               [0,         (dt**3)/2, 0,         0,         dt**2,     0        ],
                               [0,         0,         (dt**3)/2, 0,         0,         dt**2    ]])*sig**2

        self.R = np.eye(6,6)*sig

        self.target_filtered_pub = rospy.Publisher('/target/target_filtered', PoseStamped, queue_size=10)
        self.target_sub = rospy.Subscriber("/target/target", PoseStamped, self.callback)
        rospy.spin()

    def callback(self, target):
        self.count += 1.0
        vx = 0.0
        vy = 0.0
        vz = 0.0
        if not self.prev is None:
            t = target.header.stamp.to_sec() - self.prev.header.stamp.to_sec()
            vx = (target.pose.position.x-self.prev.pose.position.x)/t
            vy = (target.pose.position.y-self.prev.pose.position.y)/t
            vz = (target.pose.position.z-self.prev.pose.position.z)/t
        
        z = np.asarray([
            [target.pose.position.x],
            [target.pose.position.y],
            [target.pose.position.z],
            [vx],
            [vy],
            [vz]
        ])

        self.predict()
        self.update(z)
        print('==============================')
        print(target.header.stamp.to_sec())

        filtered = deepcopy(target)
        filtered.pose.position.x = self.x.getA()[0][0]
        filtered.pose.position.y = self.x.getA()[1][0]
        filtered.pose.position.z = self.x.getA()[2][0]

        
        print('target')
        print(target.pose.position)
        print('filtered')
        print(filtered.pose.position)
        

        if not self.prev is None:
            print('target diff prev')
            d_targ_prev = math.sqrt(
(target.pose.position.x - self.prev.pose.position.x)**2+
(target.pose.position.y - self.prev.pose.position.y)**2+
(target.pose.position.z - self.prev.pose.position.z)**2
            )
            print(d_targ_prev)


            print('filtered target diff prev')
            d_filt_prev = math.sqrt(
(filtered.pose.position.x - self.prev.pose.position.x)**2+
(filtered.pose.position.y - self.prev.pose.position.y)**2+
(filtered.pose.position.z - self.prev.pose.position.z)**2
            )
            print(d_filt_prev)        
            if d_targ_prev < d_filt_prev:
                print("not working")
                self.incorrect += 1.0

        print('--------------------')
        print((self.count-self.incorrect)/self.count)

        self.target_filtered_pub.publish(filtered)
        self.prev = deepcopy(target)

    def update(self, z):
        # Refer to :Eq.(11), Eq.(12) and Eq.(13)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #Eq.(11)

        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))   #Eq.(12)

        I = np.eye(self.H.shape[1])

        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P   #Eq.(13)
        return self.x[0:2]

    def predict(self):
        # Refer to :Eq.(9) and Eq.(10)  in https://machinelearningspace.com/object-tracking-simple-implementation-of-kalman-filter-in-python/?preview_id=1364&preview_nonce=52f6f1262e&preview=true&_thumbnail_id=1795

        # Update time state
        #x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)

        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]

if __name__ == '__main__':
    fil = TargetFilter()
