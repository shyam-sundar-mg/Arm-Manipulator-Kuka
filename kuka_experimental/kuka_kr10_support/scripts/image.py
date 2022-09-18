#!/usr/bin/env python3
from cv2 import sort
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import rostopic
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import tf
from tf.transformations import euler_from_quaternion
from math import degrees
import matplotlib.pyplot as plt
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import os
from sensor_msgs.msg import PointCloud2
import struct
import sensor_msgs.point_cloud2
import math

def callback(data):
    global current_frame
    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(data)
    ## current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
    ## frame=current_frame
    ## frame1=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    plt.figure()
    plt.imshow(current_frame) 
    plt.show()
    
    
if __name__ == '__main__':
    print('Hi')
    rospy.init_node('get_blue_object', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.spin()
    cv2.destroyAllWindows()
