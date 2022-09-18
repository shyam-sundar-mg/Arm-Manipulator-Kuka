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

current_frame=0
def callback(data):
    global current_frame
    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

def extractPoints(data):
    depth_points = [] # List to store the 3D points
    for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=False): #Function to get the individual points
        depth_points.append((point[0],point[1],point[2]))

    return depth_points

def img2imgpoints(image_name='camera_output_raw.png'):
    frame=current_frame
    frame1=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100,50,50])
    upper_blue = np.array([130,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    output = []
    for cnt in contours:
        x_rect,y_rect,w_rect,h_rect = cv2.boundingRect(cnt)
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])    
        center_blue=[cx,cy]
        edge_1 = [x_rect+2, y_rect+2]
        edge_2 = [x_rect+w_rect-2, y_rect+2]
        output.append([center_blue,edge_1,edge_2])
    return output


init = 0
X1, Y1, Z1 = [0,0,0]
dummy = []
def depthCallBack(cloud):
    global dummy
    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step
    row_step = cloud.row_step
    depth_points=extractPoints(cloud)
    output = list(list(img2imgpoints('camera_output_raw.png')))

    result = []
    for coords in output:
        u,v=coords[0]
        #print(u,v)
        edge_1_px, edge_1_py = coords[1]
        edge_2_px, edge_2_py = coords[2]
        pos=u+v*width

        pos_edge_1 = edge_1_px + edge_1_py * width
        pos_edge_2 = edge_2_px + edge_2_py * width

        cam_edge_x1, cam_edge_y1, cam_edge_z1 = depth_points[pos_edge_1]
        cam_edge_x2, cam_edge_y2, cam_edge_z2 = depth_points[pos_edge_2]

        edge_1_wrt_source = Point(cam_edge_z1, -cam_edge_x1,-cam_edge_y1)
        edge_2_wrt_source = Point(cam_edge_z2, -cam_edge_x2,-cam_edge_y2)


        #print('pos:',pos)
        x,y,z=depth_points[pos]
        #print('Points wrt source:',depth_points[pos])

        source_frame= 'camera_link'
        target_frame='base_link'

        point_wrt_source = Point(z, -x, -y)
        transformation = get_transformation_perception(source_frame, target_frame)
        point_wrt_target = transform_point_perception(transformation, point_wrt_source)

        edge_1_wrt_source = Point(cam_edge_z1, -cam_edge_x1,-cam_edge_y1)
        edge_2_wrt_source = Point(cam_edge_z2, -cam_edge_x2,-cam_edge_y2)


        edge_1_wrt_target = transform_point_perception(transformation, edge_1_wrt_source)
        edge_2_wrt_target = transform_point_perception(transformation, edge_2_wrt_source)

        X1,Y1,Z1 = point_wrt_target

        edge_1_x, edge_1_y, edge_1_z = edge_1_wrt_target
        edge_2_x, edge_2_y, edge_2_z = edge_2_wrt_target
        print('Points:', point_wrt_target)
        #print('\n')
        width_obs = math.sqrt((edge_1_x-edge_2_x)**2+(edge_1_y-edge_2_y)**2)
        print("Width",width_obs)

        print("Edge_1:",edge_1_wrt_target, "\nEdge_2:", edge_2_wrt_target)
        
        result.append([X1,Y1,Z1,width_obs])
    
    dummy = result

def get_result():
    return dummy
    
def transform_point_perception(transformation, point_wrt_source):
    point_wrt_target = \
        tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),
            transformation).point
    return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

def get_transformation_perception(source_frame='camera_link', target_frame='base_link',tf_cache_duration=2.0):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
    tf2_ros.TransformListener(tf_buffer)
    global transformation

    # get the tf at first available time
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(1))
        #print(transformation)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation')
    return transformation

#main_code
if __name__ == '__main__':
    print('Hi')
    rospy.init_node('obj_detection', anonymous=True)
    rospy.Subscriber("/camera/depth/points",PointCloud2,depthCallBack)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.spin()
    cv2.destroyAllWindows()
