#! /usr/bin/env python3
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
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



class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('pose_reach', anonymous=True)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    
    def go_to_pose(self, arg_pose):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        
        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
        return flag_plan
    

    # Destructor 
    def __del__(self):
        moveit_commander.roscpp_shutdown() 


class gripperMoveit:

    # Constructor
    def __init__(self):

        self._planning_group = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        #pose_values = self._group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        #rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan


def main():

    rospy.Subscriber("/camera/depth/points",PointCloud2,depthCallBack)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    ur5 = Ur5Moveit()
    gripper = gripperMoveit()
    output = get_result()
    print("Output achieved", output)
    
    gripper_angle_close = [0.018, 0.018]
    gripper_angle_open = [0,0]
 
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = output[0][0]
    ur5_pose_1.position.y = output[0][1]
    ur5_pose_1.position.z = output[0][2] - 0.5
    ur5_pose_1.orientation.x = 0.0007679995666944821
    ur5_pose_1.orientation.y = 0.7076495922360097
    ur5_pose_1.orientation.z = 0.0007605329559611779
    ur5_pose_1.orientation.w = 0.7065627264259683

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 1.0248207245818008
    ur5_pose_3.position.y = -4.466576181778505e-05
    ur5_pose_3.position.z = 1.0847458749382433
    ur5_pose_3.orientation.x = 0.0007710397076692243
    ur5_pose_3.orientation.y = 0.6973618900455334
    ur5_pose_3.orientation.z = 0.000761239599975542
    ur5_pose_3.orientation.w = 0.7167183689038268

    
    ur5.go_to_pose(ur5_pose_1)
    rospy.sleep(1)
    gripper.set_joint_angles(gripper_angle_open)
    rospy.sleep(1)
    gripper.set_joint_angles(gripper_angle_close)
    rospy.sleep(2)    
    ur5.go_to_pose(ur5_pose_3)
    rospy.sleep(2)
    rospy.spin()
    cv2.destroyAllWindows()  



if __name__ == '__main__':
    main()