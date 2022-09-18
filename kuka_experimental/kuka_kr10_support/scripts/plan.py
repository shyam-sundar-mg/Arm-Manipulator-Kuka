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


class Moveit:

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

        
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        if (flag_plan == True):
            print('Success')
        else:
            print('Failed')
        return flag_plan
    def printer(self):
        list_joint_values = self._group.get_current_joint_values()
        print(list_joint_values)


    # Destructor 
    def __del__(self):
        moveit_commander.roscpp_shutdown() 



def main():
    kuka = Moveit()

    box_1 = [math.radians(0),
             math.radians(-62),
             math.radians(113),
             math.radians(0),
             math.radians(33),
             math.radians(0),0,0]
    box_1_grip = [math.radians(0),
             math.radians(-62),
             math.radians(113),
             math.radians(0),
             math.radians(33),
             math.radians(0),0.015,0.015]    
    drop_pose_11 = [math.radians(0),
             math.radians(-45),
             math.radians(45),
             math.radians(0),
             math.radians(45),
             math.radians(0),0.015,0.015]
    drop_pose_12 = [math.radians(90),
             math.radians(-45),
             math.radians(90),
             math.radians(0),
             math.radians(45),
             math.radians(0),0.015,0.015]
    drop = [math.radians(90),
             math.radians(-45),
             math.radians(90),
             math.radians(0),
             math.radians(45),
             math.radians(0),0,0]
    box_2 = [math.radians(30),
             math.radians(-62),
             math.radians(113),
             math.radians(0),
             math.radians(33),
             math.radians(30),0,0]
    box_2_grip = [math.radians(30),
             math.radians(-62),
             math.radians(113),
             math.radians(0),
             math.radians(33),
             math.radians(30),0.015,0.015]
    drop_pose_21 = [math.radians(30),
             math.radians(-45),
             math.radians(45),
             math.radians(0),
             math.radians(45),
             math.radians(30),0.015,0.015]
    drop_pose_22 = [math.radians(90),
             math.radians(-45),
             math.radians(90),
             math.radians(0),
             math.radians(45),
             math.radians(0),0.015,0.015]
    box_3 = [math.radians(-30),
             math.radians(-62),
             math.radians(113),
             math.radians(0),
             math.radians(33),
             math.radians(-30),0,0]
    box_3_grip = [math.radians(-30),
             math.radians(-62),
             math.radians(113),
             math.radians(0),
             math.radians(33),
             math.radians(-30),0.015,0.015]
    drop_pose_31 = [math.radians(-30),
             math.radians(-45),
             math.radians(45),
             math.radians(0),
             math.radians(45),
             math.radians(-30),0.015,0.015]
    drop_pose_32 = [math.radians(90),
             math.radians(-45),
             math.radians(90),
             math.radians(0),
             math.radians(45),
             math.radians(0),0.015,0.015]

    pose_names = [box_1,box_1_grip, drop_pose_11, drop_pose_12,drop, drop_pose_21, drop_pose_12]


    while not rospy.is_shutdown():
        kuka.set_joint_angles(box_1)
        kuka.printer()
        kuka.set_joint_angles(box_1_grip)

        kuka.set_joint_angles(drop_pose_11)
        kuka.set_joint_angles(drop_pose_12)
        kuka.set_joint_angles(drop)

        kuka.set_joint_angles(drop_pose_21)
        kuka.set_joint_angles(box_2)
        kuka.set_joint_angles(box_2_grip)
        kuka.set_joint_angles(drop_pose_21)
        kuka.set_joint_angles(drop_pose_22)
        kuka.set_joint_angles(drop)

        kuka.set_joint_angles(drop_pose_31)
        kuka.set_joint_angles(box_3)
        kuka.set_joint_angles(box_3_grip)
        kuka.set_joint_angles(drop_pose_31)
        kuka.set_joint_angles(drop_pose_32)
        kuka.set_joint_angles(drop)
        break

    del kuka

if __name__ == '__main__':
    main()