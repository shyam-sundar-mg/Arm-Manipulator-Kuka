# Robotic Arm Manipulation with Kuka KR10 arm

**Pick and Place task using Kuka arm**

Arm name - Kuka KR10 R1420
Source repository link - https://github.com/ros-industrial/kuka_experimental

Modifications done:
 * Added inertia element to each link. (Calculated using Meshlab)
 * Attached the intelrealsense camera plug-in through the camera link.
 * Attached a gripper link.

Progress:
 - [x] Moveit Setup for arm
 - [x] Forward Kinematics using OMPL planner
 - [x] Inverse Kinematics using OMPL planner
 
 Link for video:
 https://drive.google.com/file/d/1P5Kcih5bqxdbGG423fsxMVZsDV7M660N/view?usp=sharing
 
 Issues:
 * Gripper attachment is imprecise
 * Gripper friction / object friction is not sufficient for picking task.

Current work:
 * Implementing without planners
