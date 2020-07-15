# 1. Repository description
`MiR200_calibration`: Codes are aimed to calibrate systematic odometry errors for MiR200. Most codes are inherited from matchRos under following link (https://github.com/matchRos/MiR200_Sim). Note that codes, who are related to kinematic calibration, are changed and added by myself. Other codes stay the same as matchRos. readme_1 is the readme file directly copied from matchRos who shows the original description of packages. The following description focuses only on the codes that i changed or added for calibration.

# 2. Change overview
Some codes are partly modified in terms of the original matchRos codes. Some codes are newly written and added. In this section, i will list those codes, so that you can easily coordinate the codes related to kinematic calibration. For other codes, please visit their original source with the link that i mentioned above or ROS official website.

### Codes list (modified on the original matchRos codes)
```
'diffdrive_controller.yaml'    in folder    'mir_description/config'
'mir.urdf.xacro'               in folder    'mir_description/urdf'
'mir_empty_world.launch'       in folder    'mir_gazebo/launch/includes'
'nav_start.launch'             in folder    'mir_navigation/launch'
'start_maps.launch'            in folder    'mir_navigation/launch'
'start_planner.launch'         in folder    'mir_navigation/launch'
```

### Codes list (newly added in contrast to the original matchRod codes)
```
The whole package 'diff_drive_controller_calibration'
'mir_200_v1_calibration.urdf.xacro'    in folder    'mir_description/urdf/include'
'empty_map_calibration.yaml'           in folder    'mir_gazebo/maps/world'
'empty_map_calibration.pgm'            in folder    'mir_gazebo/maps/world'
The whole folder 'calibration'         in folder    'mir_navigation/nodes'
'navigation_calibration.rviz'          in folder    'mir_navigation/rviz'
```

# 3. How to realize the calibration with codes
In this section, i will introduce how to use those codes to calibrate the sysmatic odometry errors and correct the robotic kinematics. And i will also introduce the purposes of the codes in the section 2.

### 1. Modelling of the mobile robot
Before calibration or other operations, the robot modeling must be firstly accomplished in order to simulate in gazebo. The file 'mir_200_v1_calibration.urdf.xacro' builds the robot model with the basic kinematic and physical descriptions. Note that the robot is built with both wheel diameter and wheelbase errors as the precondition for calibration. Both errors as parameters can be changed in the code at will. The file 'mir.urdf.xacro' is used to upload the robot model.

### 2. Modelling of the mobile robot






