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

### 1. Modeling of the mobile robot
Before calibration or other operations, the robot modeling must be firstly accomplished in order to simulate in gazebo. The file 'mir_200_v1_calibration.urdf.xacro' builds the robot model with the basic kinematic and physical descriptions. Note that the robot is built with both wheel diameter and wheelbase errors as the precondition for calibration. Both errors as parameters can be changed in the code at will. The file 'mir.urdf.xacro' is used to upload the robot model.

### 2. Configuration of the simualtion environment and rviz
The file 'mir_empty_world.launch' changes and launches the simulation environment in gazebo as an empty world. The files 'empty_map_calibration.yaml' and 'empty_map_calibration.pgm', which are obtained from the empty world, change the map also as empty. The files 'start_maps.launch' and 'start_planner.launch' are used to load and launch the empty map.

The simualtion trajectory results will be presented in rviz. So the rviz window must be configurated to my desired version by using file 'navigation_calibration.rviz'. The file 'nav_start.launch' is used to load and launch the rviz configuration.

### 3. Calibration operations (estimation phase)
The calibration procedure is a UMBmark-based operation scheme who lets the robot move in a sqaure path with both clockwise and counterclockwise directions. Firstly, the robot must be initialized by the file 'mir_start.launch' in folder 'mir_navigation/launch'. After initialization, the robot will run under the commands from the python scripts in folder 'mir_navigation/nodes/calibration'. The python scripts in folder 'mir_navigation/nodes/calibration' have two versions respectively for simulations and experiments. After the bi-directional square path test, the intial and final robot poses will be output in two .txt files. Then the date processing codes (also python scripts) in folder 'mir_navigation/nodes/calibration' are used to calculate the estimated wheel diameter and wheelbase errors.

### 4. Calibration operations (compensation phase)
When both wheel diameter and wheelbase errors are estimated from the bi-directional square path test, both errors should be compensated in the wheel motor controller codes by replacing the nominal values with their acutal values, so that the computed odometry is close to the actual displacement of the robot. The whole package 'diff_drive_controller_calibration' is the ROS controller package to differentially control two wheel motors. I changed the controller codes that right and left wheel diameters can be set seperately and odometry can be computed according to them. The original controller codes require that both diameters must be identical. The file 'diffdrive_controller.yaml' is the configuration file for the diff_drive_controller. All you have to do is to copy the results from data processing in this configuration file, the compensation will be carried out automatically in controller codes.

### 5. Evaluation phase
After both erros are compensated, i recommend to run the bi-directional square path test again ,which is the same as the third part, in order to evaluate the calibration performance. The trajectory presented in rviz will give you a visual evaluation of calibration performance. The pose results from data processing will give you a numerical evaluation of calibration performance.
