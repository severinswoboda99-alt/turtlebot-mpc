###### Linear MPC for TurtleBot3 burger #####

### Credits ###
The OSQP repo (https://github.com/osqp/OSQP, https://osqp.org) is used, without any alteration, in this project, licensed under the Apache License 2.0.
Locate /src/osqp/LICENSE and /src/osqp/CITATION.cff for further information.

Additionally, the OSQP-Eigen C++ Wrapper makes the integration easier (https://github.com/gbionics/osqp-eigen).
No alterations have been made here either, it is licensed under the BSD 3-Clause License.

A C++ cubic spline interpolation library (https://github.com/ttk592/spline) is used for the path planner. 
It is GNU GENERAL PUBLIC license, no part of it was modified.

Data is visualized with rosbag2 and matplotlib.

The evaluation and path tracking with the real hardware is done through the detection of ArUco markers through an Intel RealSense D435I camera. 
To achieve this, an ArUco marker is positioned on top of the robot, and the camera is recording the robot from a top down perspective. 
This data is then converted by the package.
Therefore, it depends on the "ROS Wrapper for RealSense(TM) Cameras" (https://github.com/realsenseai/realsense-ros) to convert the camera data to a ROS2 topic,
and the ArUco marker package to detect the markers and convert it into positional data (https://github.com/namo-robotics/aruco_markers).
To work with ROS2 Jazzy, the later has been slightly altered.

Since most of the workflow is done through WSL, the camera needs to be shared, following this tutorial: https://learn.microsoft.com/en-us/windows/wsl/connect-usb

### Launch ###
$ source /opt/ros/jazzy/setup.bash && source install/setup.bash
$ ros2 launch mpc-tracker launch.py

### Record and Plot ###
$ cd /home/swo/turtlebot-mpc/src/mpc-tracker/rosbag2
$ ros2 bag record -o bag_type_path1_N20_Q10_10_5_R0c1_0c05 /path /traveled_path /loop_time /cmd_vel /v_ref /w_ref
FILENAME syntax: bag_type_pathX_Nn_Qq1_q2_q3_Pp1_p2_p3_Rr1_r2

Stop recording with CTRL+C
Update bag path in evaluation.py, then:
$ cd /home/swo/turtlebot-mpc/src/evaluation 
$ python3 evaluation.py
Or, respectively
$ python3 summary_plots.py

Start camera, detect ArUco markers from video data, and convert them to a path:
$ ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p rgb_camera.color_profile:=1280x720x30
$ ros2 run aruco_markers aruco_markers --ros-args \
  -p marker_size:=0.18 \
  -p camera_frame:=camera_rgb_optical_frame \
  -p image_topic:=/camera/camera/color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info \
  -p dictionary:=DICT_4X4_50
$ cd /home/swo/turtlebot-mpc/src/mpc-tracker/rosbag2
$ ros2 bag record -o real_pathX_bagX /aruco/markers

$ cd /home/swo/turtlebot-mpc/src/evaluation 
$ python3 evaluation.py