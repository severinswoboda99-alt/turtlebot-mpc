###### Linear MPC for TurtleBot3 burger #####

### Credits ###
The OSQP repo (https://github.com/osqp/OSQP, https://osqp.org) is used, without any alteration, in this project, licensed under the Apache License 2.0.
Locate /src/osqp/LICENSE and /src/osqp/CITATION.cff for further information.

Additionally, the OSQP-Eigen C++ Wrapper makes the integration easier (https://github.com/gbionics/osqp-eigen).
No alterations have been made here either, it is licensed under the BSD 3-Clause License.

A C++ cubic spline interpolation library (https://github.com/ttk592/spline) is used for the path planner. 
It is GNU GENERAL PUBLIC license, no part of it was modified.

Data is visualized with rosbag2 and matplotlib.

### Prerequisites ###
* Install OSQP, according to their website.

### Launch ###
From sourced terminal:
$ ros2 launch mpc-tracker launch.py

### Record and Plot ###
$ cd /home/swo/turtlebot-mpc/src/mpc-tracker/rosbag2
$ ros2 bag record -o bag_type_path0_N15_Q1_1_1_1_P1_1_R1_1 /path /traveled_path /loop_time /cmd_vel /v_ref /w_ref
FILENAME syntax: bag_type_pathX_N(n)_Q(q1,q2,q3,q4)_P(p1,p2)_R(r1,r2)

Stop recording with CTRL+C
Update bag path in evaluation.py, then:
$ cd /home/swo/turtlebot-mpc/src/mpc-tracker/evaluation 
$ python3 evaluation.py