# UUV simulator

Developed based on DAVE

## Prerequisites

 - Python (Python 3.8.10 recommended)
 - ROS (ROS noetic recommended)
 - DAVE: [@DAVE](https://field-robotics-lab.github.io/dave.doc/)
 - BlueROV2 MPC [@HKPolyU-UAV](https://github.com/HKPolyU-UAV/bluerov2)

## Getting started
1.	`source uuv_ws/devel/setup.bash`
2.	for wrecked ship: `roslaunch bluerov2_dobmpc start_dobmpc_demo_ship.launch`
3.	for swimming pool (AprilTag placed underwater): `roslaunch bluerov2_dobmpc start_dobmpc_demo_pool.launch`
4.	for swimming pool (AprilTag placed above water): `roslaunch bluerov2_dobmpc start_dobmpc_demo_pool_side.launch`


