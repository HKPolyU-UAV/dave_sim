# Underwater Simulation
Developed based on DAVE

## Prerequisites

 - Python (Python 3.8.10 recommended)
 - ROS (ROS noetic recommended)
 - DAVE: [@DAVE](https://field-robotics-lab.github.io/dave.doc/)
 - BlueROV2 MPC [@HKPolyU-UAV](https://github.com/HKPolyU-UAV/bluerov2)

## Getting Started
1.	`source uuv_ws/devel/setup.bash`
2.	for wrecked ship: `roslaunch bluerov2_dobmpc start_dobmpc_demo_ship.launch`
3.	for swimming pool (AprilTag placed underwater): `roslaunch bluerov2_dobmpc start_dobmpc_demo_pool.launch`
4.	for swimming pool (AprilTag placed above water): `roslaunch bluerov2_dobmpc start_dobmpc_demo_pool_side.launch`

# Semi-Physical Dataset
The Semi-physical dataset collected in a swimming pool (Length 50 m x Width 25 m, Depth 1 m) can be avaible by https://drive.google.com/file/d/1Aa0WR3RiYMOI9ouY_rHTWtJH9byqR_ph/view?usp=drive_link.

## rosbag info
/bluerov2/camera_front/camera_image: Images collected by the monocular camera outfitted with the BlueROV2 robot. 
/bluerov2/dvl: Velocity collected by the DVL. 
/bluerov2/imu: Linear acceleration and angular velocity of the BlueROV2 robot. 
/bluerov2/sonar: Include the range between the sonar and the obstacle. 
/bluerov2/pressure: Water pressure collected by the pressure sensor which provides the measurement of water-depth.
