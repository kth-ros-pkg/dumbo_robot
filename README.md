dumbo_robot
===========

Overview
---------------------------------------------
Bringup files and hardware configuration files for CVAP's Dumbo robot


dumbo_bringup
---------------------------------------------

Contains the main launch files necesary to start Dumbo's core ROS components.


dumbo_sim_bringup (deprecated)
---------------------------------------------

Package for bringing up a simulated Dumbo in rviz. 
Can simulate the kinematics of the robot through a velocity integrator. 


dumbo_hardware_config
---------------------------------------------

Contains yaml parameter files for configuring several hardware components.


dumbo_calibration_config
---------------------------------------------

Contains calibration files of Dumbo's force-torque sensors and cameras.


dumbo_controller_config
---------------------------------------------

Contains controller gains and launch files for Dumbo's controllers. These controllers run using the ros_control specification.