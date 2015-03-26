cmake_minimum_required(VERSION 2.8.3)
project(dumbo_controller_config)


find_package(catkin REQUIRED COMPONENTS
)


catkin_package()


install(FILES
  config/dumbo_joint_state_controller.yaml
  config/dumbo_force_torque_sensor_controller.yaml
  config/dumbo_joint_trajectory_controllers.yaml
  config/dumbo_joint_velocity_controllers.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
