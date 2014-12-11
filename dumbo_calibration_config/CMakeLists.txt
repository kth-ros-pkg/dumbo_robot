cmake_minimum_required(VERSION 2.8.3)
project(dumbo_calibration_config)


find_package(catkin REQUIRED COMPONENTS
)


catkin_package()


install(FILES
  config/dumbo_left_arm_ft_sensor.yaml
  config/dumbo_right_arm_ft_sensor.yaml
  config/depth_PS1080_PrimeSense.yaml
  config/rgb_PS1080_PrimeSense.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
