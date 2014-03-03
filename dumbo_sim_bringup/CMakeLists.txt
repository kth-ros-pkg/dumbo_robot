cmake_minimum_required(VERSION 2.8.3)
project(dumbo_sim_bringup)

find_package(catkin REQUIRED COMPONENTS roscpp cob_srvs control_msgs brics_actuator sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES test
  CATKIN_DEPENDS cob_srvs control_msgs brics_actuator sensor_msgs
  #  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(dumbo_powercube_chain_sim src/dumbo_powercube_chain_sim.cpp)
target_link_libraries(dumbo_powercube_chain_sim ${catkin_LIBRARIES})

add_dependencies(dumbo_powercube_chain_sim cob_srvs_gencpp) 

add_executable(dumbo_velocity_integrator src/dumbo_velocity_integrator.cpp)
target_link_libraries(dumbo_velocity_integrator ${catkin_LIBRARIES})

install(TARGETS dumbo_powercube_chain_sim dumbo_velocity_integrator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)