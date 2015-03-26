cmake_minimum_required(VERSION 2.8.3)
project(dumbo_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS dumbo_description
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/dumbo_robotiq_bringup.launch
  ${CMAKE_CURRENT_SOURCE_DIR}/components/start_pci_CAN_cards.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/components/stop_pci_CAN_cards.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/components/restart_pci_CAN_cards.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)