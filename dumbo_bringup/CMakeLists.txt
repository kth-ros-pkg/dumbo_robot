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
