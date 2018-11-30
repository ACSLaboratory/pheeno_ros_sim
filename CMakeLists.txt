cmake_minimum_required(VERSION 2.8.3)
project(pheeno_ros_sim)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  pheeno_ros
  pheeno_ros_description
  gazebo_ros
  gazebo_ros_control
  gazebo_msgs
  gazebo_plugins
)

# Boost and Gazebo
FIND_PACKAGE(Boost REQUIRED COMPONENTS system)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

INCLUDE(FindPkgConfig)
IF(PKG_CONFIG_FOUND)
  PKG_CHECK_MODULES(GAZEBO gazebo)
ENDIF()

FIND_PACKAGE(gazebo REQUIRED)
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
LINK_DIRECTORIES(${GAZEBO_LIBRARY_DIRS})

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   sensor_msgs
# )

## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pheeno_ros_sim
  CATKIN_DEPENDS roscpp rospy tf std_msgs nav_msgs sensor_msgs geometry_msgs pheeno_ros pheeno_ros_description gazebo_ros gazebo_ros_control gazebo_msgs gazebo_plugins
  DEPENDS gazebo
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
ADD_LIBRARY(gazebo_ros_ir_sensor SHARED src/gazebo_ros_ir_sensor.cpp)
TARGET_LINK_LIBRARIES(gazebo_ros_ir_sensor ${catkin_LIBRARIES} ${boost_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS gazebo_ros_ir_sensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch models worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pheeno_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
