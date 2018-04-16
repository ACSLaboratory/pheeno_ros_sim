cmake_minimum_required(VERSION 2.8.3)
project(pheeno_ros_sim)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  urdf
  xacro
  rviz
  tf
  image_transport
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
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

INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
LINK_DIRECTORIES(${GAZEBO_LIBRARY_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pheeno_IR_Sensor.msg
  Pheeno_Cam_Analysis.msg
)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pheeno_ros_sim
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(pheeno_ros
#   src/${PROJECT_NAME}/pheeno_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pheeno_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(pheeno_ros_node src/pheeno_ros_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(pheeno_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(pheeno_ros_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

ADD_LIBRARY(gazebo_ros_ir_sensor SHARED src/gazebo_ros_ir_sensor.cpp)
TARGET_LINK_LIBRARIES(gazebo_ros_ir_sensor ${catkin_LIBRARIES} ${boost_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin)

ADD_EXECUTABLE(obstacle_avoidance_sim src/command_line_parser.cpp src/pheeno_robot.cpp src/obstacle_avoidance.cpp)
TARGET_LINK_LIBRARIES(obstacle_avoidance_sim ${catkin_LIBRARIES})

ADD_EXECUTABLE(random_walk_sim src/command_line_parser.cpp src/pheeno_robot.cpp src/random_walk.cpp)
TARGET_LINK_LIBRARIES(random_walk_sim ${catkin_LIBRARIES})

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS gazebo_ros_ir_sensor obstacle_avoidance_sim random_walk_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

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
