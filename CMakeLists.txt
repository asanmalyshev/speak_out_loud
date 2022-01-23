cmake_minimum_required(VERSION 3.0.2)
project(speak_out_loud)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Priority.msg
)

add_action_files(
  DIRECTORY action
  FILES Speak.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs  
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS actionlib actionlib_msgs roscpp
)

###########
## Build ##
###########

include_directories(
# include
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_srv src/speak_out_loud_action_srv.cpp)

# add_dependencies(${PROJECT_NAME}_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_srv
#   ${catkin_LIBRARIES}
#   speechd
# )

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/sol_server.py scripts/speak_out_loud_client_example_node.py examples/turtlesim/speak_out_loud_turtle_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

