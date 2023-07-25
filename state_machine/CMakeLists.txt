cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
  geometry_msgs
  nav_msgs
  actionlib  
  actionlib_msgs 
  move_base_msgs
  )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  state_machine_request.msg
  state_machine_reply.msg
  pose.msg
  point.msg
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
  std_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs

)


catkin_package(
  CATKIN_DEPENDS
  roscpp 
  rospy 
  std_msgs 
  message_runtime
  actionlib
  actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(state_machine_request_node
  src/state_machine_request_node.cpp
)
target_link_libraries(state_machine_request_node
    jsoncpp
    ${catkin_LIBRARIES}

)
add_dependencies(state_machine_request_node 
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(state_machine_reply_node
  src/state_machine_reply_node.cpp
)
target_link_libraries(state_machine_reply_node
    jsoncpp
    ${catkin_LIBRARIES}
)
add_dependencies(state_machine_reply_node 
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)


link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)
FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)

add_executable(state_machine_request src/state_machine_request.cpp)
target_link_libraries(state_machine_request
    ${catkin_LIBRARIES}
)
add_dependencies(state_machine_request 
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( state_machine_request 
  ${YAML_CPP_LIBRARIES}
)