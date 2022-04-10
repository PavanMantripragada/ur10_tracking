cmake_minimum_required(VERSION 3.0.2)
project(ur10_tracking)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp rospy std_msgs tf2 tf2_ros moveit_msgs moveit_ros_planning     
  moveit_ros_planning_interface moveit_core moveit_visual_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ur10_im_tracker src/ur10_im_tracker.cpp)
target_link_libraries(ur10_im_tracker ${catkin_LIBRARIES})

