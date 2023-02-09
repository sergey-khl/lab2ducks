cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  LEDService.srv
)

catkin_package()