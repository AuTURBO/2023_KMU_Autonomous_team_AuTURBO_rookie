cmake_minimum_required(VERSION 3.0.2)
project(ar_track_alvar_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

set(MSG_FILES AlvarMarker.msg AlvarMarkers.msg)
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
