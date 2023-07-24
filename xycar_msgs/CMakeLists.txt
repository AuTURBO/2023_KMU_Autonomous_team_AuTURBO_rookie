cmake_minimum_required(VERSION 3.0.2)
project(xycar_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    sensor_msgs
    std_msgs
)

add_message_files(FILES
    xycar_motor.msg
    xycar_ultrasounds.msg
)

generate_messages(DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    std_msgs
)
