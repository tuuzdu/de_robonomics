cmake_minimum_required(VERSION 2.8.3)
project(de_msgs)

find_package(catkin REQUIRED 
    COMPONENTS
        message_generation
)

add_message_files(
    FILES
        Mission.msg
        Waypoint.msg
)

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

