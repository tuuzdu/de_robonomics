cmake_minimum_required(VERSION 2.8.3)
project(de_airsense)

find_package(catkin REQUIRED 
    COMPONENTS
        dji_sdk
        mavros
        rospy
        de_msgs
        std_msgs
        sensor_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/agent_node
        scripts/mission_dji_node
        scripts/mission_px4_node
        scripts/waspmote_serial_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/mission.launch
        launch/dji.launch
        launch/px4.launch
        launch/waspmote.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
