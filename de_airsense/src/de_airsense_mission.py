#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from dji_sdk.srv import SDKControlAuthority, MissionWpUpload, MissionWpAction
from dji_sdk.msg import MissionWaypointAction, MissionWaypointTask, MissionWaypoint
from de_msgs import Mission

def waypoints_create (mission_msg):
    rospy.loginfo('Received mission from objective: ')
    rospy.loginfo(mission_msg)
    newWaypointList = []
    for index in range(len(mission_msg.waypoints)):
        cmd_parameter = [mission_msg.waypoints[index].staytime * 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        newWaypointList.append(MissionWaypoint( latitude = mission_msg.waypoints[index].latitude, 
                                                            longitude = mission_msg.waypoints[index].longitude, 
                                                            altitude = mission_msg.waypoints[index].altitude, 
                                                            damping_distance = 2, 
                                                            target_yaw = 0, 
                                                            has_action = 1, 
                                                            target_gimbal_pitch = 0, 
                                                            turn_mode = 0, 
                                                            action_time_limit = 64000,
                                                            waypoint_action = MissionWaypointAction(
                                                                action_repeat = 10,
                                                                command_parameter = cmd_parameter)))
    return newWaypointList

def mission_start(mission_msg):
    try:
        auth = rospy.ServiceProxy('dji_sdk/sdk_control_authority', SDKControlAuthority)
        resp = auth(1)
        rospy.loginfo('Service. Sdk control authority:')
        rospy.loginfo(resp.result)

    except rospy.ServiceException, e:
        rospy.loginfo(e)
       
    try:
        mission = rospy.ServiceProxy('dji_sdk/mission_waypoint_upload', MissionWpUpload)
        mission_task_msg = MissionWaypointTask()
        mission_task_msg.velocity_range     = 10;
        mission_task_msg.idle_velocity      = 10;
        mission_task_msg.action_on_finish   = MissionWaypointTask.FINISH_RETURN_TO_HOME;
        mission_task_msg.mission_exec_times = 1;
        mission_task_msg.yaw_mode           = MissionWaypointTask.YAW_MODE_AUTO;
        mission_task_msg.trace_mode         = MissionWaypointTask.TRACE_POINT;
        mission_task_msg.action_on_rc_lost  = MissionWaypointTask.ACTION_AUTO;
        mission_task_msg.gimbal_pitch_mode  = MissionWaypointTask.GIMBAL_PITCH_FREE;

        mission_task_msg.mission_waypoint = waypoints_create(mission_msg)

        rospy.loginfo(mission_task_msg)
        resp = mission(mission_task_msg)
        rospy.loginfo('Service. Mission waypoint upload:')
        rospy.loginfo(resp.result)

    except rospy.ServiceException, e:
        rospy.loginfo(e)
 
    try:
        start = rospy.ServiceProxy('dji_sdk/mission_waypoint_action', MissionWpAction)
        resp = start(0)
        rospy.loginfo('Service. Mission waypoint action:')
        rospy.loginfo(resp.result)

    except rospy.ServiceException, e:
        rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node('de_airsense_mission')
    rospy.loginfo('Waiting dji_sdk for services...')
    rospy.wait_for_service('dji_sdk/sdk_control_authority')
    rospy.wait_for_service('dji_sdk/mission_waypoint_action')
    rospy.wait_for_service('dji_sdk/mission_waypoint_upload')
    rospy.Subscriber('objective/mission', Mission, mission_start)

    rospy.spin()
