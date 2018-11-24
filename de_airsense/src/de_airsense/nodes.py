# -*- coding: utf-8 -*-

from . import agent 
from . import mission_dji
from . import mission_px4
from . import waspmote_serial

def agent_node():
    agent.Agent().spin()

def mission_dji_node():
    mission_dji.FlightMission().spin()

def mission_px4_node():
    mission_px4.FlightMission().spin()

def waspmote_serial_node():
    waspmote_serial.WaspmoteSerial().spin()