# -*- coding: utf-8 -*-

import serial
from de_msgs.msg import WaspmoteSensors, Sensor
import rospy
from threading import Thread
from collections import namedtuple

class WaspmoteSerial:

    SensData = namedtuple('SensData', ['name', 'value', 'unit'])
    units = dict(CO2 = 'ppm', CO = 'ppm', NO = 'ppm', SO2 = 'ppm', CH4 = '% LEL', TC = 'Cel', HUM = '%', PRES = 'Pa', GPS = 'deg')

    def __init__(self):

        rospy.loginfo('Starting waspmote gas sensors...')
        rospy.loginfo('Waiting for ROS services...')
        rospy.init_node('de_airsense_waspmote')

        serial_name = rospy.get_param('~serial_name')
        baud_rate = rospy.get_param('~baud_rate')

        self.sensor_pub = rospy.Publisher('~sensor/measurements', WaspmoteSensors, queue_size=10)
        self.sensors_msg = WaspmoteSensors()

        self.serial_port = serial.Serial( 
            serial_name, 
            baud_rate, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS
            )

        thread = Thread(target=self.serial_receiver, daemon=True).start()

    def serial_receiver(self):
        STOP_SYMBOL = b'\n'
        rate = rospy.Rate(5)
        frame = str()
        waspmote_ready = False
        while not rospy.is_shutdown():
            try:    
                while self.serial_port.in_waiting > 0:
                    if waspmote_ready == False:
                        waspmote_ready = True
                        rospy.loginfo('Waspmote gas sensors and ROS are ready')
                    byte = self.serial_port.read()

                    if byte == STOP_SYMBOL:
                        frame  = frame [5:]
                        data = frame.split('#')
                        self.frame_parse(data)
                        frame = str()
                        continue

                    if byte != b'\x86' and byte != b'\x00':
                        frame += byte.decode()

                if len(self.sensors_msg.waspmote_sensors) > 6:  # waspmote frames don't more than 6 fields
                    self.sensor_pub.publish(self.sensors_msg)
                    self.sensors_msg.waspmote_sensors = []
                rate.sleep()

            except KeyboardInterrupt: 
                rospy.loginfo('\nExit')
                break

    def frame_parse(self, data):
        for part in data:
            if part.find(':') != -1:
                data_array = part.split(':')
                if data_array[0] in self.units.keys():
                    if data_array[0] == 'GPS':
                        lat = data_array[1].split(';')[0]
                        lon = data_array[1].split(';')[1]
                        rospy.logdebug(self.SensData('GPS_LAT', lat, self.units[data_array[0]]))
                        rospy.logdebug(self.SensData('GPS_LON', lon, self.units[data_array[0]]))
                        self.append_sensor('GPS_LAT', lat, self.units[data_array[0]])
                        self.append_sensor('GPS_LON', lon, self.units[data_array[0]])
                    else:
                        rospy.logdebug(self.SensData(data_array[0], data_array[1], self.units[data_array[0]]))
                        self.append_sensor(data_array[0], data_array[1], self.units[data_array[0]])

    def append_sensor(self, name, value, unit):
        self.sensors_msg.waspmote_sensors.append(Sensor(
            name=str(name), 
            value=float(value), 
            unit=str(unit)
        ))

    def spin(self):
        rospy.spin()