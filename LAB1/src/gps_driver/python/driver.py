#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import utm
import serial
from gps_driver.msg import gps_msg
import sys
def convert_to_degrees(value):
    return float(value)//100 + (float(value) - (float(value)//100)*100)/60

#def handle_gps_operations(gps_pub, secs, nsecs):
#    gps_data = gps_msg()
#    gps_data.Header.stamp.secs = secs
#    gps_data.Header.stamp.nsecs = nsecs
#    gps_data.Header.frame_id = "GPS_Frame"
#    gps_data.Latitude = float(convert_to_degrees(float(gpgga_data[2])))
#    gps_data.Longitude = convert_to_degrees(float(gpgga_data[4]))
#    gps_data.Altitude = float(gpgga_data[9])
#    gps_data.UTM_easting = utm_result[0]
#    gps_data.UTM_northing = utm_result[1]
#    gps_data.Zone = utm_result[2]
#    gps_data.Letter = utm_result[3]
#    gps_pub.publish(gps_data)
#    print(gps_data)

def parse_gps_data(line):
    line = line.decode('UTF-8')
    print(line)
    return line.split(",")

if __name__ == '__main__':
    rospy.init_node('gps_talker')
    port = rospy.get_param('/port', "/dev/ttyUSB0")
    
    serial_port = serial.Serial(port, 4800, timeout=300.0)

    gps_pub = rospy.Publisher('/gps', gps_msg, queue_size=10)

    try:
        while not rospy.is_shutdown():
            line = serial_port.readline()
            if line == '':
                rospy.logwarn("Serial Port: No valid data")
            else:
                if line.startswith(b'\r$GPGGA'):
                    gpgga_data = parse_gps_data(line)
                    time = gpgga_data[1].split('.')

                    secs = int(time[0]) % 10000
                    nsecs = int(time[1]) * pow(10,6)

                    e_w = -1 if gpgga_data[5] == 'W' else 1
                    n_s = -1 if gpgga_data[3] == 'S' else 1
                    utm_result = utm.from_latlon(n_s*convert_to_degrees(float(gpgga_data[2])), 
                                                  e_w*convert_to_degrees(float(gpgga_data[4])))
                    
                    gps_data = gps_msg()
                    gps_data.Header.stamp.secs = secs
                    gps_data.Header.stamp.nsecs = nsecs
                    gps_data.Header.frame_id = "GPS1_Frame"
                    gps_data.Latitude = float(convert_to_degrees(float(gpgga_data[2])))
                    gps_data.Longitude = convert_to_degrees(float(gpgga_data[4]))
                    gps_data.HDOP = float(gpgga_data[8])
                    gps_data.Altitude = float(gpgga_data[9])
                    gps_data.UTM_easting = utm_result[0]
                    gps_data.UTM_northing = utm_result[1]
                    gps_data.Zone = utm_result[2]
                    gps_data.Letter = utm_result[3]
                    gps_pub.publish(gps_data)
                    print(gps_data)
                    #handle_gps_operations(gps_pub, secs, nsecs)

    except rospy.ROSInterruptException:
        pass

