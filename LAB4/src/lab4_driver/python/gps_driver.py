#!/usr/bin/env python3
import utm
from serial import Serial
import rospy
from imu_gps.msg import gps_msg
import sys

# lat 42.33709
#lon -71.090029

def driver():

    argument = rospy.myargv(argv=sys.argv)
    number = argument[1]
    port1 = Serial(number, 4800, timeout=100.00)

    msg = gps_msg()
    msg.Header.frame_id = 'GPS1_Frame'
    # seq = -1
    rospy.init_node('driver', anonymous=True)
    pub = rospy.Publisher('/gps', gps_msg, queue_size=10)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        data = str(port1.readline())

        if 'GPGGA' in data:

            values = data.split(',')
            # print(values[2], values[4])
            utc, latitude, longitude, hdop, altitude = values[1], values[2], values[4],values[8], values[9]
            if latitude != "" and longitude != "" and hdop != "":
                latitude = float(latitude[:2])+(float(latitude[2:])/60)
                longitude = (float(longitude[:3])+(float(longitude[3:])/60))*-1

                a = utm.from_latlon(latitude, longitude)
                # print(utc[7:])
                # print(utc)
                UTC = float(utc)
                UTC_hour = int(UTC/10000)
                #print(UTC_hour)
                UTC_minutes = int((UTC - UTC_hour*10000)/100)
                #print(UTC_minutes)
                UTC_seconds = int(UTC - UTC_hour*10000 - UTC_minutes*100)
                #print(UTC_seconds)
                UTC_total_seconds = float(UTC_hour*3600 + UTC_minutes*60 + UTC_seconds)
                #print(UTC_total_seconds)
                UTC_nano_seconds = int((UTC - UTC_hour*10000 - UTC_minutes*100 - UTC_seconds)*(10**9))

                msg.Header.seq += 1
                msg.Header.stamp.secs = int(UTC_total_seconds)
                msg.Header.stamp.nsecs = UTC_nano_seconds
                msg.Latitude = latitude
                msg.Longitude = longitude
                msg.Altitude = float(altitude)
                msg.HDOP = float(hdop)
                msg.UTM_northing = a[1]
                msg.UTM_easting = a[0]
                msg.Zone = a[2]
                msg.Letter = a[3]
                msg.UTC = float(utc)    

                #print(msg)

                pub.publish(msg)
                rate.sleep()
            else:
                pass

if  __name__ == "__main__":
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
        
