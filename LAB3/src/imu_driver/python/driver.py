#!/usr/bin/env python3

import rospy
import serial
from imu_driver.msg import imu_msg
from imu_driver.srv import convert_to_quaternion_srv, convert_to_quaternion_srvResponse

def converter(yaw, pitch, roll):
    return convert_to_quaternion_srvResponse(yaw, pitch, roll)

if __name__ == "__main__":
    rospy.init_node("imu")
    pub = rospy.Publisher("/imu", imu_msg, queue_size=10)
    # rate = rospy.Rate(40)                                                      
    serial_port = rospy.get_param("/port", default="/dev/ttyUSB0")
    ser = serial.Serial(port=serial_port, baudrate=115200)

    output_msg = imu_msg()

    system_time = rospy.get_rostime()
    system_secs = system_time.secs
    system_nsecs = system_time.nsecs
    
    hz = "$VNWRG,75,20*XX"          #800/20 = 40Hz
    data_out = "$VNWRG,75,20*XX"

    ser.write(hz.encode('utf-8'))
    ser.write(data_out.encode('utf-8'))

    while not rospy.is_shutdown():
        raw_string = (ser.readline()).decode('utf-8').strip()
        spt_string = raw_string.split(",")
        # rospy.loginfo(raw_string)

        try:
            if(spt_string[0] == '$VNYMR'):
                output_msg.Header.stamp.secs = system_secs
                output_msg.Header.stamp.nsecs = system_nsecs
                output_msg.Header.frame_id = "IMU1_Frame"
                output_msg.raw_IMU = raw_string

                yaw = float(spt_string[1])
                pitch = float(spt_string[2])
                roll = float(spt_string[3])

                rospy.wait_for_service("convert_to_quaternion")
                try:
                    conversion = rospy.ServiceProxy("convert_to_quaternion", convert_to_quaternion_srv)
                    response = conversion(yaw, pitch, roll)

                except rospy.ServiceException as e:
                    print("Service call failed: %s" %e)
                    rospy.logwarn(e)

                output_msg.MagField.magnetic_field.x = float(spt_string[4])
                output_msg.MagField.magnetic_field.y = float(spt_string[5])
                output_msg.MagField.magnetic_field.z = float(spt_string[6])

                output_msg.IMU.linear_acceleration.x = float(spt_string[7])
                output_msg.IMU.linear_acceleration.y = float(spt_string[8])
                output_msg.IMU.linear_acceleration.z = float(spt_string[9])

                output_msg.IMU.angular_velocity.x = float(spt_string[10])
                output_msg.IMU.angular_velocity.y = float(spt_string[11])
                output_msg.IMU.angular_velocity.z = float((spt_string[12].split("*"))[0])

                output_msg.IMU.orientation.w = float(response.qw)
                output_msg.IMU.orientation.x = float(response.qx)
                output_msg.IMU.orientation.y = float(response.qy)
                output_msg.IMU.orientation.z = float(response.qz)

                pub.publish(output_msg)
                rospy.loginfo(output_msg)

            else:
                pass
        except rospy.ROSInterruptException:
            pass	
        except ValueError as v:
           pass
        except IndexError as i:
        	pass
