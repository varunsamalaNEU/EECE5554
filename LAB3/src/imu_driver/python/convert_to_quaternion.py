#!/usr/bin/env python3

import rospy
from imu_driver.srv import convert_to_quaternion_srv, convert_to_quaternion_srvResponse
import numpy as np
from math import pi

def converter(req):
    yaw_rad = float(req.yaw)*pi/180
    pitch_rad = float(req.pitch)*pi/180
    roll_rad = float(req.roll)*pi/180

    cy = np.cos(yaw_rad/2)
    cp = np.cos(pitch_rad/2)
    cr = np.cos(roll_rad/2)
    sy = np.sin(yaw_rad/2)
    sp = np.sin(pitch_rad/2)
    sr = np.sin(roll_rad/2)

    qw = (cr * cp * cy) + (sr * sp * sy)
    qx = (sr * cp * cy) - (cr * sp * sy)
    qy = (cr * sp * cy) + (sr * cp * sy)
    qz = (cr * cp * sy) - (sr * sp * cy)

    return convert_to_quaternion_srvResponse(qw, qx, qy, qz)


def convert_to_quaternion_service():
    rospy.init_node("convert_to_quaternion")
    s = rospy.Service("convert_to_quaternion", convert_to_quaternion_srv, converter)
    rospy.spin()

if __name__ == "__main__":
    convert_to_quaternion_service()