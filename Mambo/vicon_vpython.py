# #!/usr/bin/env python3.6
# Written by Isaac Vargas and Alan Garduno
import math
from re import X
import rospy
import numpy as np
from vpython import *
import time

from std_msgs.msg import Empty, String, UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped

import time
import _thread


def cb_vicon_data1(data):
    global rx1, ry1, rz1, rw1, x_1, y_1, z_1
    x_1 = data.transform.translation.x
    y_1 = data.transform.translation.y
    z_1 = data.transform.translation.z
    rx1 = data.transform.rotation.x
    ry1 = data.transform.rotation.y
    rz1 = data.transform.rotation.z
    rw1 = data.transform.rotation.w


def cb_vicon_data2(data):
    global x_2
    global y_2
    global z_2
    x_2 = data.transform.translation.x
    y_2 = data.transform.translation.y
    z_2 = data.transform.translation.z
    rx = data.transform.rotation.x
    ry = data.transform.rotation.y
    rz = data.transform.rotation.z
    rw = data.transform.rotation.w


def cb_vicon_data3(data):
    global x_3
    global y_3
    global z_3
    x_3 = data.transform.translation.x
    y_3 = data.transform.translation.y
    z_3 = data.transform.translation.z
    rx = data.transform.rotation.x
    ry = data.transform.rotation.y
    rz = data.transform.rotation.z
    rw = data.transform.rotation.w

    # print(x,y,z)

# ROS node


def vicon_sys():
    global rx1, ry1, rz1, rw1, x_1, y_1, z_1
    global x_2, y_2, z_2
    global x_3, y_3, z_3
# Init Vpython vars
    ybound = 4.4
    xbound = 7.5
    xfence = 7.5
    yfence = 4.4
    x_1 = 0
    y_1 = 0
    z_1 = 0

    x1_p1 = 0
    y1_p1 = 0
    z1_p1 = 0

    rx1 = 0
    ry1 = 0
    rz1 = 0
    rw1 = 0

    x_2 = 0
    y_2 = 0
    z_2 = 0

    x2_p1 = 0
    y2_p1 = 0
    z2_p1 = 0

    x_3 = 0
    y_3 = 0
    z_3 = 0
# Init vpython space
    floor = box(pos=vector(0.25, -0.1, -0.5), size=vector(xbound, 0.2, ybound))
    #vicon_fence= extrusion(path=[vector(-3.25,0,-2.7),vector(3.75,2,1.7)],shape=shapes.rectangle(width=1,height=1, rotate=pi),opacity=0.5)
    vicon_fence = box(pos=vector(0.25, 1, -0.5), size=vector(xfence, 2,
                      yfence), opacity=0.25, color=vector(221/255, 160/255, 221/255))
    item1 = sphere(pos=vector(0, 0, 0), radius=0.1, interval=10,
                   retain=50, color=vector(0, 0, 1))
    item2 = sphere(pos=vector(0, 0, 0), radius=0.1, interval=10,
                   retain=50, color=vector(0, 1, 0))
    item3 = sphere(pos=vector(0, 0, 0), radius=0.1, interval=10,
                   retain=50, color=vector(1, 0, 0))
    arrow1 = arrow(pos=vector(x_1, z_1, y_1), axis=vector(
        0, 0, 0), shaftwidth=0.05, color=vector(1, 1, 0))
    arrow2 = arrow(pos=vector(x_2, z_2, y_2), axis=vector(
        0, 0, 0), shaftwidth=0.05, color=vector(1, 1, 0))
    #arrow3 = arrow(pos =vector(x_3,z_3,y_3), axis=vector(0,0,0), shaftwidth=0.05, color=vector(1,1,0))
    arrowx = arrow(pos=vector(0, 0.1, 0), axis=vector(1, 0, 0),
                   shaftwidth=0.05, color=vector(0, 1, 1))  # turqoise
    arrowy = arrow(pos=vector(0, 0.1, 0), axis=vector(
        0, 0, 1), shaftwidth=0.05, color=vector(221/255, 160/255, 221/255))  # purple
    arrowz = arrow(pos=vector(0, 0.1, 0), axis=vector(0, 1, 0),
                   shaftwidth=0.05, color=vector(1, 165/255, 0))  # organge

    turn_arrow = arrow(pos=vector(x_1, z_1, y_1), axis=vector(
        0, 1, 0), shaftwidth=0.05, color=vector(1, 165/255, 0))  # organge

    item1_face = arrow(pos=vector(x_1, z_1, y_1), axis=vector(
        0, 0, 1), shaftwidth=0.05, color=vector(0.2, 0.8, 0.5))
    scene.camera.pos = vector(1.57, 2.82, 3.62)
    scene.camera.axis = vector(-1.57, -2.82, -3.62)
# Vicon callback
    # Init ROS node
    rospy.init_node('vicon_reader', anonymous=True)
    rate = rospy.Rate(100)

    # Choose which topic to listen
    s1_vicon = rospy.Subscriber(
        '/vicon/Mambo_1/Mambo_1', TransformStamped, cb_vicon_data1, buff_size=1)
    s3_vicon = rospy.Subscriber(
        '/vicon/Mambo_3/Mambo_3', TransformStamped, cb_vicon_data3, buff_size=1)
    s2_vicon = rospy.Subscriber(
        '/vicon/Mambo_5/Mambo_5', TransformStamped, cb_vicon_data2, buff_size=1)

    real_angle = 0
    mam_angle = 0
# MAIN
    while not rospy.is_shutdown():
        rate.sleep()
        roll, pitch, yaw = quaterion2euler(rx1, ry1, rz1, rw1)
        yaw = yaw*180/math.pi

        if yaw < 0:
            yaw = math.fabs(yaw)
        elif yaw > 0:
            yaw = -yaw

        delx1 = x_1-x1_p1
        delz1 = z_1-z1_p1
        dely1 = y_1-y1_p1

        delx2 = x_2-x2_p1
        delz2 = z_2-z2_p1
        dely2 = y_2-y2_p1

        prev_vel = vector((-delx1), (delz1), (dely1))*5
        prev_vel2 = vector((-delx2), (delz2), (dely2))*5

        arrow2.pos = vector(-x_2, z_2, y_2)
        arrow1.pos = vector(-x_1, z_1, y_1)

        arrow1.axis = prev_vel
        arrow2.axis = prev_vel2

        item1.pos = vector(-x_1, z_1, y_1)
        item2.pos = vector(-x_2, z_2, y_2)
        item3.pos = vector(-x_3, z_3, y_3)

        x1_p1 = (x1_p1)*0.90 + x_1*0.1
        y1_p1 = (y1_p1)*0.90 + y_1*0.1
        z1_p1 = (z1_p1)*0.90 + z_1*0.1

        x2_p1 = (x2_p1)*0.90 + x_2*0.1
        y2_p1 = (y2_p1)*0.90 + y_2*0.1
        z2_p1 = (z2_p1)*0.90 + z_2*0.1

        turn_angle = round(
            (math.atan2((y1_p1-y2_p1), (x1_p1-x2_p1))*180/math.pi), 6)

        if turn_angle < 0:
            real_angle = math.fabs(turn_angle)
        elif turn_angle > 0:
            real_angle = -turn_angle

        real_angle = real_angle - 90
        # print(real_angle)
        if real_angle < -180:
            real_angle = 180 - math.fabs(real_angle + 180)
        print(x_1, y_1)
        #print('----------------------------------Turn_angle = vector angle beteen two drones = ', real_angle)

        # print('Pos = ', scene.camera.pos, 'Axis: ', scene.camera.axis)
        # if (math.fabs(mam_angle) > 175 and math.fabs(-real_angle) > 175) or (math.fabs(-mam_angle) > 175 and math.fabs(-real_angle) > 175):
        #     diff = (180 - math.fabs(mam_angle)) + (180 - math.fabs(real_angle))
        #     if mam_angle > 0 and real_angle < 0:
        #         turn_thresh = diff
        #     elif mam_angle < 0 and real_angle > 0:
        #         turn_thresh = -diff
        # else:

        complement = (mam_angle*0 + yaw*1)
        # if (math.fabs(complement) > 175 and math.fabs(-real_angle) > 175) or (math.fabs(-complement) > 175 and math.fabs(-real_angle) > 175):
        #     diff = (180 - math.fabs(complement)) + (180 - math.fabs(real_angle))
        #     if (complement > 0) and (real_angle < 0):
        #         turn_thresh = diff
        #     elif (complement < 0 and real_angle > 0):
        #         turn_thresh = -diff
        # else:

        turn_thresh = (real_angle - complement)
        if turn_thresh < -180:
            turn_thresh = turn_thresh + 360
        #print('This is how much the Drone needs to turn =', turn_thresh)
        #turn_thresh = (real_angle - complement)

        if complement < 0:
            calc_angle = complement+360
        else:
            calc_angle = complement

        item1_x = 0.4*math.cos((calc_angle*math.pi/180))
        item1_y = 0.4*math.sin((calc_angle)*math.pi/180)

        turn_x = 0.4*math.cos((real_angle)*math.pi/180)
        turn_y = 0.4*math.sin((real_angle)*math.pi/180)

        item1_face.pos = vector(-x_1, z_1, y_1)
        # Arrow that shows where 'front' of item is pointing in frame
        item1_face.axis = vector(-item1_y, 0, item1_x)
        turn_arrow.pos = vector(-x_1, z_1, y_1)
        turn_arrow.axis = vector(-turn_y, 0, turn_x)

        # if (turn_thresh > 2 or turn_thresh < -2):
        #     #mambo.turn_degrees(turn_thresh)
        #     mam_angle = mam_angle + turn_thresh
        #     print('Turn "this" much = ', turn_thresh, "Angle in Space frame = ", mam_angle)

        #print('x = ',x_1,'y = ',y_1,'z = ',z_1)


def stay_insidebounds(xmin, xmax, ymin, ymax):
    global x_1, y_1, z_1


def quaterion2euler(rx, ry, rz, rw):

    sinr_cosp = 2*(rw*rx + ry*rz)
    cosr_cosp = 1 - 2*(rx*rx + ry*ry)

    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2*(rw * ry - rz * rx)
    if (math.fabs(sinp)) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2*(rw*rz + ry*rx)
    cosy_cosp = 1 - 2*(rz*rz + ry*ry)

    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# Main function
if __name__ == '__main__':
    try:
        vicon_sys()
    except rospy.ROSInterruptException:
        print("Exiting...")
