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
# Globals
    global x_1, y_1, z_1
    global x_2, y_2, z_2
    global x_3, y_3, z_3
    x_1 = y_1 = z_1 = 0
    x_2 = y_2 = z_2 = 0
    x_3 = y_3 = z_3 = 0
# Init Vpython vars
    ybound = 4.4
    xbound = 7.5
    xfence = 7.5
    yfence = 4.4

    pos_vec_prev_mambo1 = np.array([0,0,0])
    pos_vec_prev_mambo2 = np.array([0,0,0])
    pos_vec_prev_mambo3 = np.array([0,0,0])

    vel_mambo1 = vector(0,0,0)
    vel_mambo2 = vector(0,0,0)
    vel_mambo3 = vector(0,0,0)
# Init vpython space
    time.sleep(1)
    scene = canvas(width=1280, height=800, center=vector(0,0,0), background=color.white)

    floor = box(pos=vector(0.25, 0.05, -0.5), size=vector(xbound, 0.05, ybound), color = vector(100/255,100/255,100/255))
    #vicon_fence= extrusion(path=[vector(-3.25,0,-2.7),vector(3.75,2,1.7)],shape=shapes.rectangle(width=1,height=1, rotate=pi),opacity=0.5)
    vicon_fence = box(pos=vector(0.25, 1.05, -0.5), size=vector(xfence, 2, yfence), opacity=0.25, color=vector(120/255,120/255,120/255))#(221/255, 160/255, 221/255)
    item1 = sphere(pos=vector(x_1, y_1, z_1), radius=0.08, interval=10, color=vector(0, 0, 1), make_trail = False, trail_color = vector(0, 0, 1), retain = 5000)
    item2 = sphere(pos=vector(x_2, y_2, z_2), radius=0.08, interval=10, color=vector(0, 1, 0), make_trail = False, trail_color = vector(0, 1, 0), retain = 5000)
    item3 = sphere(pos=vector(x_3, y_3, z_3), radius=0.08, interval=10, color=vector(1, 0, 0), make_trail = False, trail_color = vector(1, 0, 0), retain = 5000)

    item1_arrow = arrow(pos=vector(x_1, z_1, y_1), axis=vector(0, 0, 0), shaftwidth=0.05, color=vector(1, 1, 0))
    item2_arrow = arrow(pos=vector(x_2, z_2, y_2), axis=vector(0, 0, 0), shaftwidth=0.05, color=vector(1, 1, 0))
    item3_arrow = arrow(pos=vector(x_3, z_3, y_3), axis=vector(0, 0, 0), shaftwidth=0.05, color=vector(1, 1, 0))

    #arrow3 = arrow(pos =vector(x_3,z_3,y_3), axis=vector(0,0,0), shaftwidth=0.05, color=vector(1,1,0))
    arrowx = arrow(pos=vector(0, 0.07, 0), axis=vector(0.5, 0, 0), shaftwidth=0.03, color=vector(0, 0.8, 1))  # turqoise
    arrowy = arrow(pos=vector(0, 0.07, 0), axis=vector(0, 0, 0.5), shaftwidth=0.03, color=vector(221/255, 160/255, 221/255))  # purple
    arrowz = arrow(pos=vector(0, 0.07, 0), axis=vector(0, 0.5, 0), shaftwidth=0.03, color=vector(1, 165/255, 0))  # organge

    scene.camera.pos = vector(-2.19266, 3.02654, 3.31792)
    scene.camera.axis = vector(2.19266, -3.02654, -3.31792)
# Vicon callback
    # Init ROS node
    rospy.init_node('vicon_reader', anonymous=True)
    rate = rospy.Rate(100)

    # Choose which topic to listen
    s1_vicon = rospy.Subscriber('/vicon/Mambo_1/Mambo_1', TransformStamped, cb_vicon_data1, buff_size=1)
    s3_vicon = rospy.Subscriber('/vicon/Mambo_3/Mambo_3', TransformStamped, cb_vicon_data3, buff_size=1)
    s2_vicon = rospy.Subscriber('/vicon/Mambo_5/Mambo_5', TransformStamped, cb_vicon_data2, buff_size=1)
# MAIN
    while True:
        rate.sleep()
        trail(item1, item2, item3)
        
        #Update position in Vpython
        item1.pos = vector(-x_1, z_1, y_1)
        item2.pos = vector(-x_2, z_2, y_2)
        item3.pos = vector(-x_3, z_3, y_3)

        item1_arrow.pos = vector(-x_1, z_1, y_1)
        item2_arrow.pos = vector(-x_2, z_2, y_2)
        item3_arrow.pos = vector(-x_3, z_3, y_3)

        item1_arrow.axis = vel_mambo1
        item2_arrow.axis = vel_mambo2
        item3_arrow.axis = vel_mambo3

        
        #Store position in row vector
        pos_vec_mambo1 = np.array([x_1,y_1,z_1])
        pos_vec_mambo2 = np.array([x_2,y_2,z_2])
        pos_vec_mambo3 = np.array([x_3,y_3,z_3])
        #Get Velocity vector 
        vec_mambo1 = get_vec(pos_vec_mambo1, pos_vec_prev_mambo1) 
        vec_mambo2 = get_vec(pos_vec_mambo2, pos_vec_prev_mambo2) 
        vec_mambo3 = get_vec(pos_vec_mambo3, pos_vec_prev_mambo3) 

        vel_mambo1 = vector((-vec_mambo1[0]), vec_mambo1[2], vec_mambo1[1])*5
        vel_mambo2 = vector((-vec_mambo2[0]), vec_mambo2[2], vec_mambo2[1])*5
        vel_mambo3 = vector((-vec_mambo3[0]), vec_mambo3[2], vec_mambo3[1])*5

        print(scene.camera.pos,scene.camera.axis)
        #Store/Update prev value through LP filter
        pos_vec_prev_mambo1 = np.add(pos_vec_prev_mambo1*0.9 , pos_vec_mambo1*0.1)
        pos_vec_prev_mambo2 = np.add(pos_vec_prev_mambo2*0.9 , pos_vec_mambo2*0.1)
        pos_vec_prev_mambo3 = np.add(pos_vec_prev_mambo3*0.9 , pos_vec_mambo3*0.1)



def get_vec(pos_coords, prev_coords):
    
    vector_ = np.subtract(pos_coords, prev_coords)

    return vector_
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

def trail(item1, item2, item3):
    if (x_1 == 0 and y_1 == 0 and z_1 == 0):
        item1.make_trail = False
    else:
        item1.make_trail = True

    if (x_2 == 0 and y_2 == 0 and z_2 == 0):
        item2.make_trail = False
    else:
        item2.make_trail = True


    if (x_3 == 0 and y_3 == 0 and z_3 == 0):
        item3.make_trail = False
    else:
        item3.make_trail = True

# Main function
if __name__ == '__main__':
    try:
        vicon_sys()
    except rospy.ROSInterruptException:
        print("Exiting...")
