# #!/usr/bin/env python3.6
import math
import rospy
import numpy as np
from vpython import *
import time 
import tf2_py
from std_msgs.msg import Empty, String, UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped

from pyparrot.Minidrone import Mambo
#from vicon_python import *
import time
import _thread



def cb_vicon_data1(data):
    global x_1
    global y_1
    global z_1
    global rw1
    global rx1
    global ry1
    global rz1
    x_1 = data.transform.translation.x
    y_1 =data.transform.translation.y
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
    
    #print(x,y,z)

# ROS node
def vicon_sys():
    global x_3
    global y_3
    global z_3

    global x_2
    global y_2
    global z_2

    global x_1
    global y_1
    global z_1
    global rw1
    global rx1
    global ry1
    global rz1


    global tko
    global land

    # Variable that will store the parameters of the mambo drone
    # mamboAdd = "E0:14:A0:AE:3D:C7"
    # wifi = False
    # retries = 12

    # # The mambo object
    # mambo = Mambo("", use_wifi=wifi)

    # # Checks if the drone is able to communicate with ROS
    # success = False

    # Variables of the drone
    tko = False
    land = False
    cannon = False
    auto_tko = False
    need_to_change_mode = False
    need_to_toggle_mode = False
    p_mode = 0
    linX = 0
    linY = 0
    Alt = 0
    Hdg = 0

# Preffered piloting mode function
# #Init Vpython
    globalxbound = 6
    ybound = 6
    xbound = 6
    x_1 = 0
    y_1 = 0
    z_1 = 0
    x1_p1 = 0
    y1_p1 = 0
    z1_p1 = 0
    rx1 = 0
    ry1 = 0
    rz1 = 0

    x_2 = 0
    y_2 = 0
    z_2 = 0
    x2_p1 = 0
    y2_p1 = 0
    z2_p1 = 0

    x_3 = 0
    y_3 = 0
    z_3 = 0

    floor = box(pos=vector(0,-0.05,0),size = vector(xbound,0.2, ybound))
    item1 = sphere(pos=vector(0,0,0), radius=0.1,interval =10,retain=50, color=vector(0,0,1))
    item2 = sphere(pos=vector(0,0,0), radius=0.1,interval =10,retain=50, color=vector(0,1,0))
    item3 = sphere(pos=vector(0,0,0), radius=0.1,interval =10,retain=50, color=vector(1,0,0))
    arrow1 = arrow(pos =vector(x_1,z_1,y_1), axis=vector(0,0,0), shaftwidth=0.05, color=vector(1,1,0))
    arrow2 = arrow(pos =vector(x_2,z_2,y_2), axis=vector(0,0,0), shaftwidth=0.05, color=vector(1,1,0))
    #arrow3 = arrow(pos =vector(x_3,z_3,y_3), axis=vector(0,0,0), shaftwidth=0.05, color=vector(1,1,0))
    arrowx = arrow(pos =vector(0,0.1,0), axis=vector(1,0,0), shaftwidth=0.05, color=vector(0,1,1))#turqoise
    arrowy = arrow(pos =vector(0,0.1,0), axis=vector(0,0,1), shaftwidth=0.05, color=vector(221/255,160/255,221/2))#purple
    arrowz = arrow(pos =vector(0,0.1,0), axis=vector(0,1,0), shaftwidth=0.05, color=vector(1,165/255,0))#organge
    item1_face = arrow(pos =vector(x_1,z_1,y_1), axis=vector(0,0,1), shaftwidth=0.05, color=vector(0.2,0.8,0.5))
    scene.camera.pos = vector(1.57,2.82,3.62)
    scene.camera.axis = vector(-1.57,-2.82,-3.62)
# Vicon callback
    # Init ROS node
    rospy.init_node('vicon_reader',anonymous=True)
    rate = rospy.Rate(100)

    # Choose which topic to listen
    s1_vicon = rospy.Subscriber('/vicon/Mambo_Autonomous/Mambo_Autonomous', TransformStamped, cb_vicon_data1, buff_size=1)
    s3_vicon = rospy.Subscriber('/vicon/SUMO_11_5_2018/SUMO_11_5_2018/', TransformStamped, cb_vicon_data3,buff_size=1)
    s2_vicon = rospy.Subscriber('/vicon/Mambo_KeyboardControlled/Mambo_KeyboardControlled', TransformStamped, cb_vicon_data2, buff_size=1)
    marktime = time.time()


    # mambo = Mambo(mamboAdd, use_wifi=wifi)
    # success = mambo.connect(retries)
    # if(success):
    #     mambo.smart_sleep(2)
    #     mambo.ask_for_state_update()
    #     mambo.smart_sleep(2)
    #     mambo.flat_trim()
    #     rate = rospy.Rate(100)
    #     land = False
    #     tko = True
    real_angle = 0
    mam_angle = 0

#plotPP(x,y,z)
    while not rospy.is_shutdown():

        roll, pitch, yaw = quaterion2euler(rx1,ry1,rz1,rw1)

        yaw = yaw*180/math.pi

        if yaw < 0:
            yaw = math.fabs(yaw)
        elif yaw > 0:
            yaw = -yaw

        # if tko == True:
        #     mambo.safe_takeoff(2)   
        #     tko = False

        # if land == True:
        #     mambo.safe_land(2)
        #     land = False

        rate.sleep()

        delx1 = x_1-x1_p1
        delz1 = z_1-z1_p1
        dely1 = y_1-y1_p1

        delx2 = x_2-x2_p1
        delz2 = z_2-z2_p1
        dely2 = y_2-y2_p1

        prev_vel = vector((delx1),(delz1),(dely1))*2
        prev_vel2 = vector((delx2),(delz2),(dely2))*2

        arrow2.pos = vector(x_2,z_2,y_2)
        arrow1.pos = vector(x_1,z_1,y_1)

        arrow1.axis = prev_vel
        arrow2.axis = prev_vel2

        item1.pos = vector(x_1,z_1,y_1)
        item2.pos = vector(x_2,z_2,y_2)
        item3.pos = vector(x_3,z_3,y_3)

        x1_p1 = (x1_p1)*0.85 + x_1*0.15
        y1_p1 = (y1_p1)*0.85 + y_1*0.15
        z1_p1 = (z1_p1)*0.85 + z_1*0.15

        x2_p1 = (x2_p1)*0.85 + x_2*0.15
        y2_p1 = (y2_p1)*0.85 + y_2*0.15
        z2_p1 = (z2_p1)*0.85 + z_2*0.15

        
        #mambo.fly_direct(roll = (0), pitch = (0),yaw = (0), vertical_movement = (0), duration=0.01)
        turn_angle = round((atan2((y1_p1-y2_p1), (x1_p1-x2_p1))*180/pi), 6)  

        #turn_angle = round((atan2((y2_p1-y1_p1), (x2_p1-x1_p1))*180/pi), 6)
        #turn_angle = turn_angle + 90
        
        if turn_angle < 0:
            real_angle = math.fabs(turn_angle)
        elif turn_angle > 0:
            real_angle = -turn_angle
        
        real_angle = real_angle - 90
        #print(real_angle)
        if real_angle < -180:
            real_angle = 180 - math.fabs(real_angle + 180)
        print('----------------------------------Turn_angle = vector angle beteen two drones = ', real_angle)


        # print('Pos = ', scene.camera.pos, 'Axis: ', scene.camera.axis)
        # if math.fabs(mam_angle) > 178 or math.fabs(real_angle) > 178:
        #     diff = (180 - math.fabs(mam_angle)) + (180 - math.fabs(real_angle))
        #     if mam_angle > 0 and real_angle < 0:
        #         turn_thresh = -diff
        #     elif mam_angle < 0 and real_angle > 0:
        #         turn_thresh = diff
        # else:
        #     turn_thresh = (real_angle - mam_angle)
        complement = (mam_angle*0.2 + yaw*0.8)
        if (math.fabs(complement) > 175 and math.fabs(-real_angle) > 175) or (math.fabs(-complement) > 175 and math.fabs(-real_angle) > 175):
            diff = (180 - math.fabs(complement)) + (180 - math.fabs(real_angle))
            if (complement > 0) and (real_angle < 0):
                turn_thresh = diff
            elif (complement < 0 and real_angle > 0):
                turn_thresh = -diff
        else:

            turn_thresh = (real_angle - complement)
        

        if mam_angle < 0:
            calc_angle = mam_angle+360
        else:
            calc_angle = mam_angle

        item1_x = 0.4*cos((calc_angle*pi/180))
        item1_y = 0.4*sin((calc_angle)*pi/180)
        
        item1_face.pos =vector(x_1,z_1,y_1)
        item1_face.axis =vector(item1_y,0,item1_x) #Arrow that shows where 'front' of item is pointing in frame

        if turn_thresh > 2 or turn_thresh < -2: #and (math.fabs(turn_thresh) < 180): 
            mam_angle = complement + turn_thresh

        # if turn_thresh > 2 or turn_thresh < -2: 
        #     # mambo.turn_degrees(turn_thresh)
        #     mam_angle = mam_angle + turn_thresh
        print('Turn "this" much = ', turn_thresh, "Angle in Space frame = ", mam_angle)
        #print('rx1 = ', round(rx1,2), 'ry1 = ', round(ry1,2), 'rz1 = ', round(rz1,2))
        # rpx = sin(rx1)*cos(ry1)
        # rpy = sin(rx1)*sin(ry1)
        # arrow3.axis= 0.7*vector(rpx, 0, rpy)#unitvector1 in direction of rx
        # arrow3.pos =vector(x_1,z_1,y_1)

def quaterion2euler(rx, ry, rz, rw):

    sinr_cosp = 2*(rw*rx + ry*rz)
    cosr_cosp = 1 - 2*(rx*rx + ry*ry)

    roll  = math.atan2(sinr_cosp, cosr_cosp)
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
