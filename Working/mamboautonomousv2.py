# #!/usr/bin/env python3.6
import math
import rospy
import numpy as np
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
    global rx1
    global ry1
    global rz1
    global rw1
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
def doyawangle(yaw):
    global y_1, y_2, x_1, x_2
    real_angle = 0
    turn_thresh = 0

    turn_angle = round((math.atan2((y_1-y_2), (x_1-x_2))*180/math.pi), 6)

    if turn_angle < 0:
        real_angle = math.fabs(turn_angle)
    elif turn_angle > 0:
        real_angle = -turn_angle

    real_angle = real_angle - 90

    if real_angle < -180:
        real_angle = 180 - math.fabs(real_angle + 180)

    complement = (yaw*1)
    turn_thresh = (real_angle - complement)

    if turn_thresh < -180:
        turn_thresh = turn_thresh + 360
    elif turn_thresh > 180:
        turn_thresh = turn_thresh - 360

    if math.fabs(turn_thresh) > 20:  # and (math.fabs(turn_thresh) < 180):
        if turn_thresh > 0:
            y = 100
        elif turn_thresh < 0:
            y = -100
    else:
        y = 0

    return y, complement
def doforward(x1, x2, y1, y2, pgain):
    global minSD 
    global maxSD 
    maxSD = 1
    minSD = 0.4

    distance = math.sqrt(math.pow((x2-x1), 2)+math.sqrt(math.pow((y2-y1), 2)))

    if distance > maxSD:
        p = pgain*100
    elif distance > minSD:
        p = 0
    else: 
        p = 0

    return p, distance  # Distance is "error"
def dovertical(z1, z2):
    global tko
    global land
    print('Autonomous height = ', z1, 'Remote controlled height', z2)
    #pid = [0.5, 0.5, 0.5]
    diff = z1 - z2 # this is also the error
    #v = np.clip(pid[0]*diff, )
    if math.fabs(diff) > 0.3:
        if diff < 0:  # z2(remote) object is higher in evelvation
            ver = 35  # movedrone higher
        elif diff > 0:  # z1(autno) object is higher
            ver = -35  # movedrone lower
    else:
        ver = 0  # do nothing
    if math.fabs(diff) > 1.20:
        tko = True
        print('Take off')
    if z2 < 0.1:
        land = True
        print('Land')

    return ver
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

    yaw = yaw*180/math.pi

    if yaw < 0:
        yaw = math.fabs(yaw)
    elif yaw > 0:
        yaw = -yaw

    print('Vicon Yaw =', round(yaw, 2))

    return roll, pitch, yaw
def mambo_init():
    global tko, land
    # Variable that will store the parameters of the mambo drone
    mamboAdd = "E0:14:A0:AE:3D:C7"
    wifi = False
    retries = 12

    # The mambo object
    mambo = Mambo("", use_wifi=wifi)

    # Checks if the drone is able to communicate with ROS
    success = False

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

    mambo = Mambo(mamboAdd, use_wifi=wifi)
    success = mambo.connect(retries)
    if(success):
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)
        mambo.flat_trim()
        rate = rospy.Rate(100)
        land = False
        tko = True

    return mambo, success 
def pitchcontroller(x1, y1, x2, y2,prev_dis):
    
    dist = math.sqrt(math.pow((x2-x1), 2)+math.sqrt(math.pow((y2-y1), 2)))
    
    pgain_pitch = (math.fabs(dist-prev_dis)*0.7 + dist*0.3) #somthing

    return pgain_pitch
def vicon_sys():
    global x_3, y_3, z_3 #item number 3 x,y,z location
    global x_2, y_2, z_2 #item number 2 x,y,z location
    global x_1, y_1, z_1, rw1, rx1, ry1, rz1 #item number 1 x,y,z location

    global tko, land

    x_1 = 0
    y_1 = 0
    z_1 = 0

    rx1 = 0
    ry1 = 0
    rz1 = 0
    rw1 = 0

    x_2 = 0
    y_2 = 0
    z_2 = 0

    x_3 = 0
    y_3 = 0
    z_3 = 0

    r = 0
    p = 0
    y = 0
    v = 0
    pgain_pitch = 0
    prev_dis = 0

    # Init ROS node
    rospy.init_node('vicon_reader', anonymous=True)
    rate = rospy.Rate(100)

    # Choose which topic to listen
    s1_vicon = rospy.Subscriber('/vicon/Mambo_Autonomous/Mambo_Autonomous', TransformStamped, cb_vicon_data1, buff_size=1)
    s3_vicon = rospy.Subscriber('/vicon/SUMO_11_5_2018/SUMO_11_5_2018/', TransformStamped, cb_vicon_data3, buff_size=1)
    s2_vicon = rospy.Subscriber('/vicon/Mambo_KeyboardControlled/Mambo_KeyboardControlled', TransformStamped, cb_vicon_data2, buff_size=1)

    mambo,_ = mambo_init()

    while not rospy.is_shutdown():
        rate.sleep()

        if tko == True:
            mambo.safe_takeoff(2)
            tko = False
        if land == True:
            mambo.safe_land(2)
            land = False

        roll, pitch, yaw = quaterion2euler(rx1, ry1, rz1, rw1)

        v = dovertical(z_1, z_2)
        y,_ = doyawangle(yaw)

        pgain_pitch = pitchcontroller(x_1,y_1,x_2,y_2,prev_dis)

        if y == 0:
            p, prev_dis = doforward(x_1, x_2, y_1, y_2, pgain_pitch)
        else:
            p =0

        mambo.fly_direct(roll=(r), pitch=(p), yaw=(y), vertical_movement=(v), duration=0.01)

        
# Main function
if __name__ == '__main__':
    try:
        vicon_sys()
    except rospy.ROSInterruptException:
        print("Exiting...")
