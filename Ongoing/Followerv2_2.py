# #!/usr/bin/env python3.6
# Written by Isaac Vargas and Alan Garduno
# This is "Keyboard Controlled"
import math
import rospy
from geometry_msgs.msg import TransformStamped
from pyparrot.Minidrone import Mambo
#from vicon_python import *
import time
import keyboard

def mambo_init():
    global tko, land
    # Variable that will store the parameters of the mambo drone
    #mamboAdd = "D0:3A:EE:30:E6:20" # ViconObject = Mambo_1
    mamboAdd = "E0:14:A0:AE:3D:C7" # ViconObject = Mambo_5
    #mamboAdd = "D0:3A:3A:2D:E6:36" # ViconObject = Mambo_3
    #mamboAdd = "D0:3A:03:39:E6:3B"  # ViconObject = Mambo_4
    wifi = False
    retries = 12

    # The mambo object
    mambo = Mambo("", use_wifi=wifi)

    # Checks if the drone is able to communicate with ROS
    success = False

    # Variables of the drone
    tko = land = False
    cannon = auto_tko = need_to_change_mode = need_to_toggle_mode = False
    p_mode = linX = linY = Alt = Hdg = 0

    mambo = Mambo(mamboAdd, use_wifi=wifi)
    success = mambo.connect(retries)
    if(success):
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)
        mambo.flat_trim()
        rate = rospy.Rate(100)
        land, tko = False, True

    return mambo, success 
def dovertical(z1):
    global tko, land
    desiredz = 1.3
    #pid = [0.5, 0.5, 0.5]
    # diff = z1 - z2 # this is also the error
    des_diff = z1 - desiredz
    #v = np.clip(pid[0]*diff, )
    if math.fabs(des_diff) > 0.3:
        if des_diff < 0:  # z2(remote) object is higher in evelvation
            ver = 35  # movedrone higher
        elif des_diff > 0:  # z1(autno) object is higher
            ver = -35  # movedrone lower
    else:
        ver = 0  # do nothing
    # if math.fabs(diff) > 0.7:
    #     tko = True
    #     print('Take off')
    # if z2 < 0.1:
    #     land = True
    #     print('Land')

    return ver
def cb_vicon_data1(data): # ViconObject = Mambo_1
    global x_1, y_1, z_1
    global rx1, ry1, rz1, rw1
    dt,dr=data.transform.translation, data.transform.rotation
    x_1,y_1,z_1 = dt.x, dt.y,dt.z
    rx1,ry1,rz1,rw1 = dr.x, dr.y, dr.z, dr.w
# def cb_vicon_data2(data): # ViconObject = Mambo_2
#     global x_2, y_2, z_2
#     dt,dr=data.transform.translation, data.transform.rotation
#     x_2,y_2,z_2 = dt.x, dt.y,dt.z
#     rx2,ry2,rz2,rw2 = dr.x, dr.y, dr.z, dr.w
def cb_vicon_data3(data): # ViconObject = Mambo_3
    global x_3, y_3, z_3
    global rx3, ry3, rz3, rw3
    dt,dr=data.transform.translation, data.transform.rotation
    x_3,y_3,z_3 = dt.x, dt.y,dt.z
    rx3,ry3,rz3,rw3 = dr.x, dr.y, dr.z, dr.w
def cb_vicon_data4(data): # ViconObject = Mambo_4
    global x_4, y_4, z_4
    global rx4, ry4, rz4, rw4
    dt,dr=data.transform.translation, data.transform.rotation
    x_4,y_4,z_4= dt.x, dt.y,dt.z
    rx4,ry4,rz4,rw4 = dr.x, dr.y, dr.z, dr.w
def cb_vicon_data5(data): # ViconObject = Mambo_5
    global x_5, y_5, z_5
    global rx5, ry5, rz5, rw5
    dt,dr=data.transform.translation, data.transform.rotation
    x_5,y_5,z_5= dt.x, dt.y,dt.z
    rx5,ry5,rz5,rw5 = dr.x, dr.y, dr.z, dr.w
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

    # print('Vicon Yaw =', round(yaw, 2))

    return roll, pitch, yaw
def dovertical(z1):
    global tko, land
    desiredz = 1.3
    #pid = [0.5, 0.5, 0.5]
    # diff = z1 - z2 # this is also the error
    des_diff = z1 - desiredz
    #v = np.clip(pid[0]*diff, )
    if math.fabs(des_diff) > 0.3:
        if des_diff < 0:  # z2(remote) object is higher in evelvation
            ver = 35  # movedrone higher
        elif des_diff > 0:  # z1(autno) object is higher
            ver = -35  # movedrone lower
    else:
        ver = 0  # do nothing
    # if math.fabs(diff) > 0.7:
    #     tko = True
    #     print('Take off')
    # if z2 < 0.1:
    #     land = True
    #     print('Land')

    return ver
def doyawangle(yaw, y1, y2, x1, x2):
    real_angle = turn_thresh = 0

    turn_angle = round((math.atan2((y1-y2), (x1-x2))*180/math.pi), 6)

    if turn_angle < 0:
        real_angle = math.fabs(turn_angle)
    elif turn_angle > 0:
        real_angle = -turn_angle

    real_angle = real_angle - 90

    if real_angle < -180:
        real_angle = 180 - math.fabs(real_angle + 180)

    
    turn_thresh = (real_angle - yaw)

    if turn_thresh < -180:
        turn_thresh = turn_thresh + 360
    elif turn_thresh > 180:
        turn_thresh = turn_thresh - 360
    if math.fabs(turn_thresh) > 30:  # and (math.fabs(turn_thresh) < 180):
        if turn_thresh > 0:
            y = 100
        elif turn_thresh < 0:
            y = -100
    else:
        y = 0
    prev_yaw = yaw
    return y, turn_thresh
def doforward(x1, x2, y1, y2, pgain):
    
    maxSD, minSD = 0.08, 0.04

    distance = math.sqrt(math.pow((x2-x1), 2)+math.sqrt(math.pow((y2-y1), 2)))

    if distance > maxSD:
        p = pgain*85
    elif distance > minSD:
        p = 0
    else: 
        p = 0

    return p, distance  # Distance is "error"


def checkvfence(x1,y1,z1, yaw,r,p,y,v,dur,mambo):
    global total_time
    r = 0
    if total_time > 2:
        inital_con = True
        total_time = 0
    else:
        inital_con = False
    #if (x1> 3.75) or (x1<-3.25) or (y1> 1.7) or (y1<-2.7) or (z1> 2.1):
    print(total_time)

    if (x1> 3.75) or (x1<-3.4) or (y1> 1.75) or (y1<-2.65) or (z1> 2.1):
        if inital_con == True:
            mambo.fly_direct(roll=(0), pitch=(0), yaw=(0), vertical_movement=(0), duration=0.1)
            inital_con = False
        dur= 0.03
        #print('x1 = ', x1 ,'y1 = ' ,y1 ,'z1 = ', z1)
        cor_x = 0-x1
        cor_y = 0-y1
        cor_z = 0-z1
        if yaw < 0:
            calc_angle = yaw+360
        else:
            calc_angle = yaw

        #YawAngle to Rectangular
        yaw_x = 0.4*math.cos((calc_angle*math.pi/180))
        yaw_y = 0.4*math.sin((calc_angle)*math.pi/180)
        yaw_z = z1

        magnit_cor =math.sqrt(math.pow(cor_x,2)+math.pow(cor_y,2)+math.pow(cor_z,2))
        magnit_yaw =math.sqrt(math.pow(yaw_x,2)+math.pow(yaw_y,2)+math.pow(yaw_z,2))
        cor_angle = cor_x*yaw_x+cor_y*yaw_y+cor_z*yaw_z

        theta=math.acos((cor_angle)/(magnit_cor*magnit_yaw))
        r = 50 * math.sin(theta) 
        p = 50 * math.cos(theta) 
        y = 0
        if z1 > 2.0:
            v = -50
        

    return r,p,y,v,dur
def swap(x1, y1, z1, prev1_x1, prev1_y1, prev1_z1, prev2_x1, prev2_y1, prev2_z1):
    
    prev3_x1, prev3_y1, prev3_z1 = prev2_x1, prev2_y1, prev2_z1   
    prev2_x1, prev2_y1, prev2_z1 = prev1_x1, prev1_y1, prev1_z1 
    prev1_x1, prev1_y1, prev1_z1 = x1, y1, z1

    return prev1_x1, prev1_y1, prev1_z1, prev2_x1, prev2_y1, prev2_z1, prev3_x1, prev3_y1, prev3_z1
def low_pass(filtered_x, filtered_y, filtered_z, prev1_x1, prev1_y1, prev1_z1, prev2_x1, prev2_y1, prev2_z1, prev3_x1, prev3_y1, prev3_z1):
    filtered_x = prev1_x1*0.05 + prev2_x1*0.1 + prev3_x1*0.15 + filtered_x * 0.7
    filtered_y = prev1_y1*0.05 + prev2_y1*0.1 + prev3_y1*0.15 + filtered_y * 0.7
    filtered_z = prev1_z1*0.05 + prev2_z1*0.1 + prev3_z1*0.15 + filtered_z * 0.7

    return filtered_x, filtered_y, filtered_z
def pitchcontroller(x1, y1, x2, y2,prev_dis):
    
    if (x2> 3.75) or (x2<-3.4) or (y2> 1.75) or (y2<-2.65):
        if x2 >3.75:
            x2 = 3.7
        if x2 <-3.4:
            x2 = -3.3
        if y2 >1.75:
            y2 = 1.7
        if y2 <-2.65:
            y2 = -2.6

    dist = math.sqrt(math.pow((x2-x1), 2)+math.sqrt(math.pow((y2-y1), 2)))
    
    pgain_pitch = (math.fabs(dist-prev_dis)*0.9 + dist*0.1) #somthing

    return pgain_pitch
def checkcondition_way(x1,y1,x2,y2):
    distance = math.sqrt(math.pow((x2-x1), 2)+math.sqrt(math.pow((y2-y1), 2)))
    if distance < 0.9:
        #print('condition met')
        return True
    else: 
        #print('condition not met')
        return False
# def do_waypoint(mambo):
    #     global w1_x, w1_y
    #     global w2_x, w2_y
    #     global w3_x, w3_y
    #     global w4_x, w4_y
    #     global w5_x, w5_y
    #     global desiredx, desiredy

    #     global current_objective_way

    #     print(current_objective_way)
    #     if current_objective_way == 1:
    #         desiredx, desiredy = w1_x, w1_y
    #         if checkcondition_way(x_5, y_5, w1_x, w1_y):
    #             current_objective_way = 2
    #             mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
    #             print('Goto waypoint 2')
    #     if current_objective_way == 2:
    #         desiredx, desiredy = w2_x, w2_y
    #         if checkcondition_way(x_5, y_5, w2_x, w2_y):
    #             current_objective_way = 3
    #             mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
    #             print('Goto waypoint 3')
    #     if current_objective_way == 3:
    #         desiredx, desiredy = w3_x, w3_y
    #         if checkcondition_way(x_5, y_5, w3_x, w3_y):
    #             current_objective_way = 4
    #             print('Goto waypoint 4')
    #             mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
    #     if current_objective_way == 4:
    #         desiredx, desiredy = w4_x, w4_y
    #         if checkcondition_way(x_5, y_5, w4_x, w4_y):
    #             current_objective_way = 5
    #             print('Goto waypoint 1')
    #             mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
    #     if current_objective_way == 5:
    #         desiredx, desiredy = w5_x, w5_y
    #         if checkcondition_way(x_5, y_5, w5_x, w5_y):
    #             current_objective_way = 1
    #             mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.7)
def getroll(x1,y1,x1_prev, y1_prev, yaw, z):
    cor_x = x1 - x1_prev
    cor_y = y1 - y1_prev
    if math.fabs(cor_x > 0.1) or math.fabs(cor_y) > 0.1:

        if yaw < 0:
            calc_angle = yaw+360
        else:
            calc_angle = yaw

            #YawAngle to Rectangular
        yaw_x = 0.4*math.cos((calc_angle*math.pi/180))
        yaw_y = 0.4*math.sin((calc_angle)*math.pi/180)

        magnit_cor =math.sqrt(math.pow(cor_x,2)+math.pow(cor_y,2))
        magnit_yaw =math.sqrt(math.pow(yaw_x,2)+math.pow(yaw_y,2))
        cor_angle = cor_x*yaw_x+cor_y*yaw_y

        theta=math.acos((cor_angle)/(magnit_cor*magnit_yaw))
        if math.fabs(theta) > 10:
            r = 15 * math.sin(theta)
        else:
            r = 0
        print(r)

        if z < 1:
            r = 0 
    else:
        r = 0
    return r
def vicon_sys():

#Declare Globals
    global x_1, y_1, z_1, rw1, rx1, ry1, rz1
    global x_3, y_3, z_3, rw3, rx3, ry3, rz3
    global x_4, y_4, z_4, rw4, rx4, ry4, rz4
    global x_5, y_5, z_5, rw5, rx5, ry5, rz5
    global desiredx, desiredy
    global tko, land
    global total_time
    
#Declare Waypoint Globals
    # global w1_x, w1_y, w2_x, w2_y, w3_x, w3_y, w4_x, w4_y, w5_x, w5_y
    # global current_objective_way
    # current_objective_way = 1 # start at way point 1
    # desiredx = desiredy = w1_x = w1_y = 0
    # w2_x, w2_y = 2.6, 0.75
    # w3_x, w3_y = 2.6, -1.6
    # w4_x, w4_y = -1.6, -1.6 
    # w5_x, w5_y = -1.6, 0.75 

#Init ROS node + Vicon
    rospy.init_node('vicon_reader', anonymous=True)
    rate = rospy.Rate(100)
    # Choose which topic to listen
    s1_vicon = rospy.Subscriber('/vicon/Mambo_1/Mambo_1', TransformStamped, cb_vicon_data1, buff_size=1)
    s3_vicon = rospy.Subscriber('/vicon/Mambo_3/Mambo_3', TransformStamped, cb_vicon_data3, buff_size=1)
    s4_vicon = rospy.Subscriber('/vicon/Mambo_4/Mambo_4', TransformStamped, cb_vicon_data4, buff_size=1)
    s5_vicon = rospy.Subscriber('/vicon/Mambo_5/Mambo_5', TransformStamped, cb_vicon_data5, buff_size=1)

    mambo,_ = mambo_init()
#Declare varibles, i think the global variablesafe_l are already initialized as 0
    x_5= y_5 = z_5 = 0
    prev1_x1 = prev1_y1 = prev1_z1 = prev2_x1 = prev2_y1 = prev2_z1 = prev3_x1 = prev3_y1 = prev3_z1 = filtered_x = filtered_y = filtered_z = 0 
    prev_yaw = 0
    rx5 = ry5 = rz5 = rw5 = 0

    x_2 = y_2 = 0
    x_3 = y_3 = z_3 = 0
    r = p = y = v = 0
    x5_prev = y5_prev = 0
    dur = 0.01
    total_time = 0
    pgain_pitch = prev_dis = 0
    timemark= time.time()
    tko = False
#MAIN LOOP
    while not rospy.is_shutdown():
        if keyboard.is_pressed('l'):
            print('Landing!')
            land = True
        if keyboard.is_pressed('t'):
            print('Takeoff!')
            tko = True

        rate.sleep()
        if tko == True:
            mambo.safe_takeoff(2)
            tko = False
        if land == True:
            mambo.safe_land(2)
            land = False

        roll, pitch, yaw5 = quaterion2euler(rx5, ry5, rz5, rw5)
        v = dovertical(z_5)

        desiredx = x_3
        desiredy = y_3

        y,yaw_diff = doyawangle(yaw5, y_5, desiredy, x_5, desiredx)
        pgain_pitch = pitchcontroller(x_5,y_5,desiredx, desiredy,prev_dis)

        # prev1_x1, prev1_y1, prev1_z1, prev2_x1, prev2_y1, prev2_z1, prev3_x1, prev3_y1, prev3_z1 = swap(x_1, y_1, z_1, prev1_x1, prev1_y1, prev1_z1, prev2_x1, prev2_y1, prev2_z1)
        # filtered_x, filtered_y, filtered_z = low_pass(filtered_x, filtered_y, filtered_z, prev1_x1, prev1_y1, prev1_z1, prev2_x1, prev2_y1, prev2_z1, prev3_x1, prev3_y1, prev3_z1)
        # r,p,y,v, dur  = checkvfence(filtered_x, filtered_y, filtered_z,yaw5,r,p,y,v, dur,mambo)
        # do_waypoint(mambo)

        if math.fabs(yaw_diff) < 45 and (z_5 > 0.7):
            p, prev_dis = doforward(x_5, desiredx, y_5, desiredy, pgain_pitch)
        else:
            p =0
        r = getroll(x_5,y_5, x5_prev, y5_prev, yaw5,z_5) #Input z to not use roll unless high enough
        mambo.fly_direct(roll=(r), pitch=(p), yaw=(y), vertical_movement=(v), duration=0.01)
        #print(x_2, y_2)
        dt = time.time() - timemark
        timemark = time.time()
        total_time = total_time + dt

        x5_prev, y5_prev = x_5, y_5

#Main function
if __name__ == '__main__':
    try:
        vicon_sys()
    except rospy.ROSInterruptException:
        print("Exiting...")
