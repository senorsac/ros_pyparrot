#!/usr/bin/env python3.6
# Written by Isaac Vargas and Alan Garduno
import rospy
import math
import time
import keyboard
from std_msgs.msg import Empty, String, UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from pyparrot.Minidrone import Mambo

import _thread

#Variable that will store the parameters of the mambo drone
#mamboAdd = "D0:3A:EE:30:E6:20"
#mamboAdd = "E0:14:A0:AE:3D:C7"
mamboAdd = "D0:3A:3A:2D:E6:36"
wifi = False
retries = 12

#The mambo object
mambo = Mambo("",use_wifi=wifi)

#Checks if the drone is able to communicate with ROS
success = False

#Variables of the drone
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

#Preffered piloting mode function
def pilotmode(Mambo,s_mode):
    if s_mode.data==0:
        mode = 'easy'
    elif s_mode.data==1:
        mode = 'medium'
    elif s_mode.data==2:
        mode = 'difficult'
    else:
        return

    #command_tuple = Mambo.command_parser.get_command_tuple("minidrone", "PilotingSettings","PreferredPilotingMode")
    (command_tuple, enum_tuple) = Mambo.command_parser.get_command_tuple_with_enum("minidrone", "PilotingSettings","PreferredPilotingMode",mode)

    #return Mambo.drone_connection.send_enum_command_packet_ack(command_tuple,s_mode.data);
    return Mambo.drone_connection.send_enum_command_packet_ack(command_tuple,enum_tuple);

def togglemode(Mambo):
    command_tuple = Mambo.command_parser.get_command_tuple("minidrone", "Piloting","TogglePilotingMode")
    return Mambo.drone_connection.send_noparam_command_packet_ack(command_tuple)

#Set preffered piloting mode
def cb_pilotmode(p):
    global p_mode
    global need_to_change_mode
    p_mode = p
    need_to_change_mode = True

#Toggle flight mode
def cb_toggle_mode(data):
    global need_to_toggle_mode
    rospy.loginfo("\n" + rospy.get_name() + " Toggling Mode...\n")
    need_to_toggle_mode = True

#Sends the spin function to another thread
def spin_th(name,envi):
    rospy.spin()

#Callback of the land command
def cb_land(data):
    global land
    rospy.loginfo("\n" + rospy.get_name() + " Land!!\n")
    land = True

#Callback of the take-off command
def cb_take_off(data):
    global tko
    rospy.loginfo("\n" + rospy.get_name() + " Take-Off!!\n")
    tko = True

#CAllback of the velocities
def cb_cmd_vel(data):
    global linX
    global linY
    global Alt
    global Hdg
    rospy.loginfo(rospy.get_name() + "\n Linear_X: %s Linear_Y: %s Linear_Z: %s Angular_Z: %s",data.linear.x,data.linear.y,data.linear.z,data.angular.z)
    linX = data.linear.x
    linY = data.linear.y
    Alt = data.linear.z
    Hdg = data.angular.z

    #Forced perturbation
#    linX += 0.3

#    if(linX < 0):
#	    linX -= 0.1
#    if(linY < 0):
#	    linX += 0.18

def cb_shoot_cannon(data):
    global cannon
    rospy.loginfo("\n Cannon activated \n")
    cannon = True

def cb_auto_take_off(data):
    global auto_tko
    rospy.loginfo("\n Auto TKO activated \n")
    auto_tko = True

def sat(value, max_value):
    if(value > max_value):
	    value = max_value
    if(value < -max_value):
	    value = -max_value
    return value

def turn_degreees(mambo, degrees):
        """
        Turn the mambo the specified number of degrees (-180, 180).  Degrees must be an integere
        so it is cast to an integer here.  If you send it a float, it will be rounded according to
        the rules of int()
        This is called cap in the xml but it means degrees per
        http://forum.developer.parrot.com/t/what-does-cap-stand-for/6213/2
        :param degrees: degrees to turn (-180 to 180)
        :return: True if the command was sent and False otherwise
        """
        degrees = int(degrees)
        if (degrees > 180):
            degrees = 180
            print("Degrees too large: setting to 180")
        elif (degrees < -180):
            degrees = -180
            print("Degrees too large and negative: setting to -180")

        command_tuple = mambo.command_parser.get_command_tuple("minidrone", "Animations", "Cap")
        return mambo.drone_connection.send_turn_command(command_tuple, degrees)

#Initialization function
def init():
    global tko
    global land
    global cannon
    global auto_tko
    global linX
    global linY
    global Alt
    global Hdg
    global p_mode
    global need_to_change_mode
    global need_to_toggle_mode
    global okay_signal
    global tot_time
    global turn_indicator
    global keywaspressed
    if keyboard.is_pressed('t'):
        print('TakingOff')
    rospy.init_node('mambo_node',anonymous=True)
    mamboAdd = rospy.get_param('~bt',str("D0:3A:3A:2D:E6:36"))
    wifi = rospy.get_param('~mambo_wifi',False)
    retries = rospy.get_param('~mambo_retries',3)
    rospy.loginfo("\n" + rospy.get_name() + "\nParameters:\n" + mamboAdd + "\n" + str(wifi) + "\n" + str(retries) +"\n")
    s_cmd_vel = rospy.Subscriber('cmd_vel',Twist,cb_cmd_vel)
    s_take_off = rospy.Subscriber('take_off',Empty,cb_take_off)
    s_land = rospy.Subscriber('land',Empty,cb_land)
    s_cannon = rospy.Subscriber('cannon',Empty,cb_shoot_cannon)
    s_auto_tko = rospy.Subscriber('auto_tko',Empty,cb_auto_take_off)
    s_piloting_mode = rospy.Subscriber('piloting_mode',UInt8,cb_pilotmode)
    s_toggle_mode = rospy.Subscriber('toggle_mode',Empty,cb_toggle_mode)
    p_ready = rospy.Publisher('ready',String,queue_size=30)
    mambo = Mambo(mamboAdd,use_wifi=wifi)
    success = mambo.connect(retries)
    okay_signal = False
    keywaspressed = False
    turn_indicator = 0
    if(success):
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)
        mambo.flat_trim()
        rate = rospy.Rate(100)
        marktime = time.time()
        dt = 0
        tot_time = 0
        while not rospy.is_shutdown():

            if tko == True:
                mambo.safe_takeoff(2)
                p_ready.publish("ok")
                tko = False

            if land == True:
                mambo.safe_land(2)
                land = False

            if cannon == True:
                mambo.fire_gun()
                cannon = False

            if auto_tko == True:
                mambo.turn_on_auto_takeoff()
                auto_tko = False

            if need_to_change_mode == True:
                if pilotmode(mambo,p_mode):
                    print("Changed mode successfully")
                else:
                    print("Failed changing mode")
                need_to_change_mode=False

            if need_to_toggle_mode == True:
                if togglemode(mambo):
                    print("Activating preffered mode...")
                else:
                    print("Failed activating preferred mode")
                need_to_toggle_mode = False
            
            obtain_keyboard()
            r_y = round(linY,2)
            p_x = round(linX,2)
            v_z = round(Alt,2)
            y_z = round(Hdg,2)

            r_y = sat(r_y, 0.98)
            p_x = sat(p_x, 0.98)
            v_z = sat(v_z, 0.98)
            y_z = sat(y_z, 0.98)
            mambo.fly_direct(roll = (-r_y * 25), pitch = (p_x*30),yaw = (-y_z *75), vertical_movement = (v_z*75), duration=0.01)
            if keywaspressed == True:
                mambo.turn_degrees(turn_indicator)
                print('the shit should be turning-----------------------')
                keywaspressed = False

            print(keywaspressed, turn_indicator)
           # linY = 0
           # linX = 0
           # Hdg = 0
           # Alt = 0
            tot_time = dt + tot_time
            if tot_time > 0.2:
                tot_time = 0
                okay_signal = True
            print(tot_time, okay_signal)
            rate.sleep()
            dt = time.time() - marktime
            marktime = time.time()


        #mambo.disconnect()

def obtain_keyboard():
    global linX
    global linY
    global Alt
    global Hdg
    global tko
    global land
    global okay_signal
    global tot_time
    global turn_indicator
    global keywaspressed

    if keyboard.is_pressed('t'):
        print('TakingOff')
        tko = True
    if keyboard.is_pressed('w'):
        print('Moving Forward')
        linX = rampfunction(linX, 50)
        time.sleep(0.05)
    elif keyboard.is_pressed('s'):
        print('Moving Backward')
        linX = rampfunction(linX, -50)
        time.sleep(0.05)
    else:
        linX = rampfunction(linX, 0)


    if keyboard.is_pressed('a'):

        print('Moving Left')
        linY = rampfunction(linY, 50)
        time.sleep(0.05)
    elif keyboard.is_pressed('d'):
        print('Moving Right')
        linY = rampfunction(linY, -50)
        time.sleep(0.05)
    else:
        linY = rampfunction(linY, 0)

    if okay_signal == True:
        if keyboard.is_pressed('m'):
            print('Turning "90"')
            #mambo.turn_degrees(90)
            turn_indicator = 90
            tot_time = 0
            okay_signal = False
            keywaspressed = True
        if keyboard.is_pressed('n'):
            print('Turning "-90"')
            okay_signal = False
            turn_indicator = -90
            #mambo.turn_degrees(-90)
            tot_time = 0
            keywaspressed = True
        if keyboard.is_pressed('o'):
            print('Turning "180"')
            turn_indicator = 180
            #mambo.turn_degrees(180)
            tot_time = 0
            okay_signal = False
            keywaspressed = True
        if keyboard.is_pressed('p'):
            print('Turning "-180"')
            turn_indicator = -180
            #mambo.turn_degrees(-180)
            tot_time = 0
            okay_signal = False
            keywaspressed = True
        if keyboard.is_pressed('q'):
            print('Turning "0"')
            #mambo.turn_degrees(0)
            turn_indicator = 0
            tot_time = 0
            okay_signal = False
            keywaspressed = True

        

    if keyboard.is_pressed('up arrow'):
        print('Veritcal Up')
        Alt = rampfunction(Alt, 50)
        time.sleep(0.05)
        
    elif keyboard.is_pressed('down arrow'):
        print('Vertical Down')
        Alt = rampfunction(Alt, -50)
    else:
        Alt = rampfunction(Alt, 0)
        
    if keyboard.is_pressed('left arrow'):
        print('Rotating Left')
        Hdg = 2
        time.sleep(0.05)
    elif keyboard.is_pressed('right arrow'):
        print('Rotating Right')
        Hdg = -2
        time.sleep(0.05)
    else:
        Hdg = 0

    if keyboard.is_pressed('l'):
        print('Landing!')
        land = True
    return linY, linX, Alt, Hdg 

def rampfunction(prev_state, proportion): #proportion is desired state (from 0 to 50)
    
    if prev_state > 50:
        proportion = 50
        return proportion #proportion is new state if no changes
    if prev_state < -50:
        proportion = -50
        return proportion
    
    if (prev_state > proportion):
        new_state = prev_state - 25
        if new_state <= -50:
            new_state = -50
        return new_state
            
    if (proportion > prev_state):
        new_state = prev_state + 25
        if new_state >= 50:
            new_state = 50
        return new_state
    return prev_state

#Main function
if __name__=='__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        mambo.disconnect()