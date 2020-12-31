#!/usr/bin/env python

import sys
import rospy
import serial
import string
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


rospy.init_node("rc")

default_port = rospy.get_param('~port', '/dev/rcnano')


pub_cmd_vel_rc  = rospy.get_param('~topic_cmd_vel_rc', "/cmd_vel_rc")
pub_cmd_safety_stop = rospy.get_param('~topic_cmd_safety_stop', "/safety_stop")

calib_file  = rospy.get_param('~calib_file', "/home/odroid/calib.txt")

VEL_LIN_MAX = rospy.get_param('~vel_lin_max', 1.0)
VEL_LIN_MIN = rospy.get_param('~vel_lin_min', -1.0)

VEL_ANG_MAX = rospy.get_param('~vel_ang_max', 0.4)
VEL_ANG_MIN = rospy.get_param('~vel_ang_min', -0.4)

velocity_publisher = rospy.Publisher(pub_cmd_vel_rc, Twist, queue_size=10)
safety_publisher = rospy.Publisher(pub_cmd_safety_stop, Bool, queue_size=10)

vel_msg = Twist()
safety_msg = Bool()

try:
    ser = serial.Serial(port=default_port, baudrate=9600, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("RC not found at port "+default_port + ". Did you specify the correct port in the file?")
    #exit
    sys.exit(0)


#rate = rospy.Rate(100);
calib = "1500,1500,1500,1500"

try:
    f = open(calib_file, "r")
    calib = f.readline()
    f.close()
except IOError:
    print('File does not exist')
    
data_calib = string.split(calib,",") 

ang_min = data_calib[0]
ang_max = data_calib[1]
lin_x_min = data_calib[2]
lin_x_max = data_calib[3] 

string_send = ang_min + "," + ang_max + "," + lin_x_min + "," + lin_x_max 

#ser.write(string_send)

while not rospy.is_shutdown():
    line = ser.readline()
    data = string.split(line,",")    # Fields split

    if(len(data) == 2):
        if(data[0] == "RC" and int(data[1])==0):
            ser.write(string_send.encode())
        continue
    elif(len(data) == 5):
        if(data[0] == "C"):
            ang_min = data[1]
            ang_max = data[2]
            lin_x_min = data[3]
            lin_x_max = data[4] 
            f = open(calib_file, "w")
            f.write(ang_min + "," + ang_max + "," + lin_x_min + "," + lin_x_max)
            f.close()
        continue

    elif (len(data) == 4):
        if (float(data[1])==0):
            lin_x = 0;
        elif (float(data[1])>1900):
            lin_x = VEL_LIN_MAX;
        elif (float(data[1])<1100): 
            lin_x = VEL_LIN_MIN;
        elif (float(data[1])<1450 and float(data[1])>=1100): 
            lin_x = (float(data[1])*VEL_LIN_MAX)/350.0 - 4.14*VEL_LIN_MAX
        elif (float(data[1])>1550 and float(data[1])<=1900): 
            lin_x = (float(data[1])*VEL_LIN_MAX)/350.0 - 4.43*VEL_LIN_MAX
        else:
            lin_x = 0;

        if (float(data[0])==0):
            ang_z = 0;
        elif (float(data[0])>1900):
            ang_z = VEL_ANG_MAX;
        elif (float(data[0])<1100): 
            ang_z = VEL_ANG_MIN;
        elif (float(data[0])<1450 and float(data[0])>=1100): 
            ang_z = (float(data[0])*VEL_ANG_MAX)/350.0 - 4.14*VEL_ANG_MAX
        elif (float(data[0])>1550 and float(data[0])<=1900): 
            ang_z = (float(data[0])*VEL_ANG_MAX)/350.0 - 4.43*VEL_ANG_MAX
        else:
            ang_z = 0;
	
        vel_msg.linear.x = lin_x
        if (float(data[3])<=1500):
            vel_msg.linear.z = 0
        else:
            vel_msg.linear.z = 1

        vel_msg.linear.y = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = ang_z

        if int(data[2])==0:
            safety_publisher.publish(Bool(False))    
        else:
            safety_publisher.publish(Bool(True))

    else:
        vel_msg.linear.y = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        vel_msg.linear.x = 0
        vel_msg.linear.z = 0
    velocity_publisher.publish(vel_msg)

#        rate.sleep()
ser.close

