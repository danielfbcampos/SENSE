#!/usr/bin/env python

# Based on (c) 2012, Tang Tiong Yew

import rospy
import serial
import string
import math
import sys
import re

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from math import atan2, sqrt, atan 

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("razor_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsg = Imu()

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]


accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
degrees2rad = math.pi/180.0


default_port='/dev/ttyUSB0'
port = rospy.get_param('~port', default_port)

#read calibration parameters
port = rospy.get_param('~port', default_port)


imu_frame='base_imu_link'
imu_frame = rospy.get_param('~imu_frame', imu_frame)

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)




while not rospy.is_shutdown():
    line = ser.readline()
    #print("\n")
    #print(line)
    line=re.sub(' +',' ',line)
    line = line.replace("Ax=","")
    line = line.replace("Ay=","")
    line = line.replace("Az=","")
    line = line.replace("Gx=","")
    line = line.replace("Gy=","")
    line = line.replace("Gz=","")
    line = line.replace(" | "," ")
    data = string.split(line," ")    # Fields split
  #  words1 = string.split(words[1]," ")    # Fields split
    if len(data) > 7:
	yaw_deg=-float(data[11])
 	if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        yaw = yaw_deg*degrees2rad
       # yaw_deg = yaw_deg + imu_yaw_calibration

	imuMsg.linear_acceleration.x = -float(data[1])* accel_factor
        imuMsg.linear_acceleration.y = float(data[2])* accel_factor
        imuMsg.linear_acceleration.z = float(data[3])* accel_factor
	imuMsg.angular_velocity.x = float(data[4])*degrees2rad
        imuMsg.angular_velocity.y =  -float(data[5])*degrees2rad
        imuMsg.angular_velocity.z = -float(data[6])*degrees2rad
 	
   	pitch =  atan (imuMsg.linear_acceleration.x/sqrt(imuMsg.linear_acceleration.y*imuMsg.linear_acceleration.y + imuMsg.linear_acceleration.z*imuMsg.linear_acceleration.z))
	roll = atan (imuMsg.linear_acceleration.y/sqrt(imuMsg.linear_acceleration.x*imuMsg.linear_acceleration.x + imuMsg.linear_acceleration.z*imuMsg.linear_acceleration.z))

	#print(roll)
	#print(pitch)
	#print(yaw)

 	q = quaternion_from_euler(roll,pitch,yaw)
   	imuMsg.orientation.x = q[0]
    	imuMsg.orientation.y = q[1]
    	imuMsg.orientation.z = q[2]
    	imuMsg.orientation.w = q[3]
	

    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = imu_frame
    pub.publish(imuMsg)           
ser.close
#f.close
