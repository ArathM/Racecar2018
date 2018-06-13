#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 30 14:34:16 2018

@author: racecar
"""

import rospy as rp
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from math import sqrt


pub = rp.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

count = 1
z = float(input("Choose the speed "))

global integral

integral = []

def callback(stuff):
    global count
    if count == 1:
       global x
       global y
       input1 = int(input("Choose the starting angle to take into account in degrees "))
       input2 = int(input("Choose the ending angle to take into account in degrees "))
       x = int(((input1*np.pi)/180)/stuff.angle_increment)
       y = int(((input2*np.pi)/180)/stuff.angle_increment)
    ##choose 80
    global z
    msg = AckermannDriveStamped()
    msg.drive.speed = z
    pub.publish(msg)
    count = 0
    distance = .8
    space = sum(stuff.ranges[x:y])/(y-x)
    error = space - distance 
    integral.append(error)
    u = -error
    a = stuff.ranges[135]
    b = 10
    c = sqrt(a**2+b**2)
    theta = np.arccos((a**2+b**2-c**2)/(2*a*b))
    if sum(integral) < 1800:
       nice = sum(integral) 
    print nice
    if error < 0:
       msg.drive.steering_angle = msg.drive.steering_angle-(1/10)*theta+((u/z)*np.absolute(error))-(1/50)*nice
       pub.publish(msg)
    if error > 0:
       msg.drive.steering_angle = msg.drive.steering_angle-((1/10)*theta*np.absolute(error))+((u/z)*np.absolute(error))-((1/50)*nice*np.absolute(error))
       pub.publish(msg)

def Follow():
    rp.init_node("Wall_follow", anonymous = True)
    rp.Subscriber("/scan", LaserScan, callback)  

if __name__ == '__main__':
    Follow()
    rp.spin() 

