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


pub = rp.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

count = 1

def callback(stuff):
    global count
    if count == 1:
       global x
       global y
       input1 = int(input("Choose the starting angle to take into account in degrees "))
       input2 = int(input("Choose the ending angle to take into account in degrees "))
       x = int(((input1*np.pi)/180)/stuff.angle_increment)
       y = int(((input2*np.pi)/180)/stuff.angle_increment)
    count = 0
    safe = 0.5
    safespace = sum(stuff.ranges[x:y])/(y-x)
    print safespace
    print stuff.ranges[x:y]
    if safespace < safe:
        msg = AckermannDriveStamped()
        msg.drive.speed = 0
        msg.drive.acceleration = 0
        pub.publish(msg)

def Safe():
    rp.init_node("Safety", anonymous = True)
    rp.Subscriber("/scan", LaserScan, callback)    

if __name__ == '__main__':
    Safe()
    rp.spin() 


