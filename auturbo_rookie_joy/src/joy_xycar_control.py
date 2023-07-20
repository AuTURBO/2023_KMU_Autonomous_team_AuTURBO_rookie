#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from geometry_msgs.msg import Twist
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Joy

ack_msg = xycar_motor()
ack_publisher = None

def map(x,input_min,input_max,output_min,output_max):
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

def callback_joy(msg):
    global ack_msg
    print(f"axes: {msg.axes}")
    print(f"buttons: {msg.buttons}")
    print(" ")

    acceleration = map(msg.axes[4], 1.0, -1.0, 0, 50)
    deacceleration = map(msg.axes[5], 1.0, -1.0, 0, -50)
    print(f"acceleration: {acceleration}")

    angle = map(msg.axes[0], 1.0, -1.0, -50, 50)
    speed = acceleration + deacceleration

    ack_msg.speed = int(speed)
    ack_msg.angle = int((angle))
    
    #ack_msg.speed = int(msg_android_speed.linear.x*50)
    #ack_msg.angle = int((-msg_android_steering.angular.z)*50)

rospy.init_node("joy_xycar_control")
rospy.Subscriber("joy",Joy, callback_joy)
ack_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

while not rospy.is_shutdown():
    ack_publisher.publish(ack_msg)
    time.sleep(0.01)
