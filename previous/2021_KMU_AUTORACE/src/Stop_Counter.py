#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg
import numpy as np
import cv2, time
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

image = np.empty(shape=[0])
bridge = CvBridge()

pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(Angle, Speed):
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

class Stop_Counter:
    def __init__(self):
        self.cnt = 0
        self.previous_time = time.time() + 5
        self.lower_yellow = (20, 100, 100)
        self.upper_yellow = (40, 255, 255)

    def check_stop_line(self, img):
        if time.time() < self.previous_time + 10 :
            return False

        img = img[ 370:410, 140:550 ] #240
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, self.lower_yellow, self.upper_yellow)

        # IMSHOW FOR DEBUG
        cv2.putText(img_mask, 'NONZERO %d'%np.count_nonzero(img_mask), (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.imshow('img_mask', img_mask)

        #print("stop line : ", np.count_nonzero(img_mask))
        if np.count_nonzero(img_mask) > 1350:
            self.on_detected()
            return True

        return False

    def on_detected(self):
        print('STOP LINE DETECTED!!!')
        print('cnt =', self.cnt)
        self.previous_time = time.time()
        self.cnt += 1

stop_counter = Stop_Counter()

def start():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('stop_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    rospy.sleep(2)

    while True:
        while not image.size == (640*480*3):
            continue

        stop_counter.check_stop_line(image)

        cv2.imshow('frame', image)
        drive(0,5)

        if cv2.waitKey(1) == ord('q'):
                break

    rospy.spin()

if __name__ == '__main__':
    start()
