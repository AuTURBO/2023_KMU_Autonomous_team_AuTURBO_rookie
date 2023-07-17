#!/usr/bin/env python
#-*- coding: utf-8 -*-

####################################################################
# 프로그램명 : hough_drive_a2.py
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 08월 12일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from obstacle_detector.msg import Obstacles
from ObstacleDetector import ObstacleDetector, Position
from matplotlib import pyplot as plt

import sys
import os
import signal
import time
from datetime import datetime  # for record

# from Crosswalk_Counter import Crosswalk_Counter
from Stop_Counter_Test import Stop_Counter
from CurveDetector import CurveDetector
from Pidcal import Pidcal
from SlidingWindow2 import SlidingWindow


def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

# rgb image
image = np.empty(shape=[0])
bridge = CvBridge()
# image width
Width = 640
# image height
Height = 480

x_location_list = list()

pub = None
x_location_old = 400

car_run_speed = 0.6
curve_count = 0
PART = 1

now = datetime.now()  # for record

# pid class
pidcal = Pidcal()
# sliding window class
slidingwindow = SlidingWindow()

obstacles = None

cw_list = [0.0 for i in range(4)]


def obstacle_callback(data):
    global obstacles
    obstacles = data


def img_callback(data):
    global image
    # 1280 x 720
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    # resize 640 x 480
    image = cv2.resize(image, (640, 480))


# publish xycar_motor msg
def drive(Angle, Speed):
    global pub
    global car_run_speed
    global curve_count
    global PART
    
    
    if PART == 1:
        # ?? < Angle < ?? 
        if (Angle > 0.1 or Angle < -0.1) and car_run_speed >= 0.6:
            car_run_speed -= 0.02
        elif car_run_speed <= 0.8:
            car_run_speed += 0.005 * 1.5 #0.005 * 2
    elif PART == 2:
        if car_run_speed > 0.4:
            car_run_speed -= 0.005 * 8
    elif PART == 3:
	car_run_speed = 0.4
    elif PART == 4:
        if car_run_speed <= 0.5:
            # increase the constant value
            car_run_speed += 0.005
        

    msg = xycar_motor()
    msg.angle = Angle
    #msg.angle = 0
    msg.speed = car_run_speed
    #msg.speed = 0.4


    pub.publish(msg)


def region_of_interest(image, image_copy, width, height1, height2, roiw, roih):
    src = np.array([[width / 2 - 320, height1],
                    [width / 2 + 320, height1],
                    [width / 2 + 360, height2],
                    [width / 2 - 360, height2]], np.float32)

    for i in src:
        cv2.circle(image_copy, (i[0], i[1]), 5, (0,0,255),-1)

    dst = np.array([[0, 0], [roiw, 0], [roiw, roih], [0, roih]], np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, M, (roiw, roih))


# show image and return lpos, rpos
def process_image(frame):
    global Width

    # gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
    blur_gray = cv2.GaussianBlur(blur_gray, (kernel_size, kernel_size), 0)
    # canny edge
    # low_threshold = 45
    # high_threshold = 100
    low_threshold = 60 # 2 floor : 35  # 41
    high_threshold = 70  # 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    return edge_img


def start():
    global pub
    global image
    global cap
    global Width, Height
    global car_run_speed
    global cw_list
    global x_location_old
    global curve_count
    global PART

    # Node Name : auto_drive
    #rospy.init_node('auto_drive')

    # motor publisher : pub
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    # usb_cam subscriber : image_sub
    # callback : img_callback
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    # obstacle_detector subscriber : obstacle_sub
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)

    print
    "---------- Xycar A2 v1.0 ----------"
    #rospy.sleep(2)
    cap = cv2.VideoCapture("0925/1.avi")

    # original image video writer
    # image height
    #h = image.shape[0]
    # image width
    #w = image.shape[1]

    x_old = 320

    setPoint = 320
    cw_cnt = 0

    PART = 1

    # crosswalk_counter = Crosswalk_Counter()
    stop_counter = Stop_Counter()
    curve_detector = CurveDetector()

    ob = ObstacleDetector()
    # last_mode = ob.mode
    obstacle_cnt = 0
    crosswalk_cnt = 0
    pid_list = list()

    while True:
        ret, frame = cap.read()
        # Folder : Video/9-21
	image = cv2.resize(frame, (640,480))
        image_copy = image.copy()



        # preprocess the rgb image
        edge_img = process_image(frame)
        warp = region_of_interest(edge_img, image_copy, 640, 390, 420, 800, 448)

        # Bird Eyes View Image
        

        # cv2.imshow("src", src)
        cv2.imshow('origin:', image)
        #cv2.imshow('image', edge_img)
        cv2.imshow('warp', warp)
        cv2.imshow('canny_img', edge_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #plt.show()
            break
        elif cv2.waitKey(-1): pass

    #rospy.spin()


if __name__ == '__main__':
    start()
