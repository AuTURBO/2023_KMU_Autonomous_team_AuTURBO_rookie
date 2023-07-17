#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, random, math, time
import rospy, rospkg
import sys
import os
import signal
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

from cv_bridge import CvBridge
from Ultrasonic import Ultrasonic
from Laser import Laser
from Slidewindow import Line_Detection
from RLineDetection import RLineDetection
from Pidcal import Pidcal
from Stop_Detector import Stop_Detector
from Stop_Counter import Stop_Counter
from ObstacleDetector import ObstacleDetector

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

slidewindow = Line_Detection()
detect2 = RLineDetection()
pidcal = Pidcal()
ultra = Ultrasonic()
lid = Laser()
Obstacle_Detector = ObstacleDetector()
Stop_Counter = Stop_Counter()
Stop_Detector = Stop_Detector()
bridge = CvBridge()

# for parking
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
ar_yaw = 0
k = 10
angle_cal = 0.017
ar_id = 0

obstacles = None
lid_data = None
motor_pub = None

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def cal_img_callback(data):
    global cal_image
    cal_image = bridge.imgmsg_to_cv2(data, "bgr8")

def lid_callback(data):
    global lid_data
    lid_data = data.ranges

def obstacle_callback(data):
    global obstacles
    obstacles = data

def ultra_callback(msg):
    global ultra
    ultra.ultra_data(msg.data)

def drive(Angle, Speed):
    global motor_pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    motor_pub.publish(msg)

def warp(img):
    src = np.float32([
        [1000, 580],
        [-360, 580],
        [-560, 276],
        [1200, 276],
    ])
    dst = np.float32([
        [400, 480],
        [100, 480],
        [-650, 0],
        [1150, 0],
    ])

    W = cv2.getPerspectiveTransform(src, dst)

    image = cv2.warpPerspective(img, W, (640, 480), flags=cv2.INTER_LINEAR)

    return image

def image_process(img):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blur=cv2.GaussianBlur(gray, (5, 5), 0)
    blur_stop_detector = cv2.GaussianBlur(gray, (5, 5), 0)

    low_threshold = 60
    high_threshold = 70
    canny = cv2.Canny(blur, low_threshold, high_threshold)

    kernel = np.ones((5, 5), np.uint8)
    close = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

    warp_img = warp(close)

    ret, thres_img = cv2.threshold(warp_img, 150, 255, cv2.THRESH_BINARY)
    ret, thres_img_stop_detector =  cv2.threshold(blur_stop_detector, 150, 255, cv2.THRESH_BINARY)

    return thres_img, thres_img_stop_detector

# 0 1 2 3 4 5 6 7
# left x x x right b_right back b_left
def ar_callback(msg):
    global ar_id
    for i in msg.markers:
        ar_id = i.id
        pose = i.pose.pose
        arData["DX"] = pose.position.x
        arData["DY"] = pose.position.y
        arData["DZ"] = pose.position.z

        arData["AX"] = pose.orientation.x
        arData["AY"] = pose.orientation.y
        arData["AZ"] = pose.orientation.z
        arData["AW"] = pose.orientation.w

def id_back():
    return ar_id

def arparking():
    global k, angle_cal
    ar_yaw = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))[2]
    angle = k * np.degrees(arData["DX"] - angle_cal)

    return angle, ar_yaw, arData["DZ"]

def re_parking():
        global arData, k, angle_cal
        sec = time.time()
        #print("re_parking2")

        while time.time() - sec < 1.3 :

            #control coef

            if arData["DX"] != 0:
                #print("DX != 0")
                angle = -k * np.degrees(arData["DX"] - angle_cal)
            else:
                angle = 0

            if ultra.ultra_steer():
                print("reparking ultra detected")
                drive(0, 0)
                time.sleep(0.1)
                break
            else:
                drive(angle, -3)
                time.sleep(0.05)

        if arData["DZ"] < 0.235:
            #print("1")
            drive(0, 0)
            time.sleep(0.05)

        else :
            #print("2")
            # control coef

            speed = 3
            angle = k * np.degrees(arData["DX"] - angle_cal)
            drive(angle, speed)
            time.sleep(0.05)

MODE = 0

def start():
    global motor_pub
    global image, cal_image
    global count, check
    global angle_cal
    global MODE
    global flag

    rospy.init_node('auto_drive')
    lid_sub = rospy.Subscriber('/scan', LaserScan, lid_callback, queue_size = 1)
    obstacle_sub = rospy.Subscriber('/obstacles', Obstacles, obstacle_callback, queue_size = 1)
    motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    cal_image_sub = rospy.Subscriber("/usb_cam/image_rect", Image, cal_img_callback)
    ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback, queue_size = 1)
    ultra_sub = rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback, queue_size = 1)

    flag = 0
    count = 0
    check = 0
    angle = 0
    first_detect = 0
    a = 0
    b = 0

    rospy.sleep(2)

    while not rospy.is_shutdown():

        while not image.size == (640*480*3):
            continue

        img_process, img_process_stop_detector = image_process(image)
        cal_img_process, trash = image_process(cal_image)
        line_detected, lpos, rpos, cpos_independent, cpos_dependent, line_flag = slidewindow.line_detect(img_process, a, b)

        cv2.imshow('result', line_detected)
        print('lpos: ', lpos)
        print('rpos: ', rpos)
        # for parking mode
        RGB_img, r = detect2.roi_detect(cal_img_process)
        park_rpos = detect2.draw_line(RGB_img, r)

        # for obstacle
        ob_mode = Obstacle_Detector.mode_on(obstacles)
        POS = Obstacle_Detector.check(obstacles)

        # for parking
        ar_id = id_back()
        roi_data = lid.roi(lid_data)

        #print("MODE = ", MODE)
        print("flag = ", line_flag)


        #angle = pidcal.get_pid(car_loc)
        if MODE == 0:

            # Straight & Can see both lanes
            if line_flag == 1 :
                #pidcal.twiddle(255)
                angle = pidcal.get_pid(cpos_dependent, 267)
                #if abs(pidcal.get_pid(car_loc, 255)) < 10:
                #    angle = 0 # angle = -x_current

            # Lane Left
            elif line_flag == 2 :
                angle = pidcal.get_pid(cpos_dependent, 267)

            # Lane Right
            elif line_flag == 3 :
                angle = pidcal.get_pid(cpos_dependent, 267)

            # See neither lanes == flag == None
            else:
                # print('pre_setpoint =', pidcal.get_pre_setpoint())
                angle = pidcal.get_pid(cpos_dependent, pidcal.get_pre_setpoint())
            #print ("angle:", angle)

            drive(angle, 7)
# center : 128/412
# left : 173
# right : 335
        if MODE == 2 and first_detect == 0:
            first_detect = POS.value

        if MODE == 2:
            if POS.value == 2: # Right Obstacle
                if count == 2:
                    count = 3
                elif count == 3:
                    pass
                else:
                    count = 1
                flag = 1
                a = 0
                b = 0
                angle = (lpos - 173) // 2
                drive(angle, 7)
                print("right detect angle = ,", angle)

            elif POS.value == 1: # Left Obstacle
                if count == 1:
                    count = 2
                else:
                    pass
                flag = 2
                a = 0
                b = 0
                angle = (rpos - 355) // 2
                drive(angle, 7)
                print("left detect angle = ", angle)

            elif POS.value == 3: # else
                if flag == 0:
                    a = 0
                    b = 0
                    angle = (cpos_independent - 255)
                    #print("range out of y")
                elif flag == 1:
                    print("no right")
                    angle = (lpos - 128) // 2
                    a = 2 * angle
                    b = 0
                else:
                    print("no left")
                    angle = (rpos - 432) // 2
                    a = 0
                    b = 2 * angle
                drive(angle, 7)


        #print("first_detect =", first_detect)
        #print("ob_mode.value: ", ob_mode.value)
        #print("\n")
        sec = time.time()

        if MODE == 2 and first_detect == 1 and ob_mode.value == 3 and count == 3:
            while time.time() - sec < 1:
                drive(-20, 7)
                time.sleep(0.05)

            MODE = 0 # auto
            print("LEFT START OBSTACLE DONE")

        elif MODE == 2 and first_detect == 2 and ob_mode.value == 3 and count == 3:
            while time.time() - sec < 1:
                drive(20, 7)
                time.sleep(0.05)

            MODE = 0 # auto
            print("RIGHT START OBSTACLE DONE")

        #CrossWalk
        if MODE != 4 and Stop_Detector.mask(img_process_stop_detector) and MODE != 2:
            print("CROSS WALK DETECT")
            drive(0,0)
            time.sleep(5.75)
            for t in range(20):
                drive(0, 7)
                time.sleep(0.1)
            MODE = 0
            count = 0

        # Stop Line(Yellow)
        detected = Stop_Counter.check_stop_line(image)

        if detected :
            if MODE == 0 and Stop_Counter.cnt == 3:
                MODE = 4
                print("PARKING MODE")

        if MODE == 4 :
            cv2.imshow("thres_img", RGB_img)
            angry = (park_rpos - 330) // 2
            drive(angry, 5)

            if roi_data >= 20 and check == 0:
                sec = time.time()
                while time.time() - sec < 2:
                    print("go straight for 0.5 seconds")
                    drive(angry, 3)
                check += 1

            elif roi_data >= 20 and check == 1:
                while True:
                    sec = time.time()
                    while time.time() - sec < 1.3:
                        drive(-20, 5)
                        time.sleep(0.05)

                    sec = time.time()
                    while time.time() - sec < 1.65:
                        drive(35, -5)
                        time.sleep(0.05)

                    sec = time.time()
                    while time.time() - sec < 4.5:
                        drive(-30, -3)
                        time.sleep(0.05)
                        if ultra.ultra_steer():
                            print("obstacle detect")
                            drive(0, 0)
                            time.sleep(0.05)
                            break
                        else:
                            pass

                    check += 1
                    #print("check = ", check)

                    break


            elif ar_id == 2 and check == 2:
                angle, yaw, DZ = arparking()

                #print("yaw :", yaw)
                #print("DX :", arData["DX"])
                #print("angle :", angle2)

                if DZ < 0.235:
                    if abs(arData["DX"]) > angle_cal:
                        print("re_parking1")
                        re_parking()
                    else:
                        print("finish")
                        drive(0, 0)
                        break
                else:
                    print("go")
                    drive(angle, 3)
                    time.sleep(0.05)


            else:
                drive(angry, 5)

        if MODE == 0 and ob_mode.value == 0:
            MODE = 2

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':

    start()
