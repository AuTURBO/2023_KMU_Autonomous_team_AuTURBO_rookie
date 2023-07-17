#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
from datetime import datetime # for record

#from Crosswalk_Counter import Crosswalk_Counter
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

pub = None

curve_sum_list = list()

car_run_speed = 1.0

now = datetime.now() # for record

cw_list = [0.0 for i in range(4)]


# pid class
pidcal = Pidcal()
# sliding window class
slidingwindow = SlidingWindow()

obstacles = None

def obstacle_callback(data):
    global obstacles
    obstacles = data

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    image = cv2.resize(image, (640,480))

#publish xycar_motor msg
def drive(Angle, Speed):
    global pub 

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    #if Angle > 0.12 and Angle < -0.12:
        #msg.angle = Angle*1.3
    #    msg.speed = Speed*0.5
    

    #pub.publish(msg)


def region_of_interest(image,image_copy,width,height1,height2,roiw,roih):
    src=np.array([[width/2-320,height1],[width/2+320,height1],[width/2+360,height2],[width/2-360,height2]],np.float32)
    #for i in src:
    #    cv2.circle(image_copy, (i[0], i[1]), 5, (0,0,255),-1)
    dst=np.array([[0,0],[roiw,0],[roiw,roih],[0,roih]],np.float32)
    M=cv2.getPerspectiveTransform(src,dst)
    return cv2.warpPerspective(image,M,(roiw,roih))


# show image and return lpos, rpos
def process_image(frame):
    global Width

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    blur_gray = cv2.GaussianBlur(blur_gray,(kernel_size, kernel_size), 0)
    # canny edge
    #low_threshold = 45
    #high_threshold = 100
    low_threshold = 5 #60 #41
    high_threshold = 60 #70 #70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    return edge_img

def start():
    global pub
    global image
    global cap
    global Width, Height
    global car_run_speed
    global cw_list

    #cap = cv2.VideoCapture("test_video/first_curve.avi")
    cap = cv2.VideoCapture("test_video/cross_walk.avi")
	
    # image height
    #h=image.shape[0]
    #image width
    #w=image.shape[1]

    x_old = 320

    setPoint = 320
    error = 0
    pTerm = 0.0
    cw_cnt=0
    cw_sum_old = 0

    PART=3

   # crosswalk_counter = Crosswalk_Counter()
    stop_counter = Stop_Counter()
    curve_detector = CurveDetector()

    ob = ObstacleDetector()
    #last_mode = ob.mode
    obstacle_cnt = 0
    crosswalk_cnt=0
    pid_list=list()

    while True:
        ret, image = cap.read()
        image = cv2.resize(image, (800,448))
        image_copy = image.copy()

        # cross walk start
        if slidingwindow.cw_sum == 0:
            cw_sum_old=0
        else:
            cw_sum_old=slidingwindow.cw_sum

        print('cw_sum_old', cw_sum_old)
        # cross walk end
        
        # preprocess the rgb image
    	edge_img = process_image(image)
        
        # Bird Eyes View Image
        #warp = region_of_interest(edge_img, 640, 350, 380, 640, 480)
        warp = region_of_interest(edge_img,image_copy, 640, 390, 420, 800, 448)


        out_img, x_location = slidingwindow.slidingwindow(warp)

        if x_location is None:
            cv2.circle(out_img, (x_old, 380), 5, (255,0,0),-1)
            x_location = x_old
            #x_location = -100
        elif x_location != None and PART==3:
	    if abs(x_location-x_old)>45:
	        print('x_location - x_old:                                 ', abs(x_location-x_old))
	        x_location=x_old
		cv2.circle(out_img, (x_location, 380), 5, (255,0,255),-1)
		print('jimin')
	else:
            cv2.circle(out_img, (x_location, 380), 5, (0,0,255),-1)
            x_old = int(x_location)
	

	pid = round(pidcal.pid_control(int(x_location),1), 6)
	
        curve_detector.list_update(pid)
        curve_detector.count_curve()

        print('x_location:                 ', x_location)
        print('pid:                 ', pid)

    	#pid = pTerm
    	#Angle = -pid * (180.0 / math.pi)
	Angle = pid

	#last_mode = ob.mode

	#ob.check(obstacles)
	
	st = 0
	if PART == 1 and curve_detector.curve_count == 2:
            car_run_speed = 0.3
	    if (ob.mode == Position.LEFT):
		obstacle_cnt+=1
	        for theta in range(280,495,10):
                    st = 0.575*np.sin(theta*np.pi/180)
		    drive(st,car_run_speed)
		    time.sleep(0.1)
		
                #while True:
                    #print("stop")

	    elif (ob.mode == Position.RIGHT):
		obstacle_cnt+=1
	        for theta in range(270,450,8):
                    st = 0.9*np.sin(theta*np.pi/180)
                    drive(-st,car_run_speed)

		    time.sleep(0.1)
		#while True:

		
	        #for theta in range(360,500,11):
                #    st = 0.19*np.sin(theta*np.pi/180)
		    #drive(-st, 2)
		
		    #time.sleep(0.5)	
	    else:
               print("dymo")
	       #drive(0, car_run_speed)

	    print('ob.mode : ', ob.mode)
	    print('st : ', st)

	#if last_mode == Position.NO and ob.mode == Position.LEFT:
	#    obstacle_cnt += 1
	
        cw_list.pop(0)
	cw_list.append(slidingwindow.cw_sum)

        print('obstacle_cnt!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: ', obstacle_cnt)

	if obstacle_cnt == 3: 
	    PART = 2
	    #drive(Angle, 0.2)
	
        #detected = crosswalk_counter.check_crosswalk(warp)
	stop_detected = stop_counter.check_stop_line(image)

	if stop_detected:
            cv2.waitKey(-1)
	    PART=1
	    obtacle_cnt=0
	    curve_detector.curve_count=0
        
        if PART == 3:
	    #print('detected!')
            if sum(cw_list) // 4 >= 2600:  # if stop_detected:
		print("crosswalk!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("type(sum(cw_list) // 4)) : ", type(sum(cw_list) // 4))
                #drive(0, 0)
                cv2.waitKey(-1)
                time.sleep(5)
                crosswalk_cnt += 1
		for i in range(40):
		    drive(0, 0.4)
                    time.sleep(0.1)
                   # rospy.spin()
                PART = 4
		continue

	print('car_run_speed', car_run_speed)
	print('cw_sum: ', slidingwindow.cw_sum)
        print("curve_detector.curve_count : {}".format(curve_detector.curve_count))
	print('st_yellow_count:            					', stop_counter.st_yellow_count)
	#print('cw_cnt:', crosswalk_counter.cw_cnt)

        try:
            out2.write(out_img)
	except:
	    pass


	if stop_counter.cnt==3:
	    while 1:
		#drive(0,0.0)
                print("stop_counter.cnt : {}".format(stop_counter.cnt))
		break


	#plt.figure(1)
	#plt.plot(curve_detector.pid_list)
	
        #print("~~~~~~~~~~~~~~~~~~",curve_detector.pid_list_sum,"~~~~~~~~~~~~~~~~~~~~")
        #curve_sum_list.append(curve_detector.pid_list_sum)

        plt.figure(1)
        plt.plot(curve_sum_list)
	
	cv2.putText(out_img, 'PID %f'%pid, (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'PART %d'%PART, (0,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'curve_count %d'%curve_detector.curve_count, (0,55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
	cv2.putText(out_img, 'stop_count %d'%stop_counter.cnt, (400,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
	cv2.putText(out_img, 'crosswalk_count %d'%crosswalk_cnt, (400,55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'obstacle_count %d'%obstacle_cnt, (0,80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'sum(cw_list)//4 %d' %int(sum(cw_list)//4), (0, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)	


        #cv2.imshow("src", src)
	cv2.imshow('origin:', image)
	cv2.imshow('image', edge_img)
	cv2.imshow('warp', warp)
        cv2.imshow('result',out_img)
        cv2.imshow('resuld',image_copy)
        # publish
        #drive(Angle, car_run_speed)
        #drive(0,0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
	    plt.show()
            break

    
if __name__ == '__main__':

    start()
