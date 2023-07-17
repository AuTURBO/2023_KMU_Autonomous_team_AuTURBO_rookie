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

import sys
import os
import signal
import time
from datetime import datetime # for record

#from Crosswalk_Counter import Crosswalk_Counter
from Stop_Counter import Stop_Counter
from CurveDetector import CurveDetector
from Pidcal import Pidcal
from SlidingWindow import SlidingWindow

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

Run_speed = 0.0

now = datetime.now() # for record
pidcal = Pidcal()
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

    pub.publish(msg)


def region_of_interest(image,width,height1,height2,roiw,roih):
    src=np.array([[width/2-280,height1],[width/2+280,height1],[width/2+320,height2],[width/2-320,height2]],np.float32)
    dst=np.array([[0,0],[roiw,0],[roiw,roih],[0,roih]],np.float32)
    M=cv2.getPerspectiveTransform(src,dst)
    return cv2.warpPerspective(image,M,(roiw,roih))


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 45
    high_threshold = 100
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    return edge_img

beforeErr = 0

def start():
    global pub
    global image
    global cap
    global Run_speed
    global Width, Height
    global car_run_speed
	
    # Node Name : auto_drive
    rospy.init_node('auto_drive')
    
    # motor publisher : pub
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    
    # usb_cam subscriber : image_sub 
    # callback : img_callback
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    # obstacle_detector subscriber : obstacle_sub
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)

    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)
    
    # original image video writer
    out = cv2.VideoWriter('/home/nvidia/Videos/9-16/original_{}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640,480))

    # sliding_window out_img video writer
    out2 = cv2.VideoWriter('/home/nvidia/Videos/9-16/sliding_{}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640,480))

    # image height
    h=image.shape[0]
    #image width
    w=image.shape[1]

    x_loc_list=list()
    x_old=0

    setPoint = 320
    error = 0
    pTerm = 0.0
    cw_cnt=0
    cw_sum_old = 0

   # crosswalk_counter = Crosswalk_Counter()
    stop_counter = Stop_Counter()
    curve_detector = CurveDetector()

    ob = ObstacleDetector()
    last_mode = ob.mode
    obstacle_cnt = 0
    #time.sleep(3)

    while True:
        # Folder : Video/9-16
        try:
            out.write(image)
	    out2.write(image)
        except: pass

	if slidingwindow.cw_sum == 0:
	    cw_sum_old=0
	else:
	    cw_sum_old=slidingwindow.cw_sum

	print('cw_sum_old', cw_sum_old)

        while not image.size == (h*w*3):
            continue

    	edge_img = process_image(image)
        print("ros is spin?")

        # Bird Eyes View Image
        warp = region_of_interest(edge_img, 640, 350, 380, 640, 480)

        out_img, x_location = slidingwindow.slidingwindow(warp)

        if x_location is None:
            #Angle = (x_old - 320)*5 // 18
            cv2.circle(out_img, (x_old, 360), 5, (255,0,255),-1)
	    error = setPoint - x_old
            x_location = x_old

        else:
            cv2.circle(out_img, (x_location, 360), 5, (255,0,255),-1)
            x_old = int(x_location)
           # Angle = (x_location - 320)*5 // 18
            x_loc_list.append(x_location)
	    error = setPoint - x_location

	pid = round(pidcal.pid_control(int(x_location)), 6)
        print('pid:                 ', pid)
	beforeErr = error
    	kp = 0.005
    	pTerm = kp * error

    	#pid = pTerm
    	#Angle = -pid * (180.0 / math.pi)
	Angle = pid

	last_mode = ob.mode

	ob.check(obstacles)
	
        #print(ob.mode)
	st = 0
	if stop_counter.flag == 1 and curve_detector.curve_count == 2:
            Run_speed = 3.0
	    if (ob.mode == Position.LEFT):
	        for theta in range(280,495,10):
                    st = 0.575*np.sin(theta*np.pi/180)
		    drive(st,Run_speed)
		    time.sleep(0.1)
		
                #while True:
                    #print("stop")

	    elif (ob.mode == Position.RIGHT):
	        for theta in range(270,450,8):
                    st = 0.9*np.sin(theta*np.pi/180)
                    drive(-st,Run_speed)

		    time.sleep(0.1)
		#while True:

		
	        #for theta in range(360,500,11):
                #    st = 0.19*np.sin(theta*np.pi/180)
		    #drive(-st, 2)
		
		    #time.sleep(0.5)	
	    else:
	       drive(0, Run_speed)

	    print('ob.mode: ', ob.mode)
	    print('st: ', st)

	if last_mode == Position.NO and ob.mode == Position.LEFT:
	    obstacle_cnt += 1
	
        print('obstacle_cnt!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: ', obstacle_cnt)

	print(stop_counter.flag)
	if obstacle_cnt == 2: 
	    stop_counter.flag = 2
	    drive(Angle, 2)
	    print(stop_counter.flag)
	
        #detected = crosswalk_counter.check_crosswalk(warp)
	stop_detected = stop_counter.check_stop_line(image)
	
	if slidingwindow.cw_sum >= 4500 and cw_sum_old >= 4500:
	    print('detected!')
	    if stop_counter.flag == 2: # if stop_detected:
	        drive(0,0)
	        time.sleep(5)
		#drive(0,4)
		#time.sleep(2)
		stop_counter.flag=3 


#	if detected:
	#    print('detected!')
	#    if crosswalk_counter.cw_cnt>=2 and check < 1:
	#	print('detected 2!')
	#        if stop_counter.flag==True:
	#	    print('detected 3!')
	#	    drive(0,0)
	#           time.sleep(3)
	#   	    stop_counter.flag=False
	#	    check += 1
	#        else:
  	#	    drive(Angle, 0)

	print('cw_sum: ', slidingwindow.cw_sum)
	print('flag: ',stop_counter.flag)
        print("curve_detector.curve_count : {}".format(curve_detector.curve_count))
	print('st_yellow_count:            					', stop_counter.st_yellow_count)
	#print('cw_cnt:', crosswalk_counter.cw_cnt)

        try:
            out2.write(out_img)
        except: pass


        #if detected: # stop line detected
	   # if stop_counter.flag==True:
	    #car_run_speed = 0.0
         #   drive(0,0)

                #ROS_INFO("Cross walk detected")
          #  time.sleep(3)
		#stop_counter.flag=False
	    #else:
		#drive(Angle,4)
                #car_run_speed = 2.0

	if stop_counter.cnt==3:
	    while 1:
		drive(0,0)
		break

        #cv2.imshow("src", src)
	cv2.imshow('origin:', image)
	cv2.imshow('image', edge_img)
	cv2.imshow('warp', warp)
        cv2.imshow('result',out_img)
        # publish
        drive(Angle, Run_speed)
        #drive(0,0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':

    start()
