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

car_run_speed = 0.3
curve_count = 0
PART = 1

start_time = 0
end_time = 0
park_flag = 0
time_flag = 0

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
        if car_run_speed <= 0.6:
            # increase the constant value
            car_run_speed += 0.005
    elif PART == 5:
	car_run_speed = Speed
        

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
    low_threshold = 5 # 2 floor : 35  # 41
    high_threshold = 60  # 70
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
    global start_time
    global end_time
    global park_flag
    global time_flag

    # Node Name : auto_drive
    rospy.init_node('auto_drive')

    # motor publisher : pub
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    # usb_cam subscriber : image_sub
    # callback : img_callback
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    # obstacle_detector subscriber : obstacle_sub
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)

    print
    "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    # original image video writer
    out = cv2.VideoWriter(
        '/home/nvidia/Videos/9-25/original_{}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour,
                                                                      now.minute), cv2.VideoWriter_fourcc(*'MJPG'), 30,
        (640, 480))

    # sliding_window out_img video writer
    out2 = cv2.VideoWriter(
        '/home/nvidia/Videos/9-25/sliding_{}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour,
                                                                     now.minute), cv2.VideoWriter_fourcc(*'MJPG'), 30,
        (800, 448))

    # image height
    h = image.shape[0]
    # image width
    w = image.shape[1]

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
        # Folder : Video/9-21
	image = cv2.resize(image, (640,480))
        image_copy = image.copy()
        #obstacle_cnt = 0
        #PART = 1

        try:
            out.write(image)
        except:
            pass

        while not image.size == (h * w * 3):
            continue

        # --------------------------------------------------- #

        curve_count = curve_detector.curve_count

        # cross walk start
        #if slidingwindow.cw_sum == 0:
        #    cw_sum_old = 0
        #else:
        #    cw_sum_old = slidingwindow.cw_sum

        # cross walk end
        
        # --------------------------------------------------- #
        # preprocess the rgb image
        edge_img = process_image(image)

        # Bird Eyes View Image
        warp = region_of_interest(edge_img, image_copy, 640, 390, 420, 800, 448)

        out_img, x_location = slidingwindow.slidingwindow(warp)

        if x_location is None:
            cv2.circle(out_img, (x_old, 380), 5, (255, 0, 0), -1)
            x_location = x_location_old
        else:
            cv2.circle(out_img, (x_location, 380), 5, (0, 0, 255), -1)
            x_location_old = int(x_location)


        pid = round(pidcal.pid_control(int(x_location), PART), 6)

        curve_detector.list_update(pid)
        aaa = curve_detector.count_curve()
        x_location_list.append(pid)

        print('x_location:                                 ', x_location)
        #print('pid:                 ', pid)

        Angle = pid

        # last_mode = ob.mode

        ob.check(obstacles)

        st = 0

###
	PART = 5
###

	if PART == 1 and curve_count == 2:
	    PART = 2

        # ------------------------------------------------------------------------- #
        # -------------------OBSTACLE LEFT TO RIGHT TO LEFT------------------------ #
        # ------------------------------------------------------------------------- #
	# LEFT VERSION
        if PART == 2:
            if (ob.mode == Position.LEFT):
                obstacle_cnt += 1
		if obstacle_cnt == 1:   
                    for theta in range(270, 515, 10):   # (270, 515, 16)
                        st = 0.6 * np.sin(theta * np.pi / 180) #0.68
                        drive(st, car_run_speed)
                        time.sleep(0.1)

		elif obstacle_cnt == 3:
		    for theta in range(270, 520, 10):   # (270, 515, 16)
                        st = 0.40 * np.sin(theta * np.pi / 180) #0.68
                        drive(st, car_run_speed)
                        time.sleep(0.1) 
                continue


            elif (ob.mode == Position.RIGHT):
                obstacle_cnt += 1
                if obstacle_cnt == 2:
                    for theta in range(270, 430, 12): # (280, 440, 10) #270,450, 8
                        st = 0.22 * np.sin(theta * np.pi / 180) #0.9
                        drive(-st, car_run_speed)
                        time.sleep(0.1)
            
                continue
		

            # for theta in range(360,500,11):
            #    st = 0.19*np.sin(theta*np.pi/180)
            # drive(-st, 2)

            # time.sleep(0.5)
            else:
                #drive(0, car_run_speed)
                pass
        # ------------------------------------------------------------------------- #
        # ------------------------------------------------------------------------- #
        # ------------------------------------------------------------------------- #
 
        if PART == 2 and obstacle_cnt >= 3:
            PART = 3
            # when obstacle end -> stop
	    #while True:
            #    print('stop')
            #drive(Angle, 0.0)

        stop_detected = stop_counter.check_stop_line(image)

        if PART == 4 and stop_detected:
            PART = 1
            obstacle_cnt = 0
            curve_detector.curve_count = 0
            

	cw_list.pop(0)
	cw_list.append(slidingwindow.cw_sum)
	

       # if slidingwindow.cw_sum >= 2900 and cw_sum_old >= 2900: # check!!!!!!!!!!!!
        if PART == 3:
            #if slidingwindow.cw_sum >= ?? 
            if sum(cw_list) // 4 >= 2600:  # if stop_detected: 210
                drive(0, 0)
                time.sleep(5)
                crosswalk_cnt += 1
		for i in range(20):
		    drive(0, 0.4)
                    time.sleep(0.1)
                   # rospy.spin()
                PART = 4
		continue
	
	print("PART:            %d"%PART)

	while(True):
	    ob.check(obstacles)
	    print("rightX: %f	rightY: %f"%(ob.rightX, ob.rightY))
	    print("frontX: %f	frontY: %f"%(ob.frontX, ob.frontY)) 
	    #print(ob.mode)
	
	if PART == 5:
	    while(True):
		ob.check(obstacles)

		if time_flag == 0:
		    start_time = time.time()
		    time_flag = 1

		#park_flag = 3
		if park_flag == 0:
		    if int(time.time() - start_time) < 4:
			drive(0, 0.34)
		    else:
			park_flag = 1
			start_time = time.time()

		if park_flag == 1:
		    if int(time.time() - start_time) < 2:
			drive(-0.3, -0.375)
		    else:
			park_flag = 2
			start_time = time.time()
		if park_flag == 2:
		    if int(time.time() - start_time) < 2:
			drive(0.35, -0.25)
		    else:
		    	park_flag = 3
			start_time = time.time()
		if park_flag == 3:
		    steer = ob.rightX - 0.3
		    speed = 0

		    #if ob.rightX == 0.0 and ob.rightY == 0.0:
			#steer = 0.2
		    #if ob.frontY < -0.2 or (ob.frontX == 0.0 and ob.frontY == 0.0):
			#speed = 0.2
		    #else:
			#speed = -0.2
		
		    print("rightX: %f	rightY: %f"%(ob.rightX, ob.rightY))
		    print("frontX: %f	frontY: %f"%(ob.frontX, ob.frontY)) 

		    while time.time() - start_time < 0.2:
			drive(-steer, 0.2)

		    park_flag = 4
#	    if ob.mode == Position.RIGHT:
#		drive(-0.01, 0.1)
#		time.sleep(2)
#	    elif ob.mode == Position.FRONT:
#		if ob.y < -0.5:
#		    if abs(p.x) < 0.3:
#			if p.x >= 0.05:
#			    drive(0.01, -0.1)
#			else:
#			    drive(-0.01, -0.1)
#		else:
#		    if abs(p.x) < 0.3:
#			if p.x >= 0.05:
#			    drive(0.01, 0.1)
#			else:
#			    drive(-0.01, 0.1)




           
           #drive(Angle, 0.5)
	
	#print('cw_list  sum', sum(cw_list)//4)
        #print('cw_sum: ', slidingwindow.cw_sum) # delete after checking
        #print("curve_detector.curve_count : {}".format(curve_detector.curve_count))
        #print('st_yellow_count:            					', stop_counter.st_yellow_count) # delete after checking
        # print('cw_cnt:', crosswalk_counter.cw_cnt)

        

        #if stop_counter.cnt == 2: #3
         #   while 1:
                #drive(0, 0.0)
	#	print("stop_counter.cnt : {}".format(stop_counter.cnt))
         #        break

	#if curve_detector.curve_count > 1:
	 #   drive(0, 0.0)
	  #  break
        #plt.figure(1)
        #plt.plot(x_location_list)
	#print(curve_sum_list)
        #print("~~~~~~~~~~~~~~~~~~", curve_detector.pid_list_sum, "~~~~~~~~~~~~~~~~~~~~")

        cv2.putText(out_img, 'PID %f' % pid, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'PART %d' % PART, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'curve_count %d' % curve_detector.curve_count, (0, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
        cv2.putText(out_img, 'stop_count %d' % stop_counter.cnt, (400, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
        cv2.putText(out_img, 'crosswalk_count %d' % crosswalk_cnt, (400, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
        cv2.putText(out_img, 'obstacle_count %d' % obstacle_cnt, (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
        cv2.putText(out_img, 'sum(cw_list)//4 %d' % int(sum(cw_list)//4), (0, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
	try:
            out2.write(out_img)
        except:
            pass

        # cv2.imshow("src", src)
        #cv2.imshow('origin:', image)
        #cv2.imshow('image', edge_img)
        #cv2.imshow('warp', warp)
        cv2.imshow('result', out_img)
        #cv2.imshow('copy_image', image_copy)
        # publish

	#drive(Angle, car_run_speed)
        # drive(0,0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #plt.show()
            break

    rospy.spin()


if __name__ == '__main__':
    start()
