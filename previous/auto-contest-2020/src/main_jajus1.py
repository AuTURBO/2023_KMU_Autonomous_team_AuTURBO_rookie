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
#from Stop_Counter_Test import Stop_Counter
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

# start_flag
start_flag = False

x_location_list = list()

pub = None
x_location_old = 400

car_run_speed = 0.3
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
    global start_flag
    
    if start_flag is False:
        msg = xycar_motor()
        msg.angle = 0
        msg.speed = 0.3
        pub.publish(msg)
        return
    elif PART == 1:
	#car_run_speed = Speed
        # ?? < Angle < ?? 
	if (Angle > 0.1 or Angle < -0.1) and car_run_speed >= 0.6:
	    #print('speed decccccccccccccccccccccccccccccccccccccccccccccccccccccc')
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
def process_image(frame,curve_count):
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
    low_threshold_1 = 60 #0925 jajus 60 # 2 floor : 35  # 41
    high_threshold_1 = 70  # 70
    low_threshold_2 = 90
    high_threshold_2 = 100
    low_threshold = 35
    high_threshold = 60
    edge_img_1 = cv2.Canny(np.uint8(blur_gray), low_threshold_1, high_threshold_1)
    edge_img_2 = cv2.Canny(np.uint8(blur_gray), low_threshold_2, high_threshold_2)
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    if 1 <= curve_count < 2:
	print('low_threshold',low_threshold_2)
	print('high_threshold', high_threshold_2)
	return edge_img_2
    else:
	print('low_threshold',low_threshold_1)
	print('high_threshold', high_threshold_1)
	return edge_img_1
    #return edge_img


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
    global start_flag

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
    #stop_counter = Stop_Counter()
    curve_detector = CurveDetector()

    ob = ObstacleDetector()
    # last_mode = ob.mode
    obstacle_cnt = 0
    crosswalk_cnt = 0
    pid_list = list()
    stop_cnt=1
    part_3_curve_count=0
    direction = 0

    while True:
        if start_flag is False:
            for i in range(20000):
                drive(0, 0.3)
                #time.sleep(0.1)
            start_flag = True
	    
                #rospy.spin()
            #continue

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
        edge_img = process_image(image, curve_count)

        # Bird Eyes View Image
        warp = region_of_interest(edge_img, image_copy, 640, 380, 410, 800, 448)

        out_img, x_location = slidingwindow.slidingwindow(warp)

        if x_location is None:
            cv2.circle(out_img, (x_location_old, 380), 5, (255, 0, 0), -1)
            x_location = x_location_old
        else:
            cv2.circle(out_img, (x_location, 380), 5, (0, 0, 255), -1)
            x_location_old = int(x_location)


        pid = round(pidcal.pid_control(int(x_location), PART), 6)

        curve_detector.list_update(pid)
        aaa = curve_detector.count_curve()
        x_location_list.append(pid)

        #print('x_location:                                 ', x_location)
        #print('pid:                 ', pid)

        Angle = pid

        # last_mode = ob.mode

        ob.check(obstacles)

        st = 0

	if PART == 1 and curve_count == 2:
	    PART = 2

        # ------------------------------------------------------------------------- #
        # -------------------OBSTACLE LEFT TO RIGHT TO LEFT------------------------ #
        # ------------------------------------------------------------------------- #
	# LEFT VERSION
	
        if obstacle_cnt == 0 and ob.mode == Position.LEFT:
	    direction = 1
	elif obstacle_cnt == 0 and ob.mode == Position.RIGHT:
	    direction = 2

        if PART == 2:
	    # LEFT - RIGHT - LEFT
	    if direction == 1:
		print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                if (ob.mode == Position.LEFT):
                    obstacle_cnt += 1
	    	    if obstacle_cnt == 1:   
                        for theta in range(270, 515, 14): #10  # (270, 515, 16)
                            st = 0.6 * np.sin(theta * np.pi / 180) #0.68
                            drive(st, car_run_speed)
                            time.sleep(0.1)

	    	    elif obstacle_cnt >= 3:
			st = 0
		        for theta in range(315, 445, 9):   # (270, 515, 16)
                            st = 0.4 * np.sin(theta * np.pi / 180) #0.68
                            drive(st, car_run_speed)
                            time.sleep(0.1) 
			for i in range(3000):
			    drive(st, car_run_speed)
                    continue


                elif (ob.mode == Position.RIGHT):
                    obstacle_cnt += 1
                    if obstacle_cnt == 2:
                        for theta in range(270, 530, 15): # (280, 440, 10) #270,450, 8
                            st = 0.8 * np.sin(theta * np.pi / 180) #0.9
                            drive(-st, car_run_speed)
                            time.sleep(0.1)
    
                    continue
  
                else:
                    pass
	   
	    # RIGHT - LEFT - RIGHT
	    elif direction == 2:
		print("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
		if (ob.mode == Position.LEFT):
                    obstacle_cnt += 1
		    if obstacle_cnt == 2:
                        for theta in range(280, 520, 12):
                            st = 0.71 * np.sin(theta * np.pi / 180)
                            drive(st, car_run_speed)
                            time.sleep(0.1)
                        continue

                elif (ob.mode == Position.RIGHT):
		    obstacle_cnt += 1
	    	    if obstacle_cnt ==1 :
                        for theta in range(280, 470, 10): #270,450, 8
                            st = 0.32 * np.sin(theta * np.pi / 180) #0.9
                            drive(-st, car_run_speed)
                            time.sleep(0.1)
	  	    if obstacle_cnt == 3:
 		        for theta in range(270, 510, 9): #270,450, 8
                            st = 0.55 * np.sin(theta * np.pi / 180) #0.9
                            drive(-st, car_run_speed)
                            time.sleep(0.1)
                    continue
		
                else:
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

#        stop_detected = stop_counter.check_stop_line(image)

        #if PART == 4 and stop_detected:
	 #   for i in range(5):
	  #      drive(0.5,0.3)
	   #     time.sleep(0.1)
           # PART = 1
           # obstacle_cnt = 0
           # curve_detector.curve_count = 0
            

	cw_list.pop(0)
	cw_list.append(slidingwindow.cw_sum)
	

       # if slidingwindow.cw_sum >= 2900 and cw_sum_old >= 2900: # check!!!!!!!!!!!!
        if PART == 3:
            #if slidingwindow.cw_sum >= ?? 
            if sum(cw_list) // 4 >= 3020 : #1970:#3020: #0925 jajus 3020:  # if stop_detected: 210
                drive(0, 0)
                time.sleep(8)
                crosswalk_cnt += 1
		part_3_curve_count = curve_detector.curve_count
		for i in range(10):
		    drive(0, 0.4)
                    time.sleep(0.1)
                   # rospy.spin()
                PART = 4
		continue
	if PART == 4:
	    if (sum(cw_list)//4 >= 3200) and ((curve_detector.curve_count - part_3_curve_count) >= 1): #0925_jajus 3350 #3200
		PART = 1
                obstacle_cnt = 0
                curve_detector.curve_count = 0
		stop_cnt+=1
		part_3_curve_count = 0



           
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
	#plt.plot(curve_detector.pid_sum_list)
	#print(curve_sum_list)
        #print("~~~~~~~~~~~~~~~~~~", curve_detector.pid_list_sum, "~~~~~~~~~~~~~~~~~~~~")

        cv2.putText(out_img, 'PID %f' % pid, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'PART %d' % PART, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(out_img, 'curve_count %d' % curve_detector.curve_count, (0, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
        
        cv2.putText(out_img, 'crosswalk_count %d' % crosswalk_cnt, (400, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
        cv2.putText(out_img, 'obstacle_count %d' % obstacle_cnt, (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
        cv2.putText(out_img, 'sum(cw_list)//4 %d' % int(sum(cw_list)//4), (0, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)

	cv2.putText(out_img, 'stop_count %d' % stop_cnt, (400, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
	cv2.putText(out_img, 'part_3_curve_count %d' % int(part_3_curve_count), (400, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)
	cv2.putText(out_img, 's-cw %d' % int(curve_detector.curve_count - part_3_curve_count), (400, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2)

	try:
            out2.write(out_img)
        except:
            pass

        # cv2.imshow("src", src)
        cv2.imshow('origin:', image)
        #cv2.imshow('image', edge_img)
        #cv2.imshow('warp', warp)
        cv2.imshow('result', out_img)
        #cv2.imshow('copy_image', image_copy)
        # publish
        drive(Angle, car_run_speed)
        # drive(0,0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            plt.show()
            break

    rospy.spin()


if __name__ == '__main__':
    start()
