#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
#from matplotlib import pyplot as plt
from preprocessor import PreProcessor
#from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32
from std_msgs.msg import String

from utils import undistort

#ack_msg = xycar_motor()
#ack_publisher = None

steer_angle = Int32()
steer_angle_publisher = None

lane_bin_th = 120  # 145
frameWidth = 0
frameHeight = 0
frame = None

blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)

roi_height = 200
roi_width = 640

obstacle_info = "middle"

pre_module = PreProcessor(roi_height, roi_width)

def map(x,input_min,input_max,output_min,output_max):
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

def LowPassFilter(alpha, prev, x):
    """
    (param) alpha : weight for previous estimation
            prev : previous estimation
            x : new data
    (return) estimation
    """
    return alpha * prev + (1 - alpha) * x


def simple_controller(lx, ly, mx, my, rx, ry):
    global obstacle_info
    target = 320
    side_margin = 140
    obstacle_margin = 70

    if lx != None and rx != None and len(lx) > 5 and len(rx) > 5:
        # print("ALL!!!")
        target = (lx[0] + rx[0]) // 2
    elif mx != None and len(mx) > 3:
        # print("Mid!!!")
        target = mx[0]
    elif lx != None and len(lx) > 3:
        # print("Right!!!")
        #print(f"val: {lx[0]}")
        target = lx[0] + side_margin
    elif rx != None and len(rx) > 3:
        # print("Left!!!")
        target = rx[0] - side_margin


    if obstacle_info == "middle": # obstacle이 우측에 존재
        print("No Obstacle!!!")
        pass
    elif obstacle_info == "right": # obstacle이 우측에 존재
        if rx != None and len(rx) > 3 and mx != None and len(mx) > 3: #  우측차선과 중간차선이 모두 존재할떄
            target = (rx[0] + mx[0]) // 2 #  우측차선과 중간차선의 평균
        if rx != None and len(rx) > 3:  # 우측 차선만 존재할떄
            target = rx[0] - obstacle_margin 
        if mx != None and len(mx) > 3:  # 중간 차선만 존재할떄
            target = mx[0] + obstacle_margin + 80
        print("Obstacle Right!!!")
    elif obstacle_info == "left": # obstacle이 좌측에 존재
        if lx != None and len(lx) > 3 and mx != None and len(mx) > 3: #  좌측차선과 중간차선이 모두 존재할떄
            target = (lx[0] + mx[0]) // 2 #  좌측차선과 중간차선의 평균
        if lx != None and len(lx) > 3:  # 좌측 차선만 존재할떄
            target = lx[0] + obstacle_margin
        if mx != None and len(mx) > 3:  # 중간 차선만 존재할떄
            target = mx[0] - obstacle_margin - 80
        print("Obstacle Left!!!")

    # print(f"target: {target}")
    return int(target)


def main(frame):
    global ack_publisher
    global steer_angle_publisher

    global steer_angle

    prev_target = 320
    frameRate = 11 #33

    frame = undistort.undistort_func(frame)

    #cv2.imshow("Undistort", frame)

    gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
    #cv2.imshow("gblur_img", gblur_img)

    gray = cv2.cvtColor(gblur_img, cv2.COLOR_BGR2GRAY)
    adaptive_binary = threshold_binary(gray, lane_bin_th, "adaptive", window_name="adaptive_binary", show=False)
    #cv2.imshow("adaptive_binary", adaptive_binary)

    warped_img = pre_module.warp_perspect(adaptive_binary)
    # cv2.imshow('warped_img', warped_img)	

    edge = canny(warped_img, 70, 210, show=False)

    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
    kernel_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

    #closing = cv2.morphologyEx(warped_img, cv2.MORPH_CLOSE,kernel_close)
    # cv2.imshow('closing', closing)	# 프레임 보여주기

    edge_to_closing = cv2.morphologyEx(edge, cv2.MORPH_CLOSE,kernel_close)
    #cv2.imshow('edge_to_closing', edge_to_closing)	# 프레임 보여주기

    edge_to_closing = cv2.medianBlur(edge_to_closing,5)

    msk, lx, ly, mx, my, rx, ry = pre_module.sliding_window(edge_to_closing)

    filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry = pre_module.filtering_lane(msk, lx, ly, mx, my, rx, ry)
    pre_module.drawing_lane(msk, filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)

    target = simple_controller(filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)

    #target = LowPassFilter(0.9, prev_target, target)
    prev_target = target
    #print(f"filtered_target: {target}")

    angle = target - 320
    angle = map(angle, -100, 100, -50, 50)
    angle = angle * 0.9
    #print(f"angle: {angle}")
    #ack_msg.speed = int(20)
    #ack_msg.angle = int(angle)

    # ack_publisher.publish(ack_msg)

    steer_angle.data = int(angle)

    steer_angle_publisher.publish(steer_angle)

    cv2.circle(frame, (int(target), int(480 - 135)), 1, (120, 0, 255), 10)

    cv2.imshow("Lane Detection - Sliding Windows", msk)
    cv2.imshow('frame', frame)	# 프레임 보여주기

    key = cv2.waitKey(frameRate)  # frameRate msec동안 한 프레임을 보여준다
    
    # 키 입력을 받으면 키값을 key로 저장 -> esc == 27(아스키코드)
    if key == 27:
        exit(0)

def video_read(fname):
    global frameWidth, frameHeight
    global frame

    path = '../video/'  
    filePath = os.path.join(path, fname)
    print(filePath)

    if os.path.isfile(filePath):	# 해당 파일이 있는지 확인
        # 영상 객체(파일) 가져오기
        cap = cv2.VideoCapture(filePath)
    else:
        print("파일이 존재하지 않습니다.")  

    # 프레임을 정수형으로 형 변환
    frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))	# 영상의 넓이(가로) 프레임
    frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))	# 영상의 높이(세로) 프레임
    
    frame_size = (frameWidth, frameHeight)
    print('frame_size={}'.format(frame_size))

    frameRate = 11 #33

    while True:
        retval, frame = cap.read()
        
        if not(retval):	# 프레임정보를 정상적으로 읽지 못하면
            break  # while문을 빠져나가기
        main(frame)
        


def threshold_binary(img, lane_bin_th, method, thresholding_type=cv2.THRESH_BINARY, window_name="threshold", show=False):
    if method == "adaptive":
        lane = cv2.adaptiveThreshold(img, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    cv2.THRESH_BINARY_INV,11,9)
    elif method == "otsu":
        _, lane = cv2.threshold(img, 0, 255, thresholding_type+cv2.THRESH_OTSU)
    elif method == "basic":
        _, lane = cv2.threshold(img, lane_bin_th, 255, thresholding_type)

    if show == True:
        cv2.imshow(window_name, lane)
    return lane

def canny(img, low_threshold, high_threshold, show=False): # Canny 알고리즘
    canny = cv2.Canny(img, low_threshold, high_threshold)

    if show == True:
        cv2.imshow("Canny", canny)
    return canny

def image_callback(msg):
    global lane_bin_th
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        main(cv_image)
        #cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def obstacle_callback(msg):
    global obstacle_info
    obstacle_info = msg.data

def start():
    global ack_publisher
    global steer_angle_publisher
    rospy.init_node("line_detector")
    image_topic = "/usb_cam/image_raw"
    obstacle_topic = "/obstacle/info"

    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(obstacle_topic, String, obstacle_callback)
    #ack_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    steer_angle_publisher = rospy.Publisher('xycar_angle', Int32, queue_size=1)

    #video_read('xycar_track2.mp4')
    rospy.spin()

if __name__ == '__main__':
    start()