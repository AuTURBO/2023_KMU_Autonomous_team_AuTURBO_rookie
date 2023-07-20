#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

lane_bin_th = 135 #145
frameWidth = 0
frameHeight = 0
frame = None

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

    frameRate = 10 #33

    while True:
        retval, frame = cap.read()
        if not(retval):	# 프레임정보를 정상적으로 읽지 못하면
            break  # while문을 빠져나가기
        
        line_detect(frame)
        
        #cv2.imshow('frame', frame)	# 프레임 보여주기
        key = cv2.waitKey(frameRate)  # frameRate msec동안 한 프레임을 보여준다
        
        # 키 입력을 받으면 키값을 key로 저장 -> esc == 27(아스키코드)
        if key == 27:
            break	# while문을 빠져나가기

def region_of_interest(img, vertices, color3=(255,255,255), color1=255, show=False): # ROI 셋팅
    global frame
    if show == True:
        cv2.circle(frame, tuple(vertices[0][0]), 3, (255, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(frame, tuple(vertices[0][1]), 3, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.circle(frame, tuple(vertices[0][2]), 3, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(frame, tuple(vertices[0][3]), 3, (255, 255, 0), -1, cv2.LINE_AA)
        
        cv2.imshow("roi", frame)

    mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
    
    if len(img.shape) > 2: # Color 이미지(3채널)라면 :
        color = color3
    else: # 흑백 이미지(1채널)라면 :
        color = color1
        
    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
    cv2.fillPoly(mask, vertices, color)
    
    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

def canny(img, low_threshold, high_threshold, show=False): # Canny 알고리즘
    canny = cv2.Canny(img, low_threshold, high_threshold)

    if show == True:
        cv2.imshow("Canny", canny)
    return canny

def draw_lines(img, lines, color=[0, 0, 255], thickness=2): # 선 그리기
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap, show=False): # 허프 변환
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)

    if show == True:
        cv2.imshow("hough_lines", line_img)
    return line_img

def line_detect(img):
    global lane_bin_th, frameWidth, frameHeight 

    blur = cv2.GaussianBlur(img,(5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY)
    edge = canny(lane, 70, 210, show=True)
    hough_img = hough_lines(edge, 1, 1 * np.pi/180, 30, 10, 20, show=True) # 허프 변환


    vertices = np.array([[(50,frameHeight),(frameWidth/2-45, frameHeight/2+60), (frameWidth/2+45, frameHeight/2+60), (frameWidth-50,frameHeight)]], dtype=np.int32)
    ROI_img = region_of_interest(edge, vertices, show=True) # ROI 설정


    cv2.imshow("Image window", img)
    cv2.imshow("L", L)
    cv2.imshow("lane ", lane)
    # cv2.imshow("ROI_img ", ROI_img)

def image_callback(msg):
    global lane_bin_th
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def start():
    rospy.init_node('image_listener')
    image_topic = "/usb_cam/image_raw"

    rospy.Subscriber(image_topic, Image, image_callback)
    #rospy.spin()


    video_read('xycar_track2.mp4')

if __name__ == '__main__':
    start()
