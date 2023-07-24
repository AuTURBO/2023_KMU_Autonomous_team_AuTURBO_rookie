#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
from preprocessor import PreProcessor

lane_bin_th = 120 #145
frameWidth = 0
frameHeight = 0
frame = None

blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)

roi_height = 200
roi_width = 640

pre_module = PreProcessor(roi_height, roi_width)

def main(frame):
        frameRate = 11 #33

        gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
        #cv2.imshow("gblur_img", gblur_img)

        gray = cv2.cvtColor(gblur_img, cv2.COLOR_BGR2GRAY)
        adaptive_binary = threshold_binary(gray, lane_bin_th, "adaptive", window_name="adaptive_binary", show=False)
        #cv2.imshow("adaptive_binary", adaptive_binary)

        warped_img = pre_module.warp_perspect(adaptive_binary)
        # cv2.imshow('warped_img', warped_img)	

        edge = canny(warped_img, 70, 210, show=True)

        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        kernel_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

        closing = cv2.morphologyEx(warped_img, cv2.MORPH_CLOSE,kernel_close)
        # cv2.imshow('closing', closing)	# 프레임 보여주기

        edge_to_closing = cv2.morphologyEx(edge, cv2.MORPH_CLOSE,kernel_close)
        cv2.imshow('edge_to_closing', edge_to_closing)	# 프레임 보여주기

        msk, lx, ly, mx, my, rx, ry = pre_module.sliding_window(edge_to_closing)

        filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry = pre_module.filtering_lane(msk, lx, ly, mx, my, rx, ry)
        pre_module.drawing_lane(msk, filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)
        cv2.imshow("Lane Detection - Sliding Windows", msk)
        # erosion = cv2.erode(closing,kernel_erosion,iterations = 1)

        # opeing = cv2.morphologyEx(closing, cv2.MORPH_OPEN,kernel_erosion)


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
        #detect_lane(frame, "frame")

        # gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
        # #cv2.imshow("gblur_img", gblur_img)

        # gray = cv2.cvtColor(gblur_img, cv2.COLOR_BGR2GRAY)
        # adaptive_binary = threshold_binary(gray, lane_bin_th, "adaptive", window_name="adaptive_binary", show=False)
        # #cv2.imshow("adaptive_binary", adaptive_binary)

        # warped_img = pre_module.warp_perspect(adaptive_binary)
        # # cv2.imshow('warped_img', warped_img)	

        # edge = canny(warped_img, 70, 210, show=True)

        # kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        # kernel_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

        # closing = cv2.morphologyEx(warped_img, cv2.MORPH_CLOSE,kernel_close)
        # # cv2.imshow('closing', closing)	# 프레임 보여주기

        # edge_to_closing = cv2.morphologyEx(edge, cv2.MORPH_CLOSE,kernel_close)
        # cv2.imshow('edge_to_closing', edge_to_closing)	# 프레임 보여주기

        # msk, lx, ly, rx, ry = pre_module.sliding_window(edge_to_closing)

        # cv2.imshow("Lane Detection - Sliding Windows", msk)
        # # erosion = cv2.erode(closing,kernel_erosion,iterations = 1)

        # # opeing = cv2.morphologyEx(closing, cv2.MORPH_OPEN,kernel_erosion)


        # cv2.imshow('frame', frame)	# 프레임 보여주기

        # key = cv2.waitKey(frameRate)  # frameRate msec동안 한 프레임을 보여준다
        
        # # 키 입력을 받으면 키값을 key로 저장 -> esc == 27(아스키코드)
        # if key == 27:
        #     break	# while문을 빠져나가기

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

def find_lane(origin, img):
    global frameWidth, frameHeight
    lane_pixel_count = 0

    first_lane_x = 0
    second_lane_x = 0

    first_inspect_line_y = 330
    cv2.line(img, (0, first_inspect_line_y), (640, first_inspect_line_y), blue, 5)
    first_inspect_line_y_roi = frameHeight//2-(frameHeight-first_inspect_line_y)
    lane_find_count = 0
    for i in range(frameWidth):
        lane_pixel_inspect_skip = 20
        #print(i)
        print(lane_find_count)
        if img[first_inspect_line_y_roi][i] == 255:
            if i < first_lane_x + lane_pixel_inspect_skip:
                continue
            lane_pixel_count += 1

            if lane_pixel_count == 15:

                print(f"lane_pixel_count: {lane_pixel_count}, lane_find_count: {lane_find_count}")
                #print(lane_find_count)
                #cv2.waitKey(0)
                if lane_find_count == 0:
                    if i > 200 and i < 380:
                        print(i)
                        #cv2.waitKey(0)
                    first_lane_x = i
                    lane_pixel_count = 0
                    lane_find_count += 1

                elif lane_find_count == 1:
                    if i > 200 and i < 380:
                        print(i)
                        #cv2.waitKey(0)
                    second_lane_x = i
                    lane_pixel_count = 0
                    lane_find_count += 1



    
    # cv2.circle(origin, (first_lane_x,first_inspect_line_y), 8, blue, -1)
    # cv2.circle(origin, (second_lane_x,first_inspect_line_y), 8, green, -1)

    # cv2.imshow("show", img)
    #cv2.imshow("origin", img)

def detect_lane(img, window_name):
    global lane_bin_th, frameWidth, frameHeight 

    _, L, _ = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))

    # vertices = np.array([[(50,frameHeight),(frameWidth/2-45, frameHeight/2+60), (frameWidth/2+45, frameHeight/2+60), (frameWidth-50,frameHeight)]], dtype=np.int32)
    # ROI_img = region_of_interest(L, vertices, show=False) # ROI 설정

    L = L[frameHeight//2 : , 0:frameWidth]

    # lane = threshold_binary(L, lane_bin_th, "basic", window_name="threshold", show=True)
    lane = threshold_binary(L, lane_bin_th, "adaptive", window_name="adaptive", show=True)
    # threshold_binary(L, lane_bin_th, "otsu", window_name="otsu", show=True)

    #edge = canny(lane, 70, 210, show=True)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
    kernel_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

    closing = cv2.morphologyEx(lane, cv2.MORPH_CLOSE,kernel_close)
    erosion = cv2.erode(closing,kernel_erosion,iterations = 1)

    opeing = cv2.morphologyEx(closing, cv2.MORPH_OPEN,kernel_erosion)

    cv2.imshow("closing", closing)
    cv2.imshow("erosion", erosion)
    #cv2.imshow("opeing", opeing)

    find_lane(img, closing)

    #hough_img = hough_lines(edge, 1, 1 * np.pi/180, 100, 60, 20, show=True) # 허프 변환
    
    cv2.imshow(window_name, lane)

    #cv2.imshow("Image window", img)
    #cv2.imshow("L", L)
    #cv2.imshow("lane ", lane)
    #cv2.imshow("ROI_img ", ROI_img)

def image_callback(msg):
    global lane_bin_th
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        #main(cv_image)
        #cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def start():
    rospy.init_node('image_listener')
    image_topic = "/usb_cam/image_raw"

    rospy.Subscriber(image_topic, Image, image_callback)
    #rospy.spin()


    video_read('xycar_track2.mp4')

if __name__ == '__main__':
    start()
