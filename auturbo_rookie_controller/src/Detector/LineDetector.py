#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from Detector.BEV import BEV
from Detector.PreProcessor import PreProcessor

# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

obstacle_info = "middle"
class LineDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''
    def __init__(self):

        # BEV
        roi_height = 200
        roi_width = 640
        self.bev = BEV(roi_height, roi_width)

        prev_target = 320
        self.frameRate = 11 #33

        # stopline detection param
        self.stopline_threshold = 125
        self.area_threshold = 2000
        self.lengh_threshold = 300
        self.pre_module = PreProcessor(roi_height, roi_width)
   
    def __call__(self, img):
        '''
        return True if stopline is detected else False
        '''
        bev = self.bev(img)

        edge = canny(bev, 70, 210, show=False)

        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        kernel_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

        #closing = cv2.morphologyEx(warped_img, cv2.MORPH_CLOSE,kernel_close)
        # cv2.imshow('closing', closing)	# 프레임 보여주기

        edge_to_closing = cv2.morphologyEx(edge, cv2.MORPH_CLOSE,kernel_close)
        #cv2.imshow('edge_to_closing', edge_to_closing)	# 프레임 보여주기

        edge_to_closing = cv2.medianBlur(edge_to_closing,5)

        msk, lx, ly, mx, my, rx, ry = self.pre_module.sliding_window(edge_to_closing)

        filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry = self.pre_module.filtering_lane(msk, lx, ly, mx, my, rx, ry)
        self.pre_module.drawing_lane(msk, filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)

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

        cv2.circle(img, (int(target), int(480 - 135)), 1, (120, 0, 255), 10)

        cv2.imshow("Lane Detection - Sliding Windows", msk)
        cv2.imshow('frame', img)	# 프레임 보여주기

        key = cv2.waitKey(self.frameRate)

        return int(angle)
    
def canny(img, low_threshold, high_threshold, show=False): # Canny 알고리즘
    canny = cv2.Canny(img, low_threshold, high_threshold)

    if show == True:
        cv2.imshow("Canny", canny)
    return canny

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
        # print("No Obstacle!!!")
        pass
    elif obstacle_info == "right": # obstacle이 우측에 존재
        if rx != None and len(rx) > 3 and mx != None and len(mx) > 3: #  우측차선과 중간차선이 모두 존재할떄
            target = (rx[0] + mx[0]) // 2  #  우측차선과 중간차선의 평균
        if rx != None and len(rx) > 3:  # 우측 차선만 존재할떄
            target = rx[0] - obstacle_margin 
        if mx != None and len(mx) > 3:  # 중간 차선만 존재할떄
            target = mx[0] + obstacle_margin -80
        # print("Obstacle Right!!!")
    elif obstacle_info == "left": # obstacle이 좌측에 존재
        if lx != None and len(lx) > 3 and mx != None and len(mx) > 3: #  좌측차선과 중간차선이 모두 존재할떄
            target = (lx[0] + mx[0]) // 2 #  좌측차선과 중간차선의 평균
        if lx != None and len(lx) > 3:  # 좌측 차선만 존재할떄
            target = lx[0] + obstacle_margin 
        if mx != None and len(mx) > 3:  # 중간 차선만 존재할떄
            target = mx[0] - obstacle_margin +80

        # print("Obstacle Left!!!")

    # print(f"target: {target}")
    print(target)
    return int(target)
def obstacle_callback(msg):
    global obstacle_info
    obstacle_info = msg.data