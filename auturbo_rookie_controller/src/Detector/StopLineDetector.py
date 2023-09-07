#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np

from Detector.utils import undistort

# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

class StopLineDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''
    def __init__(self):

        # BEV
        self.roi_height = 200
        self.roi_width = 640

        prev_target = 320
        self.frameRate = 11 #33

        # stopline detection param
        self.stopline_threshold = 180
        self.area_threshold = 2000
        self.lengh_threshold = 300
   
    def __call__(self, frame):
        '''
        return True if stopline is detected else False
        '''
        frame = undistort.undistort_func(frame)
        frame = self.warp_perspect(frame)

        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        L = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        # _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        _, stopline = cv2.threshold(L, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(stopline, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        detected = False
        for cont in contours:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)

            if not ((area > self.area_threshold) and (length > self.lengh_threshold)):
                continue
            # if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
            #     continue
            # approx_poly = cv2.approxPolyDP(cont, length*0.02, True)
            # for approx in approx_poly:
            #     cv2.circle(cont, tuple(approx[0]), 3, (255, 0, 0), -1)
            
            x, y, w, h = cv2.boundingRect(cont)
            center = (x + int(w/2), y + int(h/2))
            width = self.roi_width
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0,255,0), 2)
            print("detected boundingRect (x, y, w, h): ", x, y, w, h)
            print("detected boundingRect center (x, y): ", center)

            # if (200 <= center[0] <= (width - 200)) and (w > 400) & (h < 100):
            if (200 <= center[0] <= (width - 200)) and (w > 260) & (h < 50):
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0,255,255), 2)

                cv2.imshow('stopline', frame)
                key = cv2.waitKey(self.frameRate)
                print("detected stopline")
                # detected = True
                # # cv2.destroyWindow("stopline")
                # return detected

        cv2.imshow('stopline', frame)
        key = cv2.waitKey(self.frameRate)
        return detected
    
    def warp_perspect(self, img):
        width_margin = 215
        src = np.float32(
            [
                [0, 0],
                [0, self.roi_height - 50],
                [self.roi_width, 0],
                [self.roi_width, self.roi_height - 50],
            ]
        )  # Perspective transform을 위한 src 좌표 설정
        dst = np.float32(
            [
                [0, 0],
                [width_margin, self.roi_height - 50],
                [self.roi_width, 0],
                [self.roi_width - width_margin, self.roi_height - 50],
            ]
        )  # Perspective transform을 위한 dst 좌표 설정

        # src = np.float32([[0, 0], [0, self.roi_height-50 ],  [self.roi_width, 0], [640, self.roi_height-50 ]]) # Perspective transform을 위한 src 좌표 설정
        # dst = np.float32([[0, 0], [272, self.roi_height-50 ], [self.roi_width, 0], [367, self.roi_height-50 ]]) # Perspective transform을 위한 dst 좌표 설정
        M = cv2.getPerspectiveTransform(src, dst)  # 변환 행렬 생성 (src -> dst)
        # Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        # src, dst 시각화(디버깅 용)

        cv2.circle(img, (int(src[0][0]), int(src[0][1])), 1, (255, 0, 0), 10)
        cv2.circle(img, (int(src[1][0]), int(src[1][1])), 1, (0, 255, 0), 10)
        cv2.circle(img, (int(src[2][0]), int(src[2][1])), 1, (0, 0, 255), 10)
        cv2.circle(img, (int(src[3][0]), int(src[3][1])), 1, (255, 255, 0), 10)

        # cv2.circle(img, (int(dst[0][0]), int(dst[0][1])), 1, (255,0 ,0), 10)
        # cv2.circle(img, (int(dst[1][0]), int(dst[1][1])), 1, (0,255 ,0), 10)
        # cv2.circle(img, (int(dst[2][0]), int(dst[2][1])), 1, (0,0 ,255), 10)
        # cv2.circle(img, (int(dst[3][0]), int(dst[3][1])), 1, (255,255 ,0), 10)

        roi = img[280 : (280 + self.roi_height - 50), 0 : self.roi_width]  # ROI 적용

        cv2.imshow("roi", roi)

        warped_img = cv2.warpPerspective(
            roi, M, (self.roi_width, self.roi_height - 50), flags=cv2.INTER_LINEAR
        )  # 이미지 워핑으로 Bird Eye View 생성

        return warped_img