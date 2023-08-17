#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from Detector.BEV import BEV

# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

class StopLineDetector(object):
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
   
    def __call__(self, img):
        '''
        return True if stopline is detected else False
        '''
        bev = self.bev(img)

        blur = cv2.GaussianBlur(img, (5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        _, lane = cv2.threshold(L, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(lane, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        detected = False
        for cont in contours:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)

            if not ((area > self.area_threshold) and (length > self.lengh_threshold)):
                continue
            if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
                continue
            
            x, y, w, h = cv2.boundingRect(cont)
            center = (x + int(w/2), y + int(h/2))
            # _, width, _ = bev.shape
            _, width = bev.shape
            print("detected boundingRect center: %d", center)

            if (200 <= center[0] <= (width - 200)) and (w > 400) & (h < 80):
                cv2.rectangle(bev, (x, y), (x + w, y + h), green, 2)
                detected = True
                print("detected stopline")


        cv2.imshow('stopline', bev)
        key = cv2.waitKey(self.frameRate)
        return detected