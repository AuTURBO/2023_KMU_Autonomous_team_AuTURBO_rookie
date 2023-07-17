#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2
import numpy as np
import time

from warper import Warper

class StopDetector:
    def __init__(self):
        self.cnt = 0
        self.previous_time = time.time() + 5
        self.previous_time2 = time.time() + 5
        self.lower_yellow = (20, 100, 100)
        self.upper_yellow = (40, 255, 255)

    def check_time(self):
        if time.time() < self.previous_time + 10:
            return False
        return True

    def check_time2(self):
        if time.time() < self.previous_time2 + 10:
            return False
        return True

    def check_yellow_line(self, img):
        nonzero = self.check_many_lines(img)

        if 1700 < nonzero:
            self.on_detected_stopline()
            self.previous_time = time.time()
            return True

        return False

    def on_detected_stopline(self):
        # print('STOP LINE DETECTED!!!')
        self.previous_time = time.time()
        self.cnt += 1


    def on_detected_crosswallk(self):
        # print('CROSS WALK DETECTED!!!')
        self.previous_time2 = time.time()



    def check_many_lines(self, img):
        out_img = np.copy(img)

        # stop_roi = cv2.cvtColor(out_img[390:420, 140:550], cv2.COLOR_BGR2HSV)
        stop_roi = out_img[370:410, 140:550]
        # check_roi = cv2.cvtColor(stop_roi, cv2.COLOR_BGR2HSV)
        check_roi = np.dstack((stop_roi, stop_roi, stop_roi))
        cv2.rectangle(check_roi, (140, 390), (550, 420), (210, 100, 55), 2)

        rho = 1
        theta = np.pi / 180
        threshold = 10
        minLineLength = 5
        maxLineGap = 5
        rightlines = cv2.HoughLinesP(stop_roi, rho, theta, threshold, minLineLength, maxLineGap)

        color = [255, 0, 0]
        thickness = 2

        grads = []
	
        # if np.all(rightlines) == None:
        #     return 0, 0
        #
        # for line in rightlines:
        #     for x1, y1, x2, y2 in line:
        #         if ((x2 - x1) != 0):
        #             gradiant = (y2 - y1) / (x2 - x1)
        #             grads.append(gradiant)
        #             cv2.line(check_roi, (x1, y1), (x2, y2), color, thickness)

        # print(len(grads))

        return np.count_nonzero(stop_roi)

    def check_crocss_walk(self, warp_img):
        nonzero = self.check_many_lines(warp_img)

        if nonzero > 1350 and self.check_time ():
            self.on_detected_crosswallk()
            self.previous_time = time.time()
            return True

        return False


if __name__ == '__main__':
    stop_counter = StopDetector()
    warper = None

    cap = cv2.VideoCapture('../capture/origin18654.avi')
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        if warper == None:
            warper = Warper(frame)

        #  stop_counter.check_yellow_line(frame)
        warp_img = warper.warp(frame)
        stop_counter.check_crocss_walk(frame, warp_img)


        # cv2.imshow('frame', detect_img)
        if cv2.waitKey(0) == 27:
            break

