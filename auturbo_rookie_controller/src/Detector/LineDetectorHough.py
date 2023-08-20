#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np
from collections import deque
from BEV import BEV

# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

class LaneDetector():
    '''
    Detects left, middle, right lane from an image and calculate angle of the lane.
    Uses canny, houghlinesP for detecting possible lane candidates.
    Calculates best fitted lane position and predicted lane position from previous result.
    '''

    def __init__(self):

        self.bev = BEV()

        # canny params
        self.canny_low, self.canny_high = 100, 120

        # HoughLineP params
        self.hough_threshold, self.min_length, self.min_gap = 10, 50, 10

        # initial state
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([90.0, 320., 568.])

        # filtering params:
        self.angle_tolerance = np.radians(30)
        self.cluster_threshold = 25

    def to_canny(self, img, show=False):
        img = cv2.GaussianBlur(img, (7,7), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if show:
            cv2.imshow('canny', img)
        return img

    def hough(self, img, show=False):
        lines = cv2.HoughLinesP(img, 1, np.pi/180, self.hough_threshold, self.min_gap, self.min_length)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), red, 2)
            cv2.imshow('hough', hough_img)
        return lines

    def filter(self, lines, show=True):
        '''
        filter lines that are close to previous angle and calculate its positions
        '''
        thetas, positions = [], []
        if show:
            filter_img = np.zeros((self.bev.warp_img_h, self.bev.warp_img_w, 3))

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if y1 == y2:
                    continue
                flag = 1 if y1-y2 > 0 else -1
                theta = np.arctan2(flag * (x2-x1), flag * 0.9* (y1-y2))
                if abs(theta - self.angle) < self.angle_tolerance:
                    position = float((x2-x1)*(self.bev.warp_img_mid-y1))/(y2-y1) + x1
                    thetas.append(theta)
                    positions.append(position) 
                    if show:
                        cv2.line(filter_img, (x1, y1), (x2, y2), red, 2)

        self.prev_angle.append(self.angle)
        if thetas:
            self.angle = np.mean(thetas)
        if show:
            cv2.imshow('filtered lines', filter_img)
        return positions

    def get_cluster(self, positions):
        '''
        group positions that are close to each other
        '''
        clusters = []
        for position in positions:
            if 0 <= position < self.bev.warp_img_w:
                for cluster in clusters:
                    if abs(cluster[0] - position) < self.cluster_threshold:
                        cluster.append(position)
                        break
                else:
                    clusters.append([position])
        lane_candidates = [np.mean(cluster) for cluster in clusters]
        return lane_candidates

    def predict_lane(self):
        '''
        predicts lane positions from previous lane positions and angles
        '''
        predicted_lane = self.lane[1] + [-220/max(np.cos(self.angle), 0.75), 0, 240/max(np.cos(self.angle), 0.75)]
        predicted_lane = predicted_lane + (self.angle - np.mean(self.prev_angle))*70
        return predicted_lane

    def update_lane(self, lane_candidates, predicted_lane):
        '''
        calculate lane position using best fitted lane and predicted lane
        '''

        if not lane_candidates:
            self.lane = predicted_lane
            return

        possibles = []

        for lc in lane_candidates:

            idx = np.argmin(abs(self.lane-lc))

            if idx == 0:
                estimated_lane = [lc, lc + 220/max(np.cos(self.angle), 0.75), lc + 460/max(np.cos(self.angle), 0.75)]
                lc2_candidate, lc3_candidate = [], []
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < 50 :
                        lc2_candidate.append(lc2)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < 50 :
                        lc3_candidate.append(lc3)
                if not lc2_candidate:
                    lc2_candidate.append(lc + 220/max(np.cos(self.angle), 0.75))
                if not lc3_candidate:
                    lc3_candidate.append(lc + 460/max(np.cos(self.angle), 0.75))
                for lc2 in lc2_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc, lc2, lc3])

            elif idx == 1:
                estimated_lane = [lc - 220/max(np.cos(self.angle), 0.75), lc, lc + 240/max(np.cos(self.angle), 0.75)]
                lc1_candidate, lc3_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < 50 :
                        lc1_candidate.append(lc1)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < 50 :
                        lc3_candidate.append(lc3)
                if not lc1_candidate:
                    lc1_candidate.append(estimated_lane[0])
                if not lc3_candidate:
                    lc3_candidate.append(estimated_lane[2])
                for lc1 in lc1_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc1, lc, lc3])

            else :
                estimated_lane = [lc - 460/max(np.cos(self.angle), 0.75), lc - 240/max(np.cos(self.angle), 0.75), lc]
                lc1_candidate, lc2_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < 50 :
                        lc1_candidate.append(lc1)
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < 50 :
                        lc2_candidate.append(lc2)
                if not lc1_candidate:
                    lc1_candidate.append(estimated_lane[0])
                if not lc2_candidate:
                    lc2_candidate.append(estimated_lane[1])
                for lc1 in lc1_candidate:
                    for lc2 in lc2_candidate:
                        possibles.append([lc1, lc2, lc])

        possibles = np.array(possibles)
        error = np.sum((possibles-predicted_lane)**2, axis=1)
        best = possibles[np.argmin(error)]
        self.lane = 0.4 * best + 0.6 * predicted_lane

    def mark_lane(self, img, lane=None):
        '''
        mark calculated lane position to an image 
        '''
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lane is None:
            lane = self.lane
        l1, l2, l3 = self.lane
        cv2.circle(img, (int(l1), self.bev.warp_img_mid), 3, red, 5, cv2.FILLED)
        cv2.circle(img, (int(l2), self.bev.warp_img_mid), 3, green, 5, cv2.FILLED)
        cv2.circle(img, (int(l3), self.bev.warp_img_mid), 3, blue, 5, cv2.FILLED)
        cv2.imshow('marked', img)

    def __call__(self, img, target_lane):
        '''
        returns angle and cte of a target lane from an image
        angle : radians
        cte : pixels
        '''
        canny = self.to_canny(img, show=False)
        bev = self.bev(canny, show=False)
        lines = self.hough(bev, show=False)
        positions = self.filter(lines, show=False)
        lane_candidates = self.get_cluster(positions)
        predicted_lane = self.predict_lane()
        self.update_lane(lane_candidates, predicted_lane)
        self.mark_lane(bev)

        if target_lane == 'middle':
            return self.angle, self.lane[1]
        elif target_lane == 'left':
            return self.angle, self.lane[0]*0.75+self.lane[1]*0.25
        else:
            return self.angle, self.lane[2]*0.75+self.lane[1]*0.25