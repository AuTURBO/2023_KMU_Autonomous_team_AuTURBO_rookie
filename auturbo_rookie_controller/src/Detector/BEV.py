#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np

import rospy

from Detector.utils import undistort

def threshold_binary(img, lane_bin_th, method, thresholding_type=cv2.THRESH_BINARY, window_name="threshold", show=False):
    if method == "adaptive":
        lane = cv2.adaptiveThreshold(img, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,9)
    elif method == "otsu":
        _, lane = cv2.threshold(img, 0, 255, thresholding_type+cv2.THRESH_OTSU)
    elif method == "basic":
        _, lane = cv2.threshold(img, lane_bin_th, 255, thresholding_type)

    if show == True:
        cv2.imshow(window_name, lane)
    return lane
class BEV(object):
    '''
    Calibrates camera images to remove distortion and transforms to bird-eye-view image
    ''' 
    def __init__(self, roi_height, roi_width):
        self.roi_height = roi_height  # ROI 높이 설정
        self.roi_width = roi_width  # ROI 폭 설정

        self.lane_bin_th = 120

    def __call__(self, frame):
        
        prev_target = 320
        frameRate = 11 #33

        frame = undistort.undistort_func(frame)
        #cv2.imshow("Undistort", frame)

        gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
        #cv2.imshow("gblur_img", gblur_img)

        gray = cv2.cvtColor(gblur_img, cv2.COLOR_BGR2GRAY)
        adaptive_binary = threshold_binary(gray, self.lane_bin_th, "adaptive", window_name="adaptive_binary", show=False)
        #cv2.imshow("adaptive_binary", adaptive_binary)

        warped_img = self.warp_perspect(frame)
        return warped_img

    # =============================================
    # Bird Eye View 변환 함수
    # src에서 dst로 변환하는 변환행렬을 구하고,
    # 이를 이용해 Bird Eye View를 생성한다.
    # =============================================

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

    # def __init__(self):

    #     # calibration config
    #     self.img_size = (640, 480)
    #     self.warp_img_w, self.warp_img_h, self.warp_img_mid = 650, 120, 60

    #     self.mtx = np.array([[363.090103, 0.000000, 313.080058],
    #                          [0.000000, 364.868860, 252.739984],
    #                          [0.000000, 0.000000, 1.000000]])
    #     self.dist = np.array([-0.334146, 0.099765, -0.000050, 0.001451, 0.000000])
    #     self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, self.img_size, 1, self.img_size)

    #     # perspective config
    #     warpx_mid, warpx_margin_hi, warpx_margin_lo, warpy_hi, warpy_lo, tilt = 320, 200, 319, 325, 375, -5
    #     self.warp_src  = np.array([[warpx_mid+tilt-warpx_margin_hi, warpy_hi], [warpx_mid+tilt+warpx_margin_hi, warpy_hi], 
    #                                [warpx_mid-warpx_margin_lo,  warpy_lo], [warpx_mid+warpx_margin_lo, warpy_lo]], dtype=np.float32)
    #     self.warp_dist = np.array([[100, 0], [649-100, 0],
    #                                [100, 119], [649-100, 119]], dtype=np.float32)
    #     self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dist)
    
    # def to_calibrated(self, img, show=False):
    #     img = cv2.undistort(img, self.mtx, self.dist, None, self.cal_mtx)
    #     if show:
    #         cv2.imshow('calibrated', img)
    #     return img

    # def to_perspective(self, img, show=False):
    #     img = cv2.warpPerspective(img, self.M, (self.warp_img_w, self.warp_img_h), flags=cv2.INTER_LINEAR)
    #     if show:
    #         cv2.imshow('bird-eye-view', img)
    #     return img

    # def __call__(self, img, show_calibrated=False, show=False):
    #     '''
    #     return bird-eye-view image of an input image
    #     '''
    #     img = self.to_calibrated(img, show=show_calibrated)
    #     img = self.to_perspective(img, show=show)
    #     return img