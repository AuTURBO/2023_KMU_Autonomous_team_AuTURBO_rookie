#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np

class BEV(object):
    '''
    Calibrates camera images to remove distortion and transforms to bird-eye-view image
    ''' 

    def __init__(self):

        # calibration config
        self.img_size = (640, 480)
        self.warp_img_w, self.warp_img_h, self.warp_img_mid = 650, 120, 60

        self.mtx = np.array([[363.090103, 0.000000, 313.080058],
                             [0.000000, 364.868860, 252.739984],
                             [0.000000, 0.000000, 1.000000]])
        self.dist = np.array([-0.334146, 0.099765, -0.000050, 0.001451, 0.000000])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, self.img_size, 1, self.img_size)

        # perspective config
        warpx_mid, warpx_margin_hi, warpx_margin_lo, warpy_hi, warpy_lo, tilt = 320, 200, 319, 325, 375, -5
        self.warp_src  = np.array([[warpx_mid+tilt-warpx_margin_hi, warpy_hi], [warpx_mid+tilt+warpx_margin_hi, warpy_hi], 
                                   [warpx_mid-warpx_margin_lo,  warpy_lo], [warpx_mid+warpx_margin_lo, warpy_lo]], dtype=np.float32)
        self.warp_dist = np.array([[100, 0], [649-100, 0],
                                   [100, 119], [649-100, 119]], dtype=np.float32)
        self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dist)
    
    def to_calibrated(self, img, show=False):
        img = cv2.undistort(img, self.mtx, self.dist, None, self.cal_mtx)
        if show:
            cv2.imshow('calibrated', img)
        return img

    def to_perspective(self, img, show=False):
        img = cv2.warpPerspective(img, self.M, (self.warp_img_w, self.warp_img_h), flags=cv2.INTER_LINEAR)
        if show:
            cv2.imshow('bird-eye-view', img)
        return img

    def __call__(self, img, show_calibrated=False, show=False):
        '''
        return bird-eye-view image of an input image
        '''
        img = self.to_calibrated(img, show=show_calibrated)
        img = self.to_perspective(img, show=show)
        return img