#!/usr/bin/env python
#-*- coding:utf-8 -*-

import cv2
import numpy as np


class Warper:
    def __init__(self, image):


        height = image.shape[0]
        width = image.shape[1]

        src = np.float32([
            [0, 320],  # 좌상
            [0, 410],  # 좌하
            [width, 320],  # 우상
            [width, 410],  # 우하
        ])
        dst = np.float32([
            [0, 0],
            [200, height],
            [width, 0],
            [width - 200, height],
        ])


        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def warp(self, img):
        return cv2.warpPerspective(
            img,
            self.M,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPerspective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )

