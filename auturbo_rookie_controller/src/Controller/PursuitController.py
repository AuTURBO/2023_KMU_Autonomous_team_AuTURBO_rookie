#! /usr/bin/env python
# -*- coding:utf-8 -*-
import math
# from Timer import Timer
class PurePursuitController(object):
    def __init__(self, timer):
        self.WB = 0.24
        self.diff_angle = 0
        self.speed = 10
        self.timer = timer

        self.Lf = {
            'long straight': 0.4,
            'short straight': 0.4,
            'curve': 0.4,
            'zgzg': 0.17,

            'findparallelparking': 0.20,
            'findverticalparking': 0.20,
            'arparking' : 0.20,
            'verticalparking' : 0.20,
            'stopline': 0.2,
            'obstacle': 0.2,
            'parallelparking': 0.2,
            'object': 0.2,
            'ar_curve': 0.2,
            'rubbercon': 0.2
        }
        self.target_speed = {
            'long straight': 15,
            'short straight': 15,
            'curve': 15,
            'zgzg': 4,

            'findparallelparking': 4,
            'findverticalparking': 4,
            'arparking' : 3,
            'verticalparking' : 4,
            'stopline': 3,
            'obstacle': 3,
            'parallelparking': 3,
            'ar_curve': 3,
            'object': 3,
            'rubbercon' : 3
        }
        self.acc = {
            'long straight': 2,
            'short straight': -0.5,
            'curve': None,
            'zgzg': -2,

            'findparallelparking': 0.5,
            'findverticalparking': 0.5,
            'arparking' : 0.5,
            'verticalparking' : 0.5,
            'stopline': 0.5,
            'obstacle': 0.5,
            'parallelparking': 0.5,
            'ar_curve': 0.5,
            'object': 0.5,
            'rubbercon' : 0.5
        }

        self.delay = {
            'long straight': 1,
            'short straight': 0.5,
            'curve': -1,
            'zgzg': -1,

            'findparallelparking': 0.5,
            'findverticalparking': 0.5,
            'arparking' : 0.5,
            'verticalparking' : 0.5,
            'stopline': 0.5,
            'obstacle': 0.5,
            'parallelparking': 0.5,
            'ar_curve': 0.5,
            'object': 0.5,
            'rubbercon' : 0.5
        }


    def __call__(self, target, mode):
        # # 가속도에 의한 스피드제어... 그냥 바로 적용
        if self.timer() > self.delay[mode]:
            if self.acc[mode] is None:
                self.speed = self.target_speed[mode]
            elif self.acc[mode] > 0:
                self.speed = min(self.speed+self.acc[mode], self.target_speed[mode])
            else:
                self.speed = max(self.speed+self.acc[mode], self.target_speed[mode])

        else:
             self.speed = self.target_speed[mode]

        current_angle = target
        # 현재 조향각인 current_angle과 목표값(0으로 가정)을 이용하여 알고리즘을 적용합니다.
        # 다음 줄을 알고리즘에 맞게 수정해주세요.
        target_angle = 10  # -5  # 여기에 Pure Pursuit 알고리즘을 적용하여 목표 조향각을 계산합니다.

        # 현재 조향각과 목표 조향각과의 차이를 계산합니다.
        self.diff_angle = target_angle - current_angle

        # degree to radian
        self.diff_angle = self.diff_angle * math.pi / 180

        delta = math.atan2(2.0 * self.WB * math.sin(self.diff_angle), self.Lf[mode])

        # 계산된 조향각을 degree to radian
        delta = -1 * delta * 180 / math.pi
        return int(delta), int(self.target_speed[mode])

         
