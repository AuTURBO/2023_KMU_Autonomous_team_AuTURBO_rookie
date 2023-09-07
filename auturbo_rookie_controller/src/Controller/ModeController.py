#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ModeController(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    # def __init__(self, yaw0, timer):
    def __init__(self, timer, yaw0, mode, cnt=0):
        # 첫 시작은 긴 직진 모드
        self.mode = mode
        self.timer = timer
        self.error_list = []
        self.error_threshold = 10
        
        self.long_flag = 0
        self.realcurve_flag = 0
        self.realcurve_cnt = int(cnt)

        self.yaw0 = yaw0

    def set_mode(self, mode):
        self.mode = mode

    def set_yaw0(self, yaw0):        
        self.yaw0 = yaw0

    def get_mode(self):
        return self.mode

    def __call__(self, angle_error, ranges, angle_increment, yaw):
        '''
        updates and returns current mode
        '''
        ranges = np.array(ranges)
            
        # 라이다 데이터의 1/4 구간과 3/4 구간은 0으로 설정하여 로봇의 후방을 제외합니다.
        ranges[:len(ranges)//4] = 0.0
        ranges[3*len(ranges)//4:] = 0.0
            
        # 각도 값들을 계산합니다.
        deg = np.arange(len(ranges)) * angle_increment - 252 * angle_increment

        # 장애물로 판단할 조건을 마스킹하여 필터링합니다.
        # 거리값에 따른 필터링 조건을 설정합니다.
        mask = (np.abs(ranges * np.sin(deg)) < 20) & (8 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 20)
        # 필터링 조건에 따라 데이터를 필터링합니다.
        filtered = np.where(mask, ranges, 0.0)
        # 필터링된 데이터 중 0이 아닌 값의 인덱스를 찾습니다.
        nz = np.nonzero(filtered)[0]
        # print(nz)
        
        if len(nz) > 5:
            self.long_state = 1 
        else:
            self.long_state = 0
        # 긴 직진 
        error_mean = 0
        self.error_list.append(abs(angle_error))
        if len(self.error_list) > 13:
            error_mean = sum(self.error_list) / len(self.error_list)
            self.error_list.pop(0)
        
        # imu 
        degree0 = np.rad2deg(self.yaw0)
        degree = np.rad2deg(yaw)
        # print("yaw0: {0}, yaw: {1}".format(degree0, degree))

        diff_degree = abs(degree - degree0)
        if diff_degree > 180:
            diff_degree = 360 - diff_degree
        # print("diff_degree: ", diff_degree)
    
        if self.mode == 'zgzg' and self.long_state == 1 and self.timer() > 8:
            self.realcurve_flag = 0
            self.realcurve_cnt = 0
            self.mode = 'long straight'
            print(self.timer())
            self.timer.update()
            print('zgzg -> long straight')

        # long_state : 라이다가 적정거리만큼 인지 하였는가 
        # abs(error_mean) : 편균 조향각이 어느 정도인가 
        # timer() : 타이머가 얼마나 지났는가 즉, 모드가 변경되고 얼마나 지났는가
        if self.mode == 'long straight': 
            if abs(error_mean) > self.error_threshold - 3 and self.long_state == 0 and self.timer() > 5:
                if (self.realcurve_cnt >= 2):
                    self.mode = 'zgzg'
                    print(self.timer())
                    self.timer.update()
                    print('long straight -> zgzg')
                else:
                    self.set_yaw0(yaw)
                    self.mode = 'curve'
                    print(self.timer())
                    self.timer.update()
                    print('long straight -> curve')

        # 커브
        elif self.mode == 'curve':
            if self.realcurve_flag == 0 and diff_degree > 47 and self.timer() > 2:
                self.realcurve_flag = 1
                self.realcurve_cnt = self.realcurve_cnt + 1
                print('realcurve: ', self.realcurve_cnt)

            if abs(error_mean) < self.error_threshold and self.long_state == 1 and self.timer() > 2:
                self.realcurve_flag = 0
                self.mode = 'long straight'
                print(self.timer())
                self.timer.update()
                print('curve -> long straight')
         
        return self.mode