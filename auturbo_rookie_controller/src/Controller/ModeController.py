#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ModeController(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    def __init__(self, yaw0, timer):
        # 첫 시작은 긴 직진 모드
        self.mode = 'stopline'
        self.timer = timer
        self.yaw0 = yaw0
        self.lap = 0
        self.lap_target = 3

    def set_mode(self, mode):
        self.mode = mode

    def set_yaw0(self, yaw0):


        
        self.yaw0 = yaw0

    def get_mode(self):
        return self.mode

    # 구간인식 flowchart
    # 1. 직진 모드(디폴트) -- 구현됨 
    # 2. 직진 모드로 가다 평행 주차 모드로 변경 후 평행주차 -- 구현됨
    # 아래부턴 구현해야댐 
    # 4. 커브 모드의 lap == 1일때 ar 커브 모드로 변경 하면 되지 않을까?
    # 5. ar 커브모드가 끝나면 직진 모드로 변경 - 이때 객체인식 주차 모드로 변경 

    # 6. 객체인식 주차 모드가 끝나면 커브 모드로 면경 후 (lap == 2일때 장애물 인식 모드로 변경) -- 각도에 따른 조건문으로 구현이 되어있음 둘 중 잘되는걸로 
    # 7. 장애물 인식 모드가 끝나면 횡단보도 인식 모드로 변경 
    # 8. 횡단보도 인식 모드가 끝나면 커브 모드로 변경 후 lap == 3일때 정지선 라바콘 주행 모드로 변경
    # 9. 정지선 라바콘 주행 모드가 끝나면 짧은 직진 모드로 변경 후 lap == 4일때 직진 모드로 변경

    def __call__(self, yaw):
        '''
        updates and returns current mode
        '''
        # 각도 오차값 계산
        diff_yaw = abs(yaw - self.yaw0)
        # 만약 각도 오차값이 180도를 넘어가면 360도에서 빼주어 양수로 만들어준다.
        # ex) 360 - 190 = 170
        if diff_yaw > np.pi: 
            diff_yaw = 2*np.pi - diff_yaw
        # 직진 모드고, 90도 +- 0.1 radian 범위에 들어오면 짧은 직진 모드로 변경 85 ~ 95 deg
        if self.mode == 'long straight' and  np.pi/2.0 - 0.1 < diff_yaw < np.pi/2.0 + 0.1:
            self.mode = 'short straight'
            self.timer.update()
        # 짧은 직진 모드고, 180도 +- 0.15 radian 170 ~ 190 deg 범위에 들어오면 장애물 모드로 변경
        elif self.mode == 'short straight' and  np.pi - 0.15 < diff_yaw < np.pi + 0.15:
            print('detecting obstacle...')
            self.timer.update()
            self.mode = 'obstacle'
       
        # 각 모서리의 커브를 지날 때마다 lap을 1씩 증가시킴
        # lap이 lap_target보다 작으면 직진 모드로 변경
        # lap이 lap_target보다 크면 주차장 찾기 모드로 변경
        # 왜냐 1바퀴를 돌고나면 주차장을 다시 찾아야 하기 때문
        # 3deg +- 0.05 radian 범위에 들어오면 커브 모드로 변경
        elif self.mode == 'curve' and  diff_yaw < 0.05:
            self.lap += 1
            print('커브 모드가 정상적으로 작동중임 ... {}'.format(self.lap))
            
            self.timer.update()
            # ar 커브 모드
            if self.lap == 1:
                print('detecting ar curve...')
                self.mode = 'ar curve'
            # 장애물 회피 모드
            elif self.lap == 2:
                print('detecting obstacle...')
                self.mode = 'obstacle'
            # 라바콘 주행 모드
            elif self.lap == 3:
                print('lavacon mode...')
                self.mode = 'lavacon'
        return self.mode