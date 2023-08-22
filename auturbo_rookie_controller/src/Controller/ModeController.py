#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ModeController(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''
    # == 기본 주행 ===== #
    # 긴 구간 직진 'long straight' : self.pursuit,
    # 짧은 구간 직진 'short straight' : self.pursuit,
    # 각 모서리의 커브 구간'curve': self.pursuit,
    
    # == 미션 1 평행주차 ===== #
    # 주차 공간 찾기 'findparking': self.findparking,
    # 가로주차  'parallelparking': self.parallelpark,
    # AR 정밀주차 'arparking': self.arparking,
    # 차량 정지 'poweroff' : self.poweroff,
    
    # == 미션 2 ar curve 주행 == # -- 08.08 테스트
    # AR Curve 'ar_curve': self.ar_curve, 
    
    # == 미션 3 객체 인식 후 주차 == # -- 07.31 테스트
    # Object detection 'object': self.object,
    # vertical parking 'verticalparking': self.verticalparking,
    
    # == 미션 4 장애물 회피 == # -- 07.31 테스트
    # 장애물 회피 'obstacle': self.obstacle,
    
    # == 미션 5 정지선 정지 == # -- 08.08 테스트
    # stopline 'stopline': self.stopline,
    
    # == 미션 6 라바콘 주행 == # -- 07.31 테스트
    # 라바콘 주행 'rubbercon': self.rubbercon

    # def __init__(self, yaw0, timer):
    def __init__(self, timer):
        # 첫 시작은 긴 직진 모드
        self.mode = 'long straight'
        self.timer = timer
        self.error_list = []


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

    def __call__(self, angle_error, ranges, angle_increment):
        '''
        updates and returns current mode
        '''
        ranges = np.array(ranges)
            
        # 라이다 데이터의 1/4 구간과 3/4 구간은 0으로 설정하여 로봇의 전방 및 후방을 제외합니다.
        ranges[:len(ranges)//4] = 0.0
        ranges[3*len(ranges)//4:] = 0.0
            
        # 각도 값들을 계산합니다.
        # print("angle_increment: ", angle_increment)
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
        self.error_list.append(angle_error)
        if len(self.error_list) > 10:
            error_mean = sum(self.error_list) / len(self.error_list)
            self.error_list.pop(0)

        # long_state : 라이다가 적정거리만큼 인지 하였는가 
        # abs(error_mean) : 편균 조향각이 어느 정도인가 
        # timer() : 타이머가 얼마나 지났는가 즉, 모드가 변경되고 얼마나 지났는가

        if self.mode == 'long straight': 
            if abs(error_mean) > 10 and self.long_state == 0 and self.timer() > 5:
                self.mode = 'curve'
                print('long straight -> curve')
                self.timer.update()
        # 커브
        elif self.mode == 'curve':
            if abs(error_mean) < 10 and self.long_state == 0 and self.timer() > 2:
                self.mode = 'short straight'
                self.timer.update()
                print('curve -> short straight')
            elif abs(error_mean) > 10 and self.long_state == 1:
                self.mode = 'long straight'
                self.timer.update()
                print('curve -> long straight')
        # 짧은 직진
        elif self.mode == 'short straight' and self.long_state == 0 and self.timer() > 3:
            if abs(error_mean) > 10:
                self.mode = 'curve'
                self.timer.update()
            print('short straight -> curve')

        return self.mode