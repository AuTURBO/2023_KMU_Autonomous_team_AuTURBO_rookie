#!/usr/bin/env python

#rotation 1일 때 : 왼쪽 장애물이 오른쪽 장애물 보다 가까이 있음
#rotation -1일 때 : 오른쪽 장애물이 왼쪽 장애물 보다 가까이 있음
# 초기값 : 0

# 시간을 쟤서 5초 이상 시간이 지나면 다음 모드로 넘어가게 하기 
import numpy as np


class RubberconController(object):
    def __init__(self, timer):
        self.timer = timer
        self.angle = 0
        self.speed = 0
        self.error = 0 
        # 비례 제어 게인
        self.k = 1.2 
        #라이다 /scan 토픽 값을 이용하여 오차 계산

 
    def __call__(self, ranges, angle_increment, obstacle_cnt):
        
        # if self.timer() > 0.3 and self.error != 0: 
        ranges = np.array(ranges)
        
        # 라이다 데이터의 1/4 구간과 3/4 구간은 0으로 설정하여 로봇의 전방 및 후방을 제외합니다.
        ranges[:len(ranges)//4] = 0.0
        ranges[3*len(ranges)//4:] = 0.0

        # 각도 값들을 계산합니다.
        # print("angle_increment: ", angle_increment)
        deg = np.arange(len(ranges)) * angle_increment - 252 * angle_increment

        # 장애물로 판단할 조건을 마스킹하여 필터링합니다.
        # 거리값에 따른 필터링 조건을 설정합니다.
        mask = (np.abs(ranges * np.sin(deg)) < 0.6) & (0.1 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 0.6)

        filtered = np.where(mask, ranges, 0.0)

        # 필터링된 데이터 중 0이 아닌 값의 인덱스를 찾습니다.
        nz = np.nonzero(filtered)[0]

        if len(nz) > 10:
            for i in range(len(nz)):
                if nz[i] < len(ranges) // 2 and nz[i] > 0:
                    self.error -= abs(filtered[nz[i]]-0.6)
                elif nz[i] > len(ranges) // 2 and nz[i] < len(ranges) - 1:
                    self.error += abs(filtered[nz[i]]-0.6)
        else:
            self.error = 0
        
        # 장애물 감지 
        # 타이머에 충족하고, 장애물이 감지 되었을 때 
        if obstacle_cnt <= 300 and self.error != 0: 
    
            if self.error > 30:
                self.error = 30
            elif self.error < -30:
                self.error = -30
            steer = self.error * self.k
            if steer > 30:
                steer = 30
            elif steer < -30:   
                steer = -30
            return  int(steer), 3, 1
        
        # 장애물이 감지 되었었고, 타이머 안에 장애물이 감지되지 않았을 경우 
        # 0 : 장애물을 감지하기 전 
        # 1 : 장애물 인식중 
        # 2 : 장애물 인식 후 장애물 감지 없음 
        # elif obstacle_cnt <= 300 and  self.error == 0:
        #     print("라바콘 모드 종료")
        #     return 0, 0, 2
        else:
            return 0, 0, 1