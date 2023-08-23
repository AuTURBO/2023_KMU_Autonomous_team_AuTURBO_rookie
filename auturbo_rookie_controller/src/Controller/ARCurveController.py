#!/usr/bin/env python

#rotation 1일 때 : 왼쪽 장애물이 오른쪽 장애물 보다 가까이 있음
#rotation -1일 때 : 오른쪽 장애물이 왼쪽 장애물 보다 가까이 있음
# 초기값 : 0

# 시간을 쟤서 5초 이상 시간이 지나면 다음 모드로 넘어가게 하기 
import numpy as np


class ARCurveController(object):
    def __init__(self, timer):



        self.ar_curve_state = 0
        self.ar_curve_action = 0
        self.obstacle_count = 0

        self.timer = timer
        self.angle = 0
        self.speed = 0
        self.error = 0 
        # 비례 제어 게인
        self.k = 50.0  
        #라이다 /scan 토픽 값을 이용하여 오차 계산

        self.rotation = 0
        self.prev_rotation = 0

    def __call__(self, ranges, angle_increment, curve_state, curve_action) :

        self.ar_curve_state = curve_state
        self.ar_curve_action = curve_action
        min_filtered = 0
        right_filtered = 0
        left_filtered = 0
        ranges = np.array(ranges)
        
        # 라이다 데이터의 1/4 구간과 3/4 구간은 0으로 설정하여 로봇의 전방 및 후방을 제외합니다.
        ranges[:len(ranges)//4] = 0.0
        ranges[3*len(ranges)//4:] = 0.0

        # 각도 값들을 계산합니다.
        # print("angle_increment: ", angle_increment)
        deg = np.arange(len(ranges)) * angle_increment - 220 * angle_increment
        # deg = np.arange(len(ranges)) * angle_increment - 252 * angle_increment

        # 장애물로 판단할 조건을 마스킹하여 필터링합니다.
        # 거리값에 따른 필터링 조건을 설정합니다.
        mask = (np.abs(ranges * np.sin(deg)) < 0.75) & (0.1 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 0.75)

        filtered = np.where(mask, ranges, 0.0)

        # 필터링된 데이터 중 0이 아닌 값의 인덱스를 찾습니다.
        nz = np.nonzero(filtered)[0]

        #필터리된 데이터중 최대값을 찾습니다.
            
        if len(nz) > 5:
            self.ar_curve_state  = 1
            self.ar_curve_action  = 1
            print("obstacle_count: ", self.obstacle_count)
            # # print("nz: ", nz)
            # # 만약 필터링된 데이터의 개수가 5개 이상이면, 어느 방향으로 피해야 할지 결정합니다.
            # #  1. 1/4 구간에 장애물이 있으면, 오차값을 더해주고, 3/4 구간에 장애물이 있으면, 오차값을 빼줍니다.
            # if np.median(nz) > len(ranges) // 2 and np.median(nz) < len(ranges):
            #     #self.rotation = -1
                
            #     left_filtered = filtered[int(np.median(nz))]
            #     #print("왼쪽", filtered[int(np.median(nz))])
            #     #self.error += 0.5 - filtered[int(np.median(nz))]
            # elif np.median(nz) > len(ranges) // 2 - 5 and np.median(nz) < len(ranges) // 2 + 5:
            #     self.error = 0
            #     steer = 0
            # elif np.median(nz) < len(ranges) // 2 and np.median(nz) > 0:
            #     #self.rotation = 1
                
            #     right_filtered = filtered[int(np.median(nz))]
            #     #self.error -= 0.5 - filtered[int(np.median(nz))]
            #     #print("오른쪽", filtered[int(np.median(nz))])


            min_filtered = filtered[int(np.median(nz))]
            if self.obstacle_count < 2 :
                th = 0.45  #45
            else :
                th = 0.55 #55
            if min_filtered > th :
                self.rotation = 1
            elif min_filtered < th:
                self.rotation = -1

            if self.rotation == -1:
                self.error -= th - min_filtered
                # self.error -= th - right_filtered

            elif self.rotation == 1:
                self.error += th - min_filtered
                # self.error += th - left_filtered
            
            # print("min_filtered: ", min_filtered)
        else:
            self.error = 0
            self.ar_curve_action  = 0





        

        # 태그 설정
        if self.rotation != self.prev_rotation and self.rotation == 1:
            self.error = 15
            self.obstacle_count +=1

        elif self.rotation != self.prev_rotation and self.rotation == -1:
            self.error = -15
            self.obstacle_count +=1
        self.prev_rotation = self.rotation

        if self.error == 0:
            steer = 0
            self.error = 0
        else:
            steer = self.error * self.k

    


        if steer > 30:
            steer = 30
        elif steer < -30:   
            steer = -30

        #조향각 출력하기
        
        # print("rotation : {}".format(self.rotation))
        # print("angle_steer: {} ".format(int(steer)))
        # print("obstacle_count: ", self.obstacle_count)

        if self.obstacle_count >= 8 and self.ar_curve_action  == 0:
            print("수직주차 모드 변경")
            return  0, 0, 1 , 0    
        return  int(steer), 3, self.ar_curve_state , self.ar_curve_action    