#!/usr/bin/env python

#rotation 1일 때 : 왼쪽 장애물이 오른쪽 장애물 보다 가까이 있음
#rotation -1일 때 : 오른쪽 장애물이 왼쪽 장애물 보다 가까이 있음
# 초기값 : 0

# 시간을 쟤서 5초 이상 시간이 지나면 다음 모드로 넘어가게 하기 
import numpy as np


class RubberconController(object):
    def __init__(self, timer):
        
        #라바콘 회피 주행 모드 출입, 탈출 워크플로우
        #1 .state_flag 가 0 and 장애물이 인식되지 않을 때(action_flag = 0), pursuit 하면서, 라인 따라가기
        #2. state_flag 가 0이면서 and 장애물이 인식 될 때(action_flag = 1),   rubber avoidance 하면서 주행하기 -> flag를 1로 수정
        #3. state_flag 가 1일 때는 장애물이 인식되고 있을 때(action_flag = 1), rubber avoidance 하면서 주행하기
        #4. state_flag 가 1인상태에서 장애물이 1.n초 동안 인식되지 않으면  pursuit로 돌아오기
        self.state_flag = 0
        self.action_flag = 0
        self.timer = timer
        self.angle = 0
        self.speed = 0
        self.error = 0 
        # 비례 제어 게인
        self.k = 30.0  
        #라이다 /scan 토픽 값을 이용하여 오차 계산
        
        
        self.kp = 25
        self.rotation = 0
        self.prev_rotation = 0
        self.right_quarter_min_distance = 0
        self.left_quarter_min_distance = 0

    def __call__(self, ranges, angle_increment):
        
        right_filtered = 0
        left_filtered = 0
        ranges = np.array(ranges)
        
        # 라이다 데이터의 1/4 구간과 3/4 구간은 0으로 설정하여 로봇의 전방 및 후방을 제외합니다.
        ranges[:len(ranges)//4] = 0.0
        ranges[3*len(ranges)//4:] = 0.0

        # 각도 값들을 계산합니다.
        # print("angle_increment: ", angle_increment)
        deg = np.arange(len(ranges)) * angle_increment - 252 * angle_increment

        # 장애물로 판단할 조건을 마스킹하여 필터링합니다.
        # 거리값에 따른 필터링 조건을 설정합니다.
        mask = (np.abs(ranges * np.sin(deg)) < 0.8) & (0.1 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 0.8)

        filtered = np.where(mask, ranges, 0.0)

        # 필터링된 데이터 중 0이 아닌 값의 인덱스를 찾습니다.
        nz = np.nonzero(filtered)[0]

            
        if len(nz) > self.kp :   #2. flag가 0이면서 and 장애물이 인식 될 때,   rubber avoidance 하면서 주행하기 -> flag를 1로 수정
            self.action_flag = 1
            self.state_flag = 1
            
            # print("nz: ", nz)
            # 만약 필터링된 데이터의 개수가 5개 이상이면, 어느 방향으로 피해야 할지 결정합니다.
            #  1. 1/4 구간에 장애물이 있으면, 오차값을 더해주고, 3/4 구간에 장애물이 있으면, 오차값을 빼줍니다.
            
            if np.median(nz) > len(ranges) // 2 and np.median(nz) < len(ranges):
                #self.rotation = -1
                
                left_filtered = filtered[int(np.median(nz))]
                #print("왼쪽", filtered[int(np.median(nz))])
                #self.error += 0.5 - filtered[int(np.median(nz))]
            elif np.median(nz) > len(ranges) // 2 - 5 and np.median(nz) < len(ranges) // 2 + 5:
                self.error = 0
            elif np.median(nz) < len(ranges) // 2 and np.median(nz) > 0:
                #self.rotation = 1
                
                right_filtered = filtered[int(np.median(nz))]
                #self.error -= 0.5 - filtered[int(np.median(nz))]
                #print("오른쪽", filtered[int(np.median(nz))])
            

            #print("왼쪽 : ", left_filtered)
            #print("오른쪽 : ", right_filtered)
        elif len(nz) < 10 :    #1 .flag가 0 and 장애물이 인식되지 않을 때, pursuit 하면서, 라인 따라가기
            self.error = 0
            self.action_flag = 0 #action_flag == 0 이면, pursuit 하기


        if left_filtered > right_filtered:
            self.rotation = -1
        elif left_filtered < right_filtered:
            self.rotation = 1



        self.error += 0.5 - right_filtered
        self.error -= 0.5 - left_filtered
        

        # 태그 설정
        if self.rotation != self.prev_rotation:
            self.error = 0
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
        #print("rotation : {}".format(self.rotation))
        #print("angle_steer: {} ".format(int(steer)))
        print("state_Flag : {}, self.action_flag : {}".format(self.state_flag, self.action_flag))
        
        return  int(steer), 3, self.state_flag, self.action_flag