#!/usr/bin/env python

#rotation 1일 때 : 왼쪽 장애물이 오른쪽 장애물 보다 가까이 있음
#rotation -1일 때 : 오른쪽 장애물이 왼쪽 장애물 보다 가까이 있음
# 초기값 : 0

# 시간을 쟤서 5초 이상 시간이 지나면 다음 모드로 넘어가게 하기 

class RubberconController(object):
    def __init__(self):
        # self.timer = timer
        self.angle = 0
        self.speed = 0
        self.error = 0 
        # 비례 제어 게인
        self.k = 30.0  
        #라이다 /scan 토픽 값을 이용하여 오차 계산

        self.rotation = 0
        self.prev_rotation = 0

    def __call__(self, ranges):
        
        num_ranges = len(ranges)  
        quarter = num_ranges // 4
        # 오른쪽의 장애물의 최소 거리 계산
        right_quarter_ranges = [range_val for range_val in ranges[:quarter] if range_val > 0]
        if len(right_quarter_ranges) > 0:
            right_quarter_min_distance = min(right_quarter_ranges)
            print("right min distnace: {} cm".format(right_quarter_min_distance * 100))
        else:
            print("right, no obstacle.")
            right_quarter_min_distance = 0

        # 왼쪽의 최소 거리 계산
        left_quarter_ranges = [range_val for range_val in ranges[2*quarter//4:] if range_val > 0]
        if len(left_quarter_ranges) > 0:
            left_quarter_min_distance = min(left_quarter_ranges)
            print("left min distance: {} cm".format(left_quarter_min_distance * 100))
        else:
            print("left, no obstacle.")
            left_quarter_min_distance = 0

        
        if left_quarter_min_distance > right_quarter_min_distance:
            self.rotation = -1
        else:
            self.rotation = 1

        # 오차 계산
        self.error -= 0.5 - right_quarter_min_distance
        self.error += 0.5 - left_quarter_min_distance

        # 태그 설정
        if self.rotation != self.prev_rotation:
            self.error = 0
        self.prev_rotation = self.rotation

        # 장애물이 없을 때 0 조향각으로 초기화
        if len(right_quarter_ranges) == 0 and len(left_quarter_ranges) == 0:
            steer = 0
            self.error = 0
        else: # 장애물이 있을 경우 게인 값 비례 제어
            steer = self.error * self.k

        if steer > 30:
            steer = 30
        elif steer < -30:   
            steer = -30

        return -1 * int(steer), int(self.speed) 
            
    