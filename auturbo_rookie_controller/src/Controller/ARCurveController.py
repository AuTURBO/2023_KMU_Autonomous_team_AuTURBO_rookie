#!/usr/bin/env python

#rotation 1일 때 : 왼쪽 장애물이 오른쪽 장애물 보다 가까이 있음
#rotation -1일 때 : 오른쪽 장애물이 왼쪽 장애물 보다 가까이 있음
# 초기값 : 0

# 시간을 쟤서 5초 이상 시간이 지나면 다음 모드로 넘어가게 하기 
import numpy as np


class ARCurveController(object):
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


    def __call__(self, ranges, angle_increment):
        
        right_filtered = 0
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
            # print("nz: ", nz)
            # 만약 필터링된 데이터의 개수가 5개 이상이면, 어느 방향으로 피해야 할지 결정합니다.
            #  1. 1/4 구간에 장애물이 있으면, 오차값을 더해주고, 3/4 구간에 장애물이 있으면, 오차값을 빼줍니다.

        
            # if np.median(nz) > len(ranges) // 2 and np.median(nz) < len(ranges):
            #     #self.rotation = -1
                
            #     left_filtered = filtered[int(np.median(nz))]
            #     #print("왼쪽", filtered[int(np.median(nz))])
            #     #self.error += 0.5 - filtered[int(np.median(nz))]
            if np.median(nz) > len(ranges) // 2 - 5 and np.median(nz) < len(ranges) // 2 + 5:
                self.error = 0
            elif np.median(nz) < len(ranges) // 2 and np.median(nz) > 0:
                #self.rotation = 1
                
                right_filtered = filtered[int(np.median(nz))]
                #self.error -= 0.5 - filtered[int(np.median(nz))]
                #print("오른쪽", filtered[int(np.median(nz))])
            


            #print("왼쪽 : ", left_filtered)
            print("오른쪽 : ", right_filtered)
        else:
            self.error = 0
            print("장애물이 없습니다.")




        if right_filtered > 0.3:
            #장애물이 멀면 오른쪽으로
            self.error -= 0.5 - right_filtered
        else:
            #장애물이 가까우면 왼쪽으로
            self.error += 0.5 - right_filtered
        

        # 태그 설정


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
        print("rotation : {}".format(self.rotation))
        print("angle_steer: {} ".format(int(steer)))


        return  int(steer), 0


# #! /usr/bin/env python
# # -*- coding:utf-8 -*-

# from ar_track_alvar_msgs.msg import AlvarMarkers
# from tf.transformations import euler_from_quaternion

# class ARCurveController(object):
#     '''
#     Speed and Steer controller for precise parking using AR tag
#     '''
#     def __init__(self):
#         self.flag = 0
#         # self.global_marker_id = [0]*10
#         self.angle = 10

#     def __call__(self, ar_msg):
#         '''
#         return self.angle and speed from x, y, yaw of a AR tag
#         '''
#         markers_point_list = []

#         for i in ar_msg.markers:
#             pose = i.pose.pose
#             markers_point_list.append(pose.position.y)
#         markers_point_list = sorted(markers_point_list, reverse=True)

#         if len(markers_point_list) == 1 and markers_point_list[0] < 0.2:
#             # self.angle = 10
#             self.flag = 1
#             print("ar marker count : ", len(markers_point_list))
#             return self.flag, 25 
#         elif len(markers_point_list) != 0 and  markers_point_list[0] > 0.3:
#             return self.flag, 0
#         else:
#             # self.flag = 0
#             return self.flag, -25

