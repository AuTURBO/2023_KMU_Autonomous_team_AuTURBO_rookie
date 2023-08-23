#!/usr/bin/env python

import numpy as np

class ObstacleDetector(object):
    '''
    라이다 데이터를 사용하여 장애물을 감지하는 클래스입니다.
    '''

    def __init__(self, timer):
        # 초기화 함수입니다.
        # timer는 시간을 측정하는 함수로서 장애물을 얼마나 오랫동안 감지해야 하는지를 관리합니다.

        self.avoid_direction = 'middle'
        self.obstacle_counter = 0
        self.timer = timer
        # 장애물을 감지하기 위한 타이머입니다.
        self.obs_dict = {1: 1., 2: 2.6, 3: 4}

    # 콜
    def __call__(self, ranges, angle_increment):
        '''
        라이다 입력으로부터 장애물을 피하기 위한 방향을 반환합니다.
        
        '''

        if self.obstacle_counter == 0 and (self.timer() > .35):
        
            # print(ranges)
            # 장애물 카운터가 0이고, 타이머가 0.35초 이상인 경우에만 장애물을 감지합니다.
            ranges = np.array(ranges)
            
            # 라이다 데이터의 1/4 구간과 3/4 구간은 0으로 설정하여 로봇의 전방 및 후방을 제외합니다.
            ranges[:len(ranges)//4] = 0.0
            ranges[3*len(ranges)//4:] = 0.0
                
            # 각도 값들을 계산합니다.
            # print("angle_increment: ", angle_increment)
            deg = np.arange(len(ranges)) * angle_increment - 252 * angle_increment

            # 장애물로 판단할 조건을 마스킹하여 필터링합니다.
            # 거리값에 따른 필터링 조건을 설정합니다.
            mask = (np.abs(ranges * np.sin(deg)) < 0.4) & (0.15 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 0.4)
            # 필터링 조건에 따라 데이터를 필터링합니다.
            filtered = np.where(mask, ranges, 0.0)

            # 필터링된 데이터 중 0이 아닌 값의 인덱스를 찾습니다.
            nz = np.nonzero(filtered)[0]
            # print(nz)
            
            if len(nz) > 2:
                # print("nz: ", nz)
                # 만약 필터링된 데이터의 개수가 5개 이상이면, 어느 방향으로 피해야 할지 결정합니다.

                if np.median(nz) > len(ranges) // 2 and np.median(nz) < len(ranges):
                    print("right")
                    self.avoid_direction = 'right'
                else:
                    print("left")
                    self.avoid_direction = 'left'
 

                print('avoid to ' + self.avoid_direction)

                # 타이머를 업데이트하고, 장애물 카운터를 1 증가시킵니다.
                self.timer.update()
                self.obstacle_counter += 1
                return self.avoid_direction
            
            # nz 리스트 초기화
            nz = []
        elif self.obstacle_counter != 0:
            # 이미 장애물을 감지한 경우
            if self.timer() > self.obs_dict[self.obstacle_counter]:
                # 장애물에 대한 카운터 값에 해당하는 시간이 지나면 다음 단계로 넘어갑니다.
                if self.obstacle_counter == 3:
                    self.avoid_direction = 'middle'   
                                     
                else:
                    self.avoid_direction = 'left' if self.avoid_direction == 'right' else 'right'
                self.timer.update()
                self.obstacle_counter += 1
            print('avoid to ' + self.avoid_direction)
            return self.avoid_direction
    
