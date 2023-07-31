#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from Timer import Timer
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32

from XycarSensor import XycarSensor

# from Detector.StopLineDetector import StopLineDetector
from Detector.ObstacleDetector import ObstacleDetector


from Controller.PursuitController import PurePursuitController 
from Controller.ModeController import ModeController
from Controller.ARController import ARController
from Controller.ARCurveController import ARCurveController

class Xycar(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.timer = Timer()
        self.sensor = XycarSensor()
        yaw0 = self.sensor.init(self.rate)

        # 장애물 감지기 생성
        # self.obstacle_detector = ObstacleDetector(self.timer)
        # stop line 감지기 생성
        # self.stopline_detector = StopLineDetector()

        # 목표 차선 정보 받아오기 & 목표 각도 받아오기 
        rospy.Subscriber("xycar_angle", Int32, self.target_angle_callback, queue_size=10)
        self.target_angle = 0

        # 모드 컨트롤러 생성
        self.mode_controller = ModeController(yaw0, self.timer)
        # 펄슛 컨트롤러 생성
        self.pursuit_controller = PurePursuitController()
        # AR 컨트롤러 생성
        self.ar_controller = ARController()
        # AR 컨트롤러 생성
        self.ar_curve_controller = ARCurveController()
    
    
        self.target_lane = 'middle'
        self.control_dict = {
            # 직선 주행 
            # 긴 구간 직진 -- 07.31 테스트 
            'long straight' : self.pursuit,
            # 짧은 구간 직진 -- 07.31 테스트
            'short straight' : self.pursuit,
            # 각 모서리의 커브 구간을 뜻합니다. -- 07.31 테스트
            'curve': self.pursuit,
            

            # == 미션 1 평행주차 ===== #
            # 주차 공간 찾기 -- 07.31 테스트
            'findparking': self.findparking,
            # 가로주차  -- 07.31 테스트
            'parallelparking': self.parallelpark,
            # AR 정밀주차 -- 07.31 테스트
            'arparking': self.arparking,
            # ===================== #

            # 차량 정지 -- 07.31 테스트
            'poweroff' : self.poweroff,
 
            ## ---- 추가 해주시면 됩니다!---------------------------------------------- ##
            # == 미션 2 ar curve 주행 == # -- 08.08 테스트
            'ar_curve': self.ar_curve, 
            # == 미션 3 객체 인식 후 주차 == # -- 07.31 테스트
            'objectparking': self.objectparking,
            # == 미션 4 장애물 회피 == # -- 07.31 테스트
            'obstacle': self.obstacle,
            # == 미션 5 정지선 정지 == # -- 08.08 테스트
            # stopline 정지 이 부분은 만들어져 있는 함수 테스트 해주세요 따로 만드셔도 됩니다. 
            'stopline': self.stopline,
            # == 미션 6 라바콘 주행 == # -- 07.31 테스트
            # 라바콘 주행 
            'labcon': self.labcon
            #################################################################################
            
        }
    # cv로 차선 인식 후 목표 각도값 받아오기 
    def target_angle_callback(self, msg):
        self.target_angle = msg.data
    
    # 차량 정지 
    def poweroff(self):
        self.msg.speed, self.msg.angle = 0, 0
        self.pub.publish(self.msg)
        self.rate.sleep()
        # cnt++ 
        # cnt = 1 모드가 커브노드 되도록 
        # cnt = 2 모드가 커브 모드 실행 

    # 차선 컨트롤러
    def pursuit(self):
        self.msg.angle, self.msg.speed = self.pursuit_controller(self.target_angle)
        self.pub.publish(self.msg)
        self.rate.sleep()

    # ================================ 미션 1 수평주차 =======================================================#
    # 주차공간 찾기 
    def findparking(self):
        if self.timer() > 1.0:
            ranges = np.array(self.ranges)
            ranges = ranges[505//4:505//2]
            if np.count_nonzero((ranges > 0.0 ) & (ranges < 0.7)) > 20 :
                print('start parking...')
                self.mode_controller.set_mode('parallelparking')
        self.pursuit()

    # 가로주차 주차공간 들어가기 
    def parallelpark(self):
        for _ in range(34):
            self.msg.angle, self.msg.speed = 0, 20
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(43):
            self.msg.angle, self.msg.speed = 50, -20
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(43):
            self.msg.angle, self.msg.speed = -50, -20
            self.pub.publish(self.msg)
            self.rate.sleep()
        self.mode_controller.set_mode('arparking')

    # 가로주차 aruco marker 인식 후 미세조정 모드 시작
    def arparking(self):
        self.msg.angle, self.msg.speed = self.ar_controller(self.sensor.ar_x, self.sensor.ar_y, self.sensor.ar_yaw)
        self.pub.publish(self.msg)
        if self.msg.speed == 0:
            self.mode_controller.set_mode('poweroff')
            print('King wang zzang AuTURBO OK?')
        self.rate.sleep()
    # ================================================================================================#

    # ================================ 미션 2 AR 커브 주행 =============================================#
    # 이 부분을 채워주세요~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # !!!!!!!
    def ar_curve(self):
        self.msg.angle = self.ar_curve_controller(self.sensor.ar_msg)
        self.pub.publish(self.msg)
        if self.msg.angle == 0:
            print('finish ar_curve')
            # self.mode_controller.set_mode('curve')
        self.rate.sleep()
        # 다음모드 커브모드 
    # ================================================================================================#

    # ================================ 미션 3 객체 인식 후 주차 ==========================================#
    # 이 부분을 채워주세요~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # !!!!!!!
    def objectparking(self):
        return "help me"
    # ================================================================================================#

    # ================================ 미션 4 장애물 회피 ==============================================#

    def obstacle(self):
        self.target_lane = self.obstacle_detector(self.sensor.lidar, self.sensor.angle_increment)
     
        print(self.target_lane + ' 를 향해 가야함')
        # pub target lane 왼, 오, 중 차선으로 가야함
        # 차선 정보 노드에 보내기 

        # 장애물 회피 이후 스탑라인 찾기 
        if self.obstacle_detector.obstacle_counter == 4:
            print('detecting stopline...')
            self.obstacle_detector.obstacle_counter = 0
            self.mode_controller.set_mode("stopline")
        self.pursuit()
    # =====================================================================================================#

    # ================================ 미션 5 스탑라인(횡단보도) 정지 ===========================================#
    def stopline(self):
        if self.stopline_detector(self.sensor.cam):
            self.stop6s()
        else:
            self.pursuit()
    # 5초 정지 후 3초 이내에 출발 해야합니다. 
    def stop6s(self):
        print("stop for 5s...")
        yaws = []
        for _ in range(52):
            yaws.append(self.sensor.yaw)
            self.msg.angle, self.msg.speed = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        print('go!')
        yaw0 = (np.mean(yaws) - np.pi) % (2*np.pi)
        self.mode_controller.set_yaw0(yaw0)
        self.mode_controller.set_mode('curve')
    # =====================================================================================================#

    # ================================ 미션 6 라바콘 주행 ====================================================#
    def labcon(self):
        self.msg.angle, self.msg.speed = self.labcon_controller(self.sensor.yaw)
        self.pub.publish(self.msg)
        self.rate.sleep()


    # 메인 루프 
    def control(self):
        # 어떤 모드인지 확인 후 해당 모드에 맞는 제어 수행
        mode = self.mode_controller(self.sensor.yaw)
        self.control_dict['ar_curve']()
        # cv2.waitKey(1)