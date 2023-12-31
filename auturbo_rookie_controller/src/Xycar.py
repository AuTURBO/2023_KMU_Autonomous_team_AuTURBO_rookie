#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from Timer import Timer
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32, String
import time

from XycarSensor import XycarSensor

from Detector.LaneDetector import LaneDetector
from Detector.StopLineDetector import StopLineDetector
from Detector.ObstacleDetector import ObstacleDetector
from Detector.ObjectDetector import ObjectDetector

from Controller.RubberconController import RubberconController
from Controller.PursuitController import PurePursuitController 
from Controller.ModeController import ModeController
from Controller.ARController import ARController
from Controller.ARCurveController import ARCurveController

mode_dict = {
    '0': 'long straight',
    '1': 'findparallelparking',
    '2': 'ar_curve',
    '3': 'findverticalparking',
    '4': 'obstacle',
    '5': 'stopline',
    '6': 'rubbercon',
    '10': 'zgzg'
}

class Xycar(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, mode, hz=10):
        
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.timer = Timer()
        self.sensor = XycarSensor()
        yaw0 = self.sensor.init(self.rate)

        # 장애물 감지기 생성
        self.obstacle_detector = ObstacleDetector(self.timer)

        # 객체인식 주차: grandeur avante sonata
        car_target = "sonata" 
        car_rest = "grandeur"
        self.direction = "none"
        self.objectdetector = ObjectDetector(self.timer, car_target, car_rest)
        # self.verticalparking = VerticalParking(self.timer)

        # stop line 감지기 생성
        self.stopline_detector = StopLineDetector()

        # 목표 차선 정보 받아오기 & 목표 각도 받아오기 
        self.lane_detector = LaneDetector()
        # rospy.Subscriber("xycar_angle", Int32, self.target_angle_callback, queue_size=10)
        self.target_angle = 0
        # 목표 차선 정보 pub 
        self.pub_target_lane = rospy.Publisher("/obstacle/info", String,queue_size=10)

        # 모드 컨트롤러 생성
        if len(mode) == 1:
            self.mode_controller = ModeController(self.timer, self.sensor.yaw, mode_dict[mode[0]])
        if len(mode) == 2:
            self.mode_controller = ModeController(self.timer, self.sensor.yaw, mode_dict[mode[0]], mode[1])
        
        # PurePursuit 컨트롤러 생성
        self.pursuit_controller = PurePursuitController(self.timer)

        # AR 컨트롤러 생성
        self.ar_controller = ARController()
        # AR 컨트롤러 생성
        self.ar_curve_controller = ARCurveController(self.timer)
        self.ar_curve_state = 0
        self.ar_curve_action = 0
        self.ar_curve_flag = 0
    
        # 루버콘 컨트롤러 생성 
        self.rubbercon_controller = RubberconController(self.timer)
        self.rubber_action_flag= 0
        self.rubber_state_flag = 0
    
        self.target_lane = 'middle'
        self.control_dict = {
            # 1. High Speed Driving
            # 긴 구간 직진 
            'long straight' : self.pursuit,
            # 짧은 구간 직진 
            'short straight' : self.pursuit,
            # 커브 구간
            'curve': self.pursuit,
            # 지그재그 구간
            'zgzg': self.pursuit,
            # 차량 정지 
            'poweroff' : self.poweroff,

            # 2. Mission Driving
            # == 미션 1 평행주차 ===== #
            'findparallelparking': self.findparallelparking,
            'parallelparking': self.parallelpark,
            'arparking': self.arparking,
            # == 미션 2 ar curve 주행 == # 
            'ar_curve': self.ar_curve, 
            # == 미션 3 객체 인식 후 주차 == # 
            'findverticalparking': self.findverticalparking,
            'verticalparking': self.verticalparking,
            # == 미션 4 장애물 회피 == #
            'obstacle': self.obstacle,
            # == 미션 5 정지선 정지 == #
            'stopline': self.stopline,
            # == 미션 6 라바콘 주행 == # 
            'rubbercon': self.rubbercon,

            # ar tag test
            'ar_marker_pose' : self.ar_marker_pose
        }
    def ar_marker_pose(self):
        x = self.sensor.ar_x
        y = self.sensor.ar_y
        yaw = self.sensor.ar_yaw
        
        id = self.sensor.ar_id
        print("ar_marker_pose")
        print("x : ", x, "y : ", y, "yaw : ", yaw, "id : ", id)
    
    
    # 차량 정지 
    def poweroff(self):
        self.msg.speed, self.msg.angle = 0, 0
        self.pub.publish(self.msg)
        self.rate.sleep()

    # 차선 컨트롤러
    def pursuit(self):
        self.target_angle = self.lane_detector(self.sensor.cam)
        self.msg.angle, self.msg.speed = self.pursuit_controller(self.target_angle, self.mode_controller.get_mode())
        self.pub.publish(self.msg)
        self.rate.sleep()

    # ================================ 미션 1 수평주차 =======================================================#
    # 주차공간 찾기 
    # x  0.7517626732542556   y  1.1183388094957174  w  -0.26245737259726215  id  0
    # 0.5074189710992407   y  0.9376329910134537  w  -0.1507904648456464
    def findparallelparking(self):
        # self.sensor.ar_x, self.sensor.ar_y, self.sensor.ar_yaw, self.sensor.ar_id
        if self.sensor.ar_id != None:
            # msg = String()
            # msg.data = str("right")
            # self.pub_target_lane.publish(msg)
            x = self.sensor.ar_x
            y = self.sensor.ar_y
            yaw = self.sensor.ar_yaw
            
            id = self.sensor.ar_id
            print("x ", x, "  y ", y, " w ", yaw, " id ", id)
            if 0.7 < y < 2 :      #id == 0  미포함
                print('start parking...')
                self.msg.angle, self.msg.speed = 0, 0
                self.pub.publish(self.msg)
                self.mode_controller.set_mode('parallelparking')
            self.pursuit()
        else:
            # print("hi")
            self.pursuit()

    # 가로주차 주차공간 들어가기 
    def parallelpark(self):
        for _ in range(110):
            self.pursuit()
        for _ in range(7): #12 , 7
            self.msg.angle, self.msg.speed = -50, 3     
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(31):
            self.msg.angle, self.msg.speed = 50, -3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(28):
            self.msg.angle, self.msg.speed = -50, -3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(12):
            self.msg.angle, self.msg.speed = 0, 3
            self.pub.publish(self.msg)
            self.rate.sleep()
        print("평행 주차 시작")
        self.mode_controller.set_mode('arparking')

    # 가로주차 aruco marker 인식 후 미세조정 모드 시작
    #x  0.22868945620672587   y  0.91746097ObjectDetector73533844  w  0.20812997531422267  id  1
    #x  0.1765239034700272   y  0.716902588854083  w  0.24787252483351582  id  1
    #x  0.11365149612910468   y  0.39251462070955173  w  0.11461349366373742  id  1
    def arparking(self):
        self.msg.angle, self.msg.speed = self.ar_controller(self.sensor.ar_x, self.sensor.ar_y, self.sensor.ar_yaw)
        self.pub.publish(self.msg)
        if self.msg.speed == 0:
            for _ in range(30):
                self.msg.angle, self.msg.speed = 0, 0
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(15): #15
                self.msg.angle, self.msg.speed = 50, -3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(27):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(20):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(20):
                self.pursuit()

            print('King wang zzang AuTURBO OK?')
            # 모드 커브모드로 바꾸기
            self.mode_controller.set_mode('ar_curve')

        self.rate.sleep()
    # =====================================================================================================#

    # ================================ 미션 2 AR 커브 주행 =============================================#
    def ar_curve(self):
        # self.msg.angle, self.msg.speed, self.ar_curve_state, self.ar_curve_action = self.ar_curve_controller(self.sensor.lidar, self.sensor.angle_increment)
        # self.msg.angle, self.msg.speed, self.ar_curve_state, self.ar_curve_action = self.ar_curve_controller(self.sensor.lidar, self.sensor.angle_increment, self.ar_curve_state, self.ar_curve_action)
        # self.pub.publish(self.msg)  
        print("ar_y" , self.sensor.ar_y)
        if self.sensor.ar_id != None and self.sensor.ar_y < 1.2:
            print ("ar_curve 모드 시작")
            self.ar_curve_flag = 1
        if self.ar_curve_flag == 1:
            for _ in range(20):
                print("직진")
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(35):
                # print("오른쪽으로 꺾기")
                self.msg.angle, self.msg.speed = 25, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(25):
                # print("왼쪽으로 꺾기")
                self.msg.angle, self.msg.speed = -30, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(20):
                # print("중간직진 꺾기")
                self.msg.angle, self.msg.speed = -8, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(27):
                # print("왼쪽으로 꺾기")
                self.msg.angle, self.msg.speed = -30, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(15):
                self.msg.angle, self.msg.speed = 30, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            print ("ar_curve 모드 종료")
            self.mode_controller.set_mode('findverticalparking')

        else:
            self.pursuit()
    # =====================================================================================================#

    # ================================ 미션 3 객체 인식 후 주차 ==========================================#
    def findverticalparking(self):
        self.direction = self.objectdetector(self.sensor.detect, self.sensor.x_mid, self.sensor.y)
        if self.sensor.ar_id != None :
            y = self.sensor.ar_y
            print("마커 인식함")
            #print("x ", x, "  y ", y, " w ", yaw, " id ", id)
            if 0.6 < y < 2.3:      #id == 0  미포함
                if self.direction == 'none':
                    self.direction = "right"
                print('수직주차 시작...')
                self.msg.angle, self.msg.speed = 0, 0
                self.pub.publish(self.msg)
                self.mode_controller.set_mode('verticalparking')
            else:
                print("주차장 진입중...")
                self.pursuit()
        else:    
            print("주차장 진입중...2")
            self.pursuit()

    def verticalparking(self):
        # self.direction = 'right'
        if self.direction == 'right' :  #direction은 Yolo를 통해 받은 변수의 값
            print("오른쪽 주차시작")
            #AR 인식 후 앞으로 직진
            for _ in range(123):
                self.pursuit()
            #핸들 왼쪽으로 꺽어서 앞으로 살짝 진진
            for _ in range(13):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #핸들 오른쪽으로 꺽어서 후진
            for _ in range(36):
                self.msg.angle, self.msg.speed = 50, -3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #그대로 후진
            for _ in range(16):
                self.msg.angle, self.msg.speed = 0, -3 
                self.pub.publish(self.msg)
                self.rate.sleep()
            # 3초 기다리기
            for _ in range(30):
                self.msg.angle, self.msg.speed = 0, 0 
                self.pub.publish(self.msg)
                self.rate.sleep()

            #############주차 탈출하기 ##############
            #살짝 나오기
            for _ in range(17):
                self.msg.angle, self.msg.speed = 0, 3 
                self.pub.publish(self.msg)
                self.rate.sleep()
            #오른쪽으로 나오기
            for _ in range(37):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            
            # 그대로 직진
            for _ in range(50):     
                self.pursuit()

        elif self.direction == 'left':
            print("왼쪽 주차시작") 
            #AR 인식 후 앞으로 직진
            for _ in range(123):
                self.pursuit()
            #핸들 오른쪽 꺽어서 앞으로 살짝 진진
            for _ in range(13):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #핸들 왼쪽 꺽어서 후진
            for _ in range(36):
                self.msg.angle, self.msg.speed = -50, -3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #그대로 후진
            for _ in range(16):
                self.msg.angle, self.msg.speed = 0, -3 
                self.pub.publish(self.msg)
                self.rate.sleep()
            # 3초 기다리기
            for _ in range(30):
                self.msg.angle, self.msg.speed = 0, 0 
                self.pub.publish(self.msg)
                self.rate.sleep()
            
            #############주차 탈출하기 ##############  
            #살짝 나오기
            for _ in range(17):
                self.msg.angle, self.msg.speed = 0, 3 
                self.pub.publish(self.msg)
                self.rate.sleep()
            #왼쪽으로 나오기
            for _ in range(37):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            
            # 그대로 직진( 이제 차선 인식 하면 됨)
            for _ in range(50):
                self.pursuit()

        self.mode_controller.set_mode('obstacle')
    # =====================================================================================================#

    # ================================ 미션 4 장애물 회피 ==============================================#

    def obstacle(self):
        self.target_lane = self.obstacle_detector(self.sensor.lidar, self.sensor.angle_increment)
        flag = 0
        print("self.target_lane : ", self.target_lane)
        if self.target_lane == None:        #None은 장애물이 인식 안됬으므로 직진
            self.pursuit()
        else:
            if self.target_lane == "left":
                for _ in range(15):
                    self.msg.angle, self.msg.speed = -25, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(30):
                    self.msg.angle, self.msg.speed = 30, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(30):
                    self.msg.angle, self.msg.speed = -50, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(12):
                    self.msg.angle, self.msg.speed = 30, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(20):
                    self.pursuit()
                    flag = 1
            elif self.target_lane == "right":
                for _ in range(15):
                    self.msg.angle, self.msg.speed = 25, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(30):
                    self.msg.angle, self.msg.speed = -38, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(30):
                    self.msg.angle, self.msg.speed = 45, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(12):
                    self.msg.angle, self.msg.speed = -30, 3
                    self.pub.publish(self.msg)
                    self.rate.sleep()
                for _ in range(20):
                    self.pursuit()
                    flag = 1

            if flag == 1:
                print("정지선 진입 시작..")
                self.mode_controller.set_mode('stopline')
                # self.pursuit()
            else:
                self.pursuit()
    # =====================================================================================================#

    # ================================ 미션 5 스탑라인(횡단보도) 정지 ===========================================#
    def stopline(self):
        if self.stopline_detector(self.sensor.cam):
            print("정지선을 인식했습니다.")
            self.stop6s()
        else:
            self.msg.angle, self.msg.speed = 0, 3
            self.pub.publish(self.msg)
            self.rate.sleep()
    # 5초 정지 후 3초 이내에 출발 해야합니다. 
    def stop6s(self):
        print("5초간 정지합니다.")
        yaws = []
        for _ in range(52):
            yaws.append(self.sensor.yaw)
            self.msg.angle, self.msg.speed = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        print('go!')
        yaw0 = (np.mean(yaws) - np.pi) % (2*np.pi)
        # self.mode_controller.set_yaw0(yaw0)
        self.mode_controller.set_mode('rubbercon')
    # =====================================================================================================#

    # ================================ 미션 6 라바콘 주행 ====================================================#
    def rubbercon(self):
            self.msg.angle, self.msg.speed, self.rubber_state_flag, self.rubber_action_flag = self.rubbercon_controller(self.sensor.lidar, self.sensor.angle_increment)
            self.pub.publish(self.msg)
            if self.rubber_state_flag == 0 and self.rubber_action_flag == 0:
                self.pursuit()
                print("장애물 인식 전, 차선을 따라갑니다.")
            elif self.rubber_state_flag ==0 and self.rubber_action_flag == 1:
                print("라바 콘을 회피하여 주행합니다.")
            elif self.rubber_state_flag == 1 and self.rubber_action_flag == 0:
                    print('obstacle 모드 종료')
                    self.mode_controller.set_mode('zgzg')
            self.rate.sleep()
    # =====================================================================================================#

 
    # 메인 루프 
    def control(self):
        # mode = self.mode_controller(self.sensor.yaw)
        mode = self.mode_controller(self.target_angle, self.sensor.lidar, self.sensor.angle_increment, self.sensor.yaw)
        # rospy.loginfo("current mode is %s", mode)

        self.control_dict[mode]()
        # cv2.waitKey(1)    