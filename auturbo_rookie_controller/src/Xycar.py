#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from Timer import Timer
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32, String
import time

from XycarSensor import XycarSensor

from Detector.LineDetector import LineDetector
from Detector.StopLineDetector import StopLineDetector
from Detector.ObstacleDetector import ObstacleDetector
from Detector.ObjectDetector import ObjectDetector

from Controller.RubberconController import RubberconController
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
        self.obstacle_detector = ObstacleDetector(self.timer)
        # 객체인식 주차
        # grandeur avante sonata
        car_target = "avante"
        car_rest = "grandeur"
        self.direction = "none"
        self.objectdetector = ObjectDetector(self.timer, car_target, car_rest)
        # self.verticalparking = VerticalParking(self.timer)
        # stop line 감지기 생성
        self.stopline_detector = StopLineDetector()

        # 목표 차선 정보 받아오기 & 목표 각도 받아오기 
        self.line_detector = LineDetector()
        # rospy.Subscriber("xycar_angle", Int32, self.target_angle_callback, queue_size=10)
        # self.target_angle = 0
        # 목표 차선 정보 pub 
        self.pub_target_lane = rospy.Publisher("/obstacle/info", String,queue_size=10)

        # 모드 컨트롤러 생성
        self.mode_controller = ModeController(self.timer)
        # 펄슛 컨트롤러 생성
        self.pursuit_controller = PurePursuitController(self.timer)
        # AR 컨트롤러 생성
        self.ar_controller = ARController()
        # AR 컨트롤러 생성
        self.ar_curve_controller = ARCurveController()
        self.ar_start = 0
        self.ar_cnt = 0
    
        # 루버콘 컨트롤러 생성 
        self.rubbercon_controller = RubberconController(self.timer)

        self.rubber_action_flag= 0
        self.rubber_state_flag = 0
    
        self.target_lane = 'middle'
        self.control_dict = {
            # 직선 주행 findparking
            # 긴 구간 직진 -- 07.31 테스트 
            'ar_marker_pose' : self.ar_marker_pose,
            'long straight' : self.pursuit,
            # 짧은 구간 직진 -- 07.31 테스트
            'short straight' : self.pursuit,
            # 각 모서리의 커브 구간을 뜻합니다. -- 07.31 테스트
            'curve': self.pursuit,
            # == 미션 1 평행주차 ===== #
            # 주차 공간 찾기 -- 07.31 테스트
            'findparallelparking': self.findparallelparking,
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
            'object' : self.object,
            'findverticalparking': self.findverticalparking,
            # == 미션 3 객체 인식 후 주차 == # -- 07.31 테스트
            'verticalparking': self.verticalparking,
            # == 미션 4 장애물 회피 == # -- 07.31 테스트
            'obstacle': self.obstacle,
            # == 미션 5 정지선 정지 == # -- 08.08 테스트
            # stopline 정지 이 부분은 만들어져 있는 함수 테스트 해주세요 따로 만드셔도 됩니다. 
            'stopline': self.stopline,
            # == 미션 6 라바콘 주행 == # -- 07.31 테스트
            # 라바콘 주행 
            'rubbercon': self.rubbercon
            ################################################################################# 
        }
    def ar_marker_pose(self):
        x = self.sensor.ar_x
        y = self.sensor.ar_y
        yaw = self.sensor.ar_yaw
        
        id = self.sensor.ar_id
        print("ar_marker_pose")
        print("x : ", x, "y : ", y, "yaw : ", yaw, "id : ", id)
    
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
        self.target_angle = self.line_detector(self.sensor.cam)
        self.msg.angle, self.msg.speed = self.pursuit_controller(self.target_angle, self.mode_controller.get_mode())
        self.pub.publish(self.msg)
        self.rate.sleep()

    # ================================ 미션 1 수평주차 =======================================================#
    # 주차공간 찾기 
# x  0.7517626732542556   y  1.1183388094957174  w  -0.26245737259726215  id  0
# 0.5074189710992407   y  0.9376329910134537  w  -0.1507904648456464
    def findparallelparking(self):
        # self.sensor.ar_x, self.sensor.ar_y, self.sensor.ar_yaw, self.sensor.ar_id
        if self.sensor.ar_id == 0:
            # msg = String()
            # msg.data = str("right")
            # self.pub_target_lane.publish(msg)
            x = self.sensor.ar_x
            y = self.sensor.ar_y
            yaw = self.sensor.ar_yaw
            
            id = self.sensor.ar_id
            print("x ", x, "  y ", y, " w ", yaw, " id ", id)
            if 0.7 < y < 2 and yaw < 0.1 :      #id == 0  미포함
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
        for _ in range(75):
            self.msg.angle, self.msg.speed = 0, 3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(10):
            self.msg.angle, self.msg.speed = -50, 3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(27):
            self.msg.angle, self.msg.speed = 50, -3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(30):
            self.msg.angle, self.msg.speed = -50, -3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(10):
            self.msg.angle, self.msg.speed = 0, 3
            self.pub.publish(self.msg)
            self.rate.sleep()
        print("평행 주차 시작")
        self.mode_controller.set_mode('arparking')
    #x  0.22868945620672587   y  0.91746097ObjectDetector73533844  w  0.20812997531422267  id  1
    #x  0.1765239034700272   y  0.716902588854083  w  0.24787252483351582  id  1
    #x  0.11365149612910468   y  0.39251462070955173  w  0.11461349366373742  id  1

    # 가로주차 aruco marker 인식 후 미세조정 모드 시작
    def arparking(self):
        self.msg.angle, self.msg.speed = self.ar_controller(self.sensor.ar_x, self.sensor.ar_y, self.sensor.ar_yaw)
        self.pub.publish(self.msg)
        if self.msg.speed == 0:
            for _ in range(30):
                self.msg.angle, self.msg.speed = 0, 0
                self.pub.publish(self.msg)
                self.rate.sleep()
            
            for _ in range(15):
                self.msg.angle, self.msg.speed = 50, -3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(20):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(20):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()

            print('King wang zzang AuTURBO OK?')
            # 모드 커브모드로 바꾸기
            self.mode_controller.set_mode('findverticalparking')

        self.rate.sleep()
    # ================================================================================================#

    # ================================ 미션 2 AR 커브 주행 =============================================#
    # 이 부분을 채워주세요~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # !!!!!!!
    def ar_curve(self):
        self.ar_cnt = 0
        ar_flag = 0
        self.msg.angle, ar_flag = self.ar_curve_controller(self.sensor.ar_msg)
        self.ar_start = ar_flag
        if self.ar_start == 1:
            print('AR 모드 시작')
            if ar_flag == 0: # 인식될 경우
                self.ar_start = 1 
                self.msg.speed = 0
                self.pub.publish(self.msg)
            else:
                if self.ar_cnt > 20:
                    # ar 모드 종료
                    print('AR 모드 종료')
                    self.mode_controller.set_mode('findverticalparking')
                else:
                    self.ar_cnt += 1
                    self.msg.speed = 0
                    self.pub.publish(self.msg)
            self.rate.sleep()
        else:
            self.pursuit()
        # 다음모드 커브모드 
        # 객체인식 주차모드일 수 있음 
    # ================================================================================================#

    # ================================ 미션 3 객체 인식 후 주차 ==========================================#
    # 이 부분을 채워주세요~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # !!!!!!!
    def object(self):
        self.pursuit()
        self.direction = self.objectdetector(self.sensor.detect, self.sensor.x_mid, self.sensor.y)
        self.pub.publish(self.msg)
        if self.direction == "right" or self.direction == "left":
            self.mode_controller.set_mode('findverticalparking')
        self.rate.sleep()

#x  0.5155718232876816   y  1.4638857327161652  w  -0.09014562949153192  id  2

    def findverticalparking(self):
        if self.sensor.ar_id != None and self.sensor.ar_id != 7:
            x = self.sensor.ar_x
            y = self.sensor.ar_y
            yaw = self.sensor.ar_yaw
            id = self.sensor.ar_id
            #print("x ", x, "  y ", y, " w ", yaw, " id ", id)
            if 0.6 < y < 2.3:      #id == 0  미포함
                print('start parking...')
                self.msg.angle, self.msg.speed = 0, 0
                self.pub.publish(self.msg)
                self.mode_controller.set_mode('verticalparking')
        else:    
            self.pursuit()

    def verticalparking(self):
        # self.direction = 'left'
        if self.direction == 'right' :  #direction은 Yolo를 통해 받은 변수의 값
            print("오른쪽 주차시작")
            #AR 인식 후 앞으로 직진
            for _ in range(110):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #핸들 왼쪽으로 꺽어서 앞으로 살짝 진진
            for _ in range(15):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #핸들 오른쪽으로 꺽어서 후진
            for _ in range(36):
                self.msg.angle, self.msg.speed = 50, -3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #그대로 후진
            for _ in range(15):
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
            for _ in range(20):
                self.msg.angle, self.msg.speed = 0, 3 
                self.pub.publish(self.msg)
                self.rate.sleep()
            #오른쪽으로 나오기
            for _ in range(45):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            
            # 그대로 직진( 이제 차선 인식 하면 됨)
            for _ in range(20):
                self.msg.angle, self.msg.speed = 0, 3 
                self.pub.publish(self.msg)
                self.rate.sleep()

        elif self.direction == 'left':
            print("왼쪽 주차시작") 
            #AR 인식 후 앞으로 직진
            for _ in range(110):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #핸들 오른쪽 꺽어서 앞으로 살짝 진진
            for _ in range(15):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #핸들 왼쪽 꺽어서 후진
            for _ in range(36):
                self.msg.angle, self.msg.speed = -50, -3
                self.pub.publish(self.msg)
                self.rate.sleep()
            #그대로 후진
            for _ in range(15):
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
            for _ in range(20):
                self.msg.angle, self.msg.speed = 0, 3 
                self.pub.publish(self.msg)
                self.rate.sleep()
            #왼쪽으로 나오기
            for _ in range(45):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            
            # 그대로 직진( 이제 차선 인식 하면 됨)
            for _ in range(20):
                self.msg.angle, self.msg.speed = 0, 3 
                self.pub.publish(self.msg)
                self.rate.sleep()

            self.mode_controller.set_mode('obstacle')
        # ================================================================================================#



    # ================================================================================================#

    # ================================ 미션 4 장애물 회피 ==============================================#

    def obstacle(self):
        self.target_lane = self.obstacle_detector(self.sensor.lidar, self.sensor.angle_increment)
        flag = 0
        if self.target_lane == "left":
            for _ in range(7):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(8):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(15):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(8):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(9):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()                      
            for _ in range(10):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()

            for _ in range(7):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(8):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(10):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
                flag = 1

####################right####################################

        elif self.target_lane == "right":

            for _ in range(9):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(9):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(10):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(16):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()              
            for _ in range(17):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(12):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(17):
                self.msg.angle, self.msg.speed = 50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(19):
                self.msg.angle, self.msg.speed = -50, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in range(10):
                self.msg.angle, self.msg.speed = 0, 3
                self.pub.publish(self.msg)
                self.rate.sleep()
                flag = 1

        if flag == 1:
            self.mode_controller.set_mode('stopline')
            self.pursuit()
        else:
            self.pursuit()
    # =====================================================================================================#

    # ================================ 미션 5 스탑라인(횡단보도) 정지 ===========================================#
    def stopline(self):
        if self.stopline_detector(self.sensor.cam):
            print("정지선을 인식했습니다.")
            self.stop6s()
        else:
            self.pursuit()
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
                    self.mode_controller.set_mode('short straight')        #우성님...헬프
            self.rate.sleep()

 
    # 메인 루프 
    def control(self):
        # 어떤 모드인지 확인 후 해당 모드에 맞는 제어 수행
        # mode = self.mode_controller(self.sensor.yaw)
        mode = self.mode_controller()
        # rospy.loginfo("current mode is %s", mode)
        
        self.control_dict[mode]()
        # cv2.waitKey(1)    