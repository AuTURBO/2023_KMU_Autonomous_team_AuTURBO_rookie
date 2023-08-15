#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
# from sensor_msgs.msg import Ultrasonic
from detection_msgs.msg import BoundingBox, BoundingBoxes
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

class XycarSensor(object):
    '''
    Class for receiving and recording datas from Xycar's Sensors
    '''
    def __init__(self):   
        # camera sensor
        self.cam = None
        self.bridge = CvBridge()
        self.sub_cam = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_cam)

        # yolo node
        self.count = {"grandeur":0, "avante":0, "sonata":0}
        self.sum_x = {"grandeur":0.0, "avante":0.0, "sonata":0.0}
        self.sub_yolo = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.callback_yolo)

        # lidar sensor
        self.lidar = None
        self.angle_increments = None
        self.sub_lidar = rospy.Subscriber("scan", LaserScan, self.callback_lidar, queue_size=1)

        # imu sensor
        self.yaw = None
        self.sub_imu = rospy.Subscriber('imu', Imu, self.callback_imu, queue_size=1)

        # ar alvar track
        self.ar_id, self.ar_msg, self.ar_x, self.ar_y, self.ar_yaw = None, None, None, None, None
        self.sub_ar = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback_ar, queue_size = 1)
        
        # ultrasonic sensor
        self.ultra = None
        # self.sub_ultra = rospy.Subscriber('ultrasonic', Ultrasonic, self.callback_ultra, queue_size = 1)

    # def callback_ultra(self, msg):
        # code update 필요 ... 

    # 카메라 콜백 함수 일단 넣었는데 정지선 인식 부분만 따로 만드시면 사용 안해도 됩니다~ 
    def callback_cam(self, msg):
        self.cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def callback_yolo(self, msg):
        for bbox in msg.bounding_boxes:
            self.count[bbox.Class] = self.count[bbox.Class] + 1
            self.sum_x[bbox.Class] = self.sum_x[bbox.Class] + (bbox.xmin + bbox.xmax) / 2


    # 라이다 콜백 함수
    def callback_lidar(self, msg):
        self.lidar = msg.ranges
        self.angle_increment = msg.angle_increment
    # imu 콜백 함수
    def callback_imu(self, data):
        _, _, yaw = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.yaw = yaw % (2*np.pi)
    # ar 콜백 함수
    def callback_ar(self, msg):
        self.ar_msg = msg
        for i in msg.markers:     
            pose = i.pose.pose
            self.ar_x = pose.position.x
            self.ar_y = pose.position.z
            self.ar_yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w))[1]
            self.ar_id = i.id

    def init(self, rate):
        '''
        wait for initial callbacks from all sensors
        set initial yaw 
        '''
        # ready lidar
        while self.lidar is None:
            rate.sleep()
        print("lidar ready")

        # ready imu
        while self.yaw is None:
            rate.sleep()
        print("imu ready")

        # set initial yaw
        yaws = []
        for _ in range(11):
            yaws.append(self.yaw)
            rate.sleep()
        yaw0 = np.median(yaws)
        print("initial yaw = {}".format(yaw0))
        return yaw0