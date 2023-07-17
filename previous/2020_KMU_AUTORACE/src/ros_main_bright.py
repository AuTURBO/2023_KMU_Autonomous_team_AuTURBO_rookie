#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import time
import numpy as np
import math
from enum import Enum
from datetime import datetime

import roslib
import sys
import rospy

from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from rosservice import ROSServiceException

from std_msgs.msg import String
from sensor_msgs.msg import Image
from obstacle_detector.msg import Obstacles
from cv_bridge import CvBridge, CvBridgeError
from xycar_motor.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from ObstacleDetector import ObstacleDetector, Position
from posecal import Pose
from pidcal import PidCal
from slidewindow import SlideWindow
from warper import Warper
from CurveDetector import Curve
from StopDetector import StopDetector
from parking import Parking

bridge = CvBridge()
cv_image = None
obstacles = None
motor_pub = None
motor_msg = None
ar_data = None
MODE = 0
g_speed = 8.5

now = datetime.now()

warper = None
slidewindow = SlideWindow()
pidcal = PidCal()

OBSTACLE_NUM = 3


def img_callback(data):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.resize(cv_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)

    except CvBridgeError as e:
        print(e)


def motor_callback(msg):
    global motor_msg
    motor_msg = msg


def obstacle_callback(data):
    global obstacles
    obstacles = data


def get_marker(msg):
    global ar_data
    if len(msg.markers) != 0:
        for tag in msg.markers:
            if tag.id == 1:
                orientation = tag.pose.pose.orientation
                ori_lst = [orientation.x, orientation.y, orientation.z, orientation.w]
                position = tag.pose.pose.position

                ar_data = (ori_lst, position)


def get_yaw_data(data):
    yaw_data = None
    distance = None

    if data != None:
        roll, pitch, yaw_data = euler_from_quaternion(data[0])
        distance = np.sqrt(data[1].x ** 2 + data[1].y ** 2 + data[1].z ** 2)

    return yaw_data, distance


def drive(Angle, Speed):
    global motor_pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    motor_pub.publish(msg)


def img_process(img):
    cols, rows, ch = img.shape
    brightness = np.sum(img) / (255 * cols * rows)

    minimum_brightness = 1
    ratio = brightness / minimum_brightness
    bright_img = cv2.convertScaleAbs(img, alpha=1 / ratio, beta=0)

    gray = cv2.cvtColor(bright_img, cv2.COLOR_BGR2GRAY)

    kernel_size = 5
    blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    low_threshold = 45
    high_threshold = 60
    edge = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)

    roi = roi_interest(edge)

    # cv2.imshow("edge", edge)

    return roi


def roi_interest(img):
    width, height = img.shape[:2]

    vertices = np.array(
        [[
            (-300, 430),  # 좌하
            (240, 280),  # 좌상
            (420, 280),  # 우상
            (width + 300, 430)  # 우하
        ]], dtype=np.int32)

    mask = np.zeros_like(img)  # mask = img와 같은 크기의 빈 이미지

    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움
    cv2.fillPoly(mask, vertices, 255)

    # 이미지와 color로 채워진 ROI를 합침
    roi_image = cv2.bitwise_and(img, mask)

    return roi_image


def warpper_process(img):
    ret, thres_img = cv2.threshold(img, 60, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3, 3), np.uint8)
    dilate = cv2.dilate(thres_img, kernel, 3)

    sharp = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharp_img = cv2.filter2D(dilate, -1, sharp)

    return thres_img


# def calc_speed(MODE, curve_detector):
#     speed = 11
#
#     if MODE == 1 or MODE == 2:  # 장애물 전
#         speed = 5
#     elif MODE == 3:     # 횡단보도 전
#         speed = 7.45
#     elif MODE == 4:     # 주차
#         speed = 5
#     elif curve_detector.curve_count >= 3:
#         speed = 9.5
#
#
#     return speed


# test1
def calc_speed(MODE, curve_detector, pid):
    global g_speed

    if MODE == 0:
        if curve_detector.curve_count < 2:
            if abs(pid) > 0.055 and 10.5 < g_speed:
                g_speed -= 0.15
            elif g_speed < 11.0:
                g_speed += 0.1

        elif curve_detector.curve_count >= 3:
            if abs(pid) < 0.065 and 10.5 < g_speed:
                g_speed -= 0.2
            elif g_speed < 10.8:
                g_speed += 0.1


    elif MODE == 1 or MODE == 2:   # 장애물 전
        g_speed = 5
    elif MODE == 3:  # 횡단보도 전
        g_speed = 7.45



def avoidance(first_detect, POS, obs_cnt):
    # Part2. 왼오왼
    if first_detect == 1:   #
        if POS == 1:
            if obs_cnt == 0 or obs_cnt == 2:   # normal case
                for theta in range(270, 360, 9):
                    st = 0.27 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)

                for theta in range(360, 500, 9):
                    st = 0.22 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)
            else:       # 반대 케이스
                print("Reverse case")
                for theta in range(270, 360, 9):
                    st = 0.4 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    #print(st)
                    time.sleep(0.06)

                for theta in range(360, 500, 10):
                    st = 0.3 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    #print(st)
                    time.sleep(0.06)


            #drive(0, 0)
            #time.sleep(0.5)
        elif POS == 2:  # 오른쪽
            if obs_cnt == 1:  # normal case
                for theta in range(270, 520, 9):
                    st = 0.275 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    #print(st)
                    time.sleep(0.06)
            else:
                print("Reverse case")
                for theta in range(270, 360, 9):
                    st = 0.4 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)

                for theta in range(360, 500, 10):
                    st = 0.3 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)

            #drive(0, 0)
            #time.sleep(0.5)

    elif first_detect == 2:   # Right first
        # Part1. 오왼오

        if POS == 1:  # 왼쪽
            if obs_cnt == 1:  # normal case
                for theta in range(270, 360, 10):
                    st = 0.245 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)

                for theta in range(360, 500, 9):
                    st = 0.265 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)
            else:
                print("Reverse case")
                for theta in range(270, 360, 9):
                    st = 0.4 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    #print(st)
                    time.sleep(0.06)

                for theta in range(360, 500, 10):
                    st = 0.3 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    #print(st)
                    time.sleep(0.06)


            #drive(0, 0)
            #time.sleep(0.5)

        elif POS == 2:  # 오른쪽
            if obs_cnt == 0 or obs_cnt == 2:  # normal case
                for theta in range(270, 360, 9):
                    st = 0.24 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    #print(st)
                    time.sleep(0.06)

                for theta in range(360, 500, 8):
                    st = 0.19 * np.sin(theta * np.pi / 180)
                    drive(st, 5)
                    #print(st)
                    time.sleep(0.06)
            else:
                print("Reverse case")
                for theta in range(270, 360, 9):
                    st = 0.4 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)

                for theta in range(360, 500, 10):
                    st = 0.3 * np.sin(theta * np.pi / 180)
                    drive(-st, 5)
                    #print(-st)
                    time.sleep(0.06)

            #drive(0, 0)
            #time.sleep(0.5)


def finish():
    print("------ xycar finish ------")


def main():
    global motor_pub
    global cv_image
    global obstacles
    global MODE
    global ar_data
    global g_speed

    rospy.init_node("racecar")
    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size=1)
    armarker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_marker)

    h, w = 480, 640

    out = cv2.VideoWriter(
        '/home/nvidia/xycar_ws/src/racecar/video/slide{}{}{}.avi'.format(now.day, now.hour, now.minute),
        cv2.VideoWriter_fourcc("M", "J", "P", "G"), 30, (w, h))

    out2 = cv2.VideoWriter(
        "/home/nvidia/xycar_ws/src/racecar/video/origin{}{}{}.avi".format(now.day, now.hour, now.minute),
        cv2.VideoWriter_fourcc("M", "J", "P", "G"), 30, (w, h))

    print("------------- auto_race start!!! -------------")
    drive(0, 0)
    rospy.sleep(3)

    obstacle_detector = ObstacleDetector()
    curve_detector = Curve()
    stop_detector = StopDetector()
    parker = Parking()

    obs_time = 0
    start_time = time.time()
    cross_cnt = 0

    obs_cnt = 0

    first_detect = 0

    speed_default = 0
    speed_obstacle = 0

    x_location = None
    x_location_old = None

    stop_cnt = 0

    while not rospy.is_shutdown():
        global warper

        if cv2.waitKey(1) & 0xFF == 27:
            break

        if warper == None:
            warper = Warper(cv_image)

        process_img = img_process(cv_image)
        warp_img = warper.warp(process_img)
        process_img2 = warpper_process(warp_img)

        slideImage, x_location = slidewindow.slidewindow(process_img2)
        # curve 2번 돌고나서 obstacle
        POS, circle, distance = obstacle_detector.check(obstacles)

        # 시작점 체크
        # 횡단보도 및 시작점
        # and (MODE == 3 or MODE == 0)
        if MODE != 4 and stop_detector.check_crocss_walk(warp_img):
            if cross_cnt%2 == 0 and (MODE == 3 and obs_cnt > OBSTACLE_NUM):  # cross
                print("------ CROSS WALK DETECT ------")
                drive(0, 0)
                rospy.sleep(5.75)
                MODE = 0

                for t in range(13):
                    drive(0, 4)
                    rospy.sleep(0.1)

                curve_detector.curve_count += 1

            elif MODE == 0 and time.time()-start_time > 40:  # start
                print("------ STOP LINE DETECT ------")
                MODE = 0
                obs_cnt = 0
                obs_time = 0
                curve_detector.curve_count = 0
                start_time = time.time()
                stop_cnt += 1

            cross_cnt += 1

        # curve 2번 돌고나서 obstacle
        if MODE == 2:
            if obs_cnt == 0 and POS.value != 0:
                obs_time = time.time()
                first_detect = POS.value

            if POS.value != 0:
                # 왼오왼, 오왼오 통합
                print("Obstacle Detect:", POS.value, obs_cnt)
                avoidance(first_detect, POS.value, obs_cnt)
                obs_cnt += 1


        if MODE == 2 and (obs_cnt == OBSTACLE_NUM or (obs_time != 0 and time.time() - obs_time > 10)):
            # Part1. 오왼오 다시 조정 코드
            if first_detect == 2:
                for theta in range(480, 545, 9):
                    st = 0.22 * np.sin(theta * np.pi / 180)
                    drive(st, 4)
                    time.sleep(0.05)

            MODE = 3
            obs_cnt = 4
            print("obstacle detect finish")

        # parking
        if MODE == 4:
            if time.time() - start_time > 12.93:
                print("------ parking mode on ------")
                pose = Pose()
                cur_pose = pose.get_curpose()
                mid_pose, goal_pose = parker.calc_drive_pose()

                print("parking: steer RIGHT!!!")
                print("mid:", mid_pose)
                cnt = 0
                drive(0, 0)
                time.sleep(0.5)

                while cur_pose[1] < mid_pose[1]:
                    drive(1, -3)
                    time.sleep(0.0955)
                    cur_pose = pose.calc_behind(20, -3)
                    # print("cur_pose:", cur_pose)
                    cnt += 1

                print("parking: steer LEFT!!")
                print("goal:", goal_pose)
                for t in range(cnt):
                    drive(-1, -3)
                    time.sleep(0.0955)
                    cur_pose = pose.calc_behind(-20, -3)
                    # print("cur_pose:", cur_pose)


                print("DISTANCE START")
                while True:
                    drive(0, 0)
                    time.sleep(1.5)

                    yaw, distance = get_yaw_data(ar_data)
                    print(yaw, distance)

                    if distance != None and 0.23 < distance < 0.27:
                        break

                    if distance > 0.27:
                        drive(yaw * 4, 2)
                        time.sleep(0.5)
                    elif distance < 0.23:
                        drive(yaw * 4, -2)
                        time.sleep(0.5)

                rospy.on_shutdown(finish)
                break

        if x_location != None:
            if x_location_old != None:
                if np.abs(x_location - x_location_old) < 60:
                    x_location_old = x_location
                else:
                    x_location = x_location_old

            pid = round(pidcal.pid_control(int(x_location), curve_detector.curve_count), 6)
            calc_speed(MODE, curve_detector, pid)

            # if MODE != 4 and time.time() - start_time < 5:
            #     speed_default = 9
            drive(pid, g_speed)

        else:
            x_location = x_location_old
            pid = round(pidcal.pid_control(int(x_location_old), curve_detector.curve_count), 6)
            calc_speed(MODE, curve_detector, pid)
            # if MODE != 4 and time.time()-start_time < 5:
            #     speed_default = 9
            drive(pid, g_speed)

        curve_detector.update(pid)
        curve_detector.count_curve(start_time)

        # cv2.imshow("warper", warp_img)
        # cv2.imshow("origin", re_image)
        # cv2.imshow("processImage", tempImg)

        print(round(pid, 4), g_speed, curve_detector.curve_count)
        calc_speed(MODE, curve_detector, pid)


        cv2.putText(slideImage, 'PID %f' % pid, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(slideImage, 'x_location %d' % x_location, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                    2)
        cv2.putText(slideImage, 'curve_cnt %d' % curve_detector.curve_count, (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255),
                    2)

        cv2.putText(slideImage, 'MODE %d' % MODE, (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255),
                    2)

        cv2.line(slideImage, (x_location, 380), (318, 479), (0, 255, 255), 3)
        # cv2.imshow("slidewindow", slideImage)



        if MODE == 0 and curve_detector.curve_count == 2:
            MODE = 1  # cross_walk
        elif MODE == 1 and obs_cnt < OBSTACLE_NUM:
            MODE = 2  # obstacle
        elif MODE == 0 and stop_cnt == 3:
            MODE = 4  # parking



        # out.write(slideImage)
        # out2.write(cv_image)

    # out.release()
    # out2.release()



if __name__ == "__main__":
    main()



