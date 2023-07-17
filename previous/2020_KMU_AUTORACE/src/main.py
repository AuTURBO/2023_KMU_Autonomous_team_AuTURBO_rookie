#!/usr/bin/env python


import cv2
import threading
import time
import numpy as np
import math


from slidewindow import SlideWindow
from warper import Warper
from pidcal import PidCal
from CurveDetector import Curve
from StopDetector import StopDetector

fgbg = cv2.createBackgroundSubtractorMOG2(varThreshold=500, history=1)
# knn = cv2.createBackgroundSubtractorKNN(varThreshold=300, history=30)

def filter(img):
    # roi_img = roi_interest(img[:,:,:1])
    if len(img.shape) == 3:
        img = img[:,:,:1]
    img = roi_interest(img)

    out_img = np.dstack((img, img, img))

    fgmask = fgbg.apply(img)

    nlabels, label, stats, centroids = cv2.connectedComponentsWithStats(fgmask)

    for index, centroid in enumerate(centroids):
        if stats[index][0] == 0 and stats[index][1] == 0:
            continue
        if np.any(np.isnan(centroid)):
            continue

        x, y, width, height, area = stats[index]
        centerX, centerY = int(centroid[0]), int(centroid[1])

        if area > 100:
            cv2.circle(out_img, (centerX, centerY), 1, (0, 255, 0), 2)
            cv2.rectangle(out_img, (x, y), (x + width, y + height), (0, 0, 255))

    return fgmask, out_img



def img_process(img):
    cols, rows, ch = img.shape
    brightness = np.sum(img) / (255 * cols * rows)

    minimum_brightness = 1
    ratio = brightness / minimum_brightness
    bright_img = cv2.convertScaleAbs(img, alpha = 1 / ratio, beta = 0)

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
            (240, 280),   # 좌상
            (420, 280),  # 우상
            (width + 300, 430)   # 우하
          ]], dtype=np.int32)

    mask = np.zeros_like(img)  # mask = img와 같은 크기의 빈 이미지

    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움
    cv2.fillPoly(mask, vertices, 255)

    # 이미지와 color로 채워진 ROI를 합침
    roi_image = cv2.bitwise_and(img, mask)

    return roi_image


def warpper_process(img):

    ret, thres_img = cv2.threshold(img, 60, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3,3), np.uint8)
    dilate = cv2.dilate(thres_img, kernel, 3)


    sharp = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharp_img = cv2.filter2D(dilate, -1, sharp)


    return thres_img



def hough_line(img):
    outimg = np.zeros_like(img)
    outimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2BGR)


    minLineLength = 5
    maxLineGap = 15
    l_lines = cv2.HoughLinesP(img[:, :330], 1, np.pi / 180, 30, minLineLength, maxLineGap)
    r_lines = cv2.HoughLinesP(img[:, 330:], 1, np.pi / 180, 30, minLineLength, maxLineGap)

    l_minx, l_maxx, l_miny, l_maxy = 99999, 0, 99999, 0
    r_minx, r_maxx, r_miny, r_maxy = 99999, 0, 99999, 0

    if np.all(l_lines) != None:
        for x in range(0, len(l_lines)):
            for x1, y1, x2, y2 in l_lines[x]:
                cv2.line(outimg,(x1,y1),(x2,y2),(0,255,0),10, cv2.LINE_AA)
                # pts = np.array([[x1, y1], [x2, y2]], np.int32)
                # cv2.polylines(outimg, [pts], True, (0, 255, 0))

        cv2.line(outimg,(l_minx,l_miny),(l_maxx,l_maxy),(0,255,0),10, cv2.LINE_AA)

    return outimg

slidewindow  = SlideWindow()
pidcal = PidCal()
curve_detector = Curve()
warper = None
stop_detector = StopDetector()

def main():
    global warper

    flag = False
    cap = cv2.VideoCapture("../video/org2.avi")

    x_location_old = None

    start_time = time.time()

    while True:

        # 이미지를 캡쳐
        ret, cv_image = cap.read()

        # 캡쳐되지 않은 경우 처리
        if not ret:
            break
        if cv2.waitKey(0) & 0xFF == 27:
            break

        if warper == None:
            warper = Warper(cv_image)


        # warper, slidewindow 실행
        process_img = img_process(cv_image)
        warp_img = warper.warp(process_img)
        process_img2 = warpper_process(warp_img)


        slideImage, x_location = slidewindow.slidewindow(process_img2)

        stop_detector.check_crocss_walk(warp_img)
        stop_detector.check_yellow_line(warp_img)

        # print(x_location, mid_point)

        if x_location != None:
            # test 4 lines
            if curve_detector.curve_count == 3 and np.abs(x_location - x_location_old) > 40:
                x_location = x_location_old
            else:
                x_location_old = x_location

            pid = round(pidcal.pid_control(int(x_location), curve_detector.curve_count), 6)

        else:
            x_location = x_location_old
            pid = round(pidcal.pid_control(int(x_location_old), curve_detector.curve_count), 6)

        curve_detector.update(pid)
        curve_detector.count_curve(start_time)

        # cv2.imshow("warper", warp_img)
        # cv2.imshow("origin", cv_image)
        # cv2.imshow("process", process_img)


        # print(round(pid, 2), x_location)
        cv2.putText(slideImage, 'PID %f' % pid, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(slideImage, 'x_location %d' % x_location, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                    2)
        cv2.putText(slideImage, 'curve_cnt %d' % curve_detector.curve_count, (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255),
                    2)
        cv2.line(slideImage, (x_location, 380), (320, 479), (0, 255, 255), 3)
        cv2.imshow("slidewindow", slideImage)

        # angle_lst = sorted(np.linspace(0, 19, 20), reverse=True)
        # print(angle_lst)


if __name__ == '__main__':
    main()