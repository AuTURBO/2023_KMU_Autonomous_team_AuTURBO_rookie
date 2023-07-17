#Autonomous Driving Code Using OpenCV-Python in ROS
#Jin Hyeok An - KonKuk University

#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import sys

############################
width = 640
height = 480
white = 255
line_gap = 30
line_number_old = [[4, 214, 574], [0, 219, 601], [0, 226, 626], [0, 239, 0], [0, 237, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
result_old = 0
############################

def opencv(frame):
    #region = np.float32([[0,370],[238,231],[329,231],[640,370]])
    #replace_image = np.float32([[0,600],[0,0],[350,0],[350,600]])
    #matrix = cv2.getPerspectiveTransform(region,replace_image)
    #result = cv2.warpPerspective(frame,matrix,(350,600))
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(3,3),0)
    canny = cv2.Canny(blur,65,195)
    canny_roi = region_of_interest(canny)
    lines = cv2.HoughLinesP(canny_roi,1,np.pi/180,3,40)
    background = np.zeros_like(frame)
    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
        cv2.line(background,(x1,y1),(x2,y2),(255,255,255),2)
    return background

#opencv

def region_of_interest(image):
    polygons = np.array([[(0,height-130),(width,height-130),(width,height),(0,height)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask,polygons,white)
    masked_image = cv2.bitwise_and(image,mask)
    return masked_image

#region_of_intersest

def line_pixel_found(image, line_y):
    pixel = []
    for number in range(width):
        color = image[line_y, number]
        if color[0]==white:
            pixel.append(number)
    return pixel

#line_pixel_found

def seperate_line(line, line_y):
    line_first = []
    line_second = []
    line_third = []
    if len(line)==0:
        return [0,0,0]
    else:
        line_first.append(line[0])
        flag = 0

        for pixel in range(1,len(line)):
            if line[pixel]-line[pixel-1] < line_gap and flag == 0:
                line_first.append(line[pixel])
            elif line[pixel]-line[pixel-1] < line_gap and flag == 1:
                line_second.append(line[pixel])
            elif line[pixel]-line[pixel-1] < line_gap and flag == 2:
                line_third.append(line[pixel])
            else:
                if flag == 0:
                    line_second.append(line[pixel])
                elif flag == 1:
                    line_third.append(line[pixel])
                flag = flag + 1
	
        line_first_avg = average_line(line_first)
        line_second_avg = average_line(line_second)
        line_third_avg = average_line(line_third)
        line_pix = [line_first_avg,line_second_avg,line_third_avg]
        return line_pix

#seperate_line

def average_line(line):
    array_sum = 0
    for value in line:
        array_sum = array_sum + value
    if array_sum == 0:
        array_sum = 0
    else:
        array_sum = int(array_sum/float(len(line)))
    return array_sum

#average_line

def line_numbering(line_old,line_new):
    first, second, third = 0, 0, 0
    if line_old[0] == 0:
        pass
    else:
        if abs(line_old[0] - line_new[0]) < line_gap:
            first = line_new[0]
        elif abs(line_old[0] - line_new[1]) < line_gap:
            first = line_new[1]
        elif abs(line_old[0] - line_new[2]) < line_gap:
            fisrt = line_new[2]
    if line_old[1] == 0:
        pass
    else:
        if abs(line_old[1] - line_new[0]) < line_gap:
            second = line_new[0]
        elif abs(line_old[1] - line_new[1]) < line_gap:
            second = line_new[1]
        elif abs(line_old[1] - line_new[2]) < line_gap:
            second = line_new[2]
    if line_old[2] == 0:
        pass
    else:
        if abs(line_old[2] - line_new[0]) < line_gap:
            third = line_new[0]
        elif abs(line_old[2] - line_new[1]) < line_gap:
            third = line_new[1]
        elif abs(line_old[2] - line_new[2]) < line_gap:
            third = line_new[2]


    if line_old[1]==0 and line_old[2] == 0 and line_old[0] != 0:
        if line_new[1] != 0:
            if line_new[1]-line_new[0]<480:
                second = line_new[1]
            else:
                third = line_new[1]
    if line_old[0]==0 and line_old[1] == 0 and line_old[2] != 0:
        if line_new[1] != 0:
            if line_new[1]-line_new[0]<480:
                second = line_new[0]
            else:
                first = line_new[0]
    if line_old[0]==0 and line_old[1]==0 and line_old[2]==0:
        if line_new[0] < 30:
            first = line_new[0]
        elif line_new[0] > 100 and line_new[0] < 540:
            second = line_new[0]
        elif line_new[0] > 610:
            third = line_new[0]
    if line_old[0] == 0 and line_old[2] == 0 and line_old[1] != 0:
        if line_new[0] < 30:
            first = line_new[0]
        elif line_new[1] > 610:
            third = line_new[1]


    line_new[0], line_new[1], line_new[2] = first,second,third
    return line_new

#line_numbering

def middle_line_control(line):
    count = 0
    sum_line = 0
    for number in range(9):
        if line[number][1] != 0:
            count = count + 1
            sum_line = sum_line + ((line[number][1] - (width/2.0))/(width/2.0)) * (1+(9-number) * 0.01)
    if count == 0:
        result = 0
        pass
    else:
        result = sum_line/count
    return result

#middle_line_control

def slop_control(line,middle_control):
    slop = []
    if middle_control > 0:
        for number in range(8):
            if line[number][0] != 0 and line[number+1][0] != 0:
                slop.append(make_slop(line[number][0],line[number+1][0])-1.28)
    elif middle_control < 0:
        for number in range(8):
            if line[number][2] != 0 and line[number+1][2] != 0:
                slop.append(make_slop(line[number][2],line[number+1][2])+1.28)
    elif middle_control == 0:
        pass
    if not slop:
        return 0
    else:
        return np.mean(slop)

#slop_control

def make_slop(x1,x2):
    return (x1-x2)/10.0

#make_slop

def pub_motor(Angle, Speed):
    drive_info = [Angle, Speed]
    drive_info = Int32MultiArray(data = drive_info)
    pub.publish(drive_info)

#pub_motor

def start():
    global pub
    rospy.init_node('my_driver')
    pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture ('/home/kusw-13/catkin_ws/src/xycar_simul/src/track-s.mkv')
    while cap.isOpened():
        line_number_new = []
        _, frame = cap.read()
        background = opencv(frame)

        for line_y in range(360,450,10):
            color_pix = line_pixel_found(background,line_y)
            seperate = seperate_line(color_pix,line_y)
            line_number_new.append(seperate)

        for number in range(9):	
            if line_number_new[number] == [0,0,0]:
                line_number_old[number] = [0,0,0]
                pass
            else:
                if line_number_new[number][2] == 0:
                    line_number_old[number] = line_numbering(line_number_old[number],line_number_new[number])
                else:
                    line_number_old[number] = line_number_new[number]

        result_middle = middle_line_control(line_number_old)
        
        Speed = 20

        if result_middle == 0:
            result_slop = slop_control(line_number_old,result_old)
            Angle = 95 * result_slop
        else:
            Angle = 160 * result_middle
            result_old = result_middle
            result_slop = slop_control(line_number_old,result_old)

        if Angle >50:
            Angle = 50
        elif Angle < -50:
            Angle = -50

        cv2.waitKey(1)
        pub_motor(Angle,Speed)
        rate.sleep()

#start

if __name__ == '__main__':
    start()




