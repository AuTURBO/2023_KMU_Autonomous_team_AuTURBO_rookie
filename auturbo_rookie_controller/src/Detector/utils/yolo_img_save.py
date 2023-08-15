import cv2
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from undistort import undistort_func

path = './img/'

# cap = cv2.VideoCapture(0)
frameRate = 1
i = 0

def main(frame):
    global i
    frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
    real_frame = undistort_func(frame)
    cv2.imshow('frame', frame)  # 프레임 보여주기
    cv2.imshow('real_frame', real_frame)  # 프레임 보여주기
    key = cv2.waitKey(frameRate)  # frameRate msec동안 한 프레임을 보여준다

    # 키 입력을 받으면 키값을 key로 저장 -> esc == 27(아스키코드)
    if key == 27:
        exit(0)  # while문을 빠져나가기


    if key == 97:
        fname = path + 'yolo' + str(i) + '.png'
        print(fname)
        cv2.imwrite(fname, real_frame)
        i = i + 1


# if cap.isOpened():  # 영상 파일(카메라)이 정상적으로 열렸는지(초기화되었는지) 여부
#     cap.release()  # 영상 파일(카메라) 사용을 종료

# cv2.destroyAllWindows()

def image_callback(msg):
    global lane_bin_th
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        main(cv_image)
        #cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def start():
    #global ack_publisher
    #global steer_angle_publisher
    rospy.init_node('img_save')
    image_topic = "/usb_cam/image_raw"

    rospy.Subscriber(image_topic, Image, image_callback)
    #ack_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    #steer_angle_publisher = rospy.Publisher('xycar_angle', Int32, queue_size=1)
    rospy.spin()


    #video_read('xycar_track2.mp4')

if __name__ == '__main__':
    start()