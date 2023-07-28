import numpy as np
import cv2
import glob
import yaml

# load
with open('Calibration_result.yaml') as f:
    yaml_data = yaml.load(f, Loader=yaml.SafeLoader)
#print(yaml_data)

camera_matrix = yaml_data['Camera matrix']
dist_str = yaml_data['Distortion coefficient']

# [[1.18394737e+03 0.00000000e+00 9.18285069e+02]
#  [0.00000000e+00 1.13728655e+03 4.99335894e+02]
#  [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
# ---------
# [[-0.50671194  0.21897301 -0.00291398  0.00713051 -0.04281172]]

#print(type(camera_matrix))
print("-----------")

mtx_list = camera_matrix.split(',')

list_of_mtx = []

for i in range(3):
    sub_list = []
    for j in range(3):
        #print(3*i + j)
        sub_list.append(float(mtx_list[3*i + j]))
    list_of_mtx.append(sub_list)
mtx = np.array(list_of_mtx)

dist_list = dist_str.split(',')

list_of_dist = []

for i in range(5):
    list_of_dist.append(float(dist_list[i]))
dist = np.array([list_of_dist])

print(mtx)
print(dist)

cap = cv2.VideoCapture(0)
frameRate = 1

while True:
    # 한 장의 이미지(frame)를 가져오기
    # 영상 : 이미지(프레임)의 연속
    # 정상적으로 읽어왔는지 -> retval
    # 읽어온 프레임 -> frame
    retval, frame = cap.read()
    if not (retval):  # 프레임정보를 정상적으로 읽지 못하면
        break  # while문을 빠져나가기

    frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)

    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)
    ## mtx = getOptimalNewCameraMatrix parameter alpha
    ## dist = Free scaling parameter
    ## 4번째 인자 = between 0 (when all the pixels in the undistorted image are valid) and 1 (when all the source image pixels are retained in the undistorted image)
    ## 1에 가까울수록 왜곡을 펼 때 잘라낸 부분들을 더 보여준다
    ## 전체를 보고 싶다면 1, 펴진 부분만 보고 싶다면 0에 가깝게 인자 값을 주면 된다
    dst = cv2.undistort(frame, mtx, dist)  ## getOptimalNewCameraMatrix 함수를 쓰지 않은 이미지
    dst2 = cv2.undistort(frame, mtx, dist, None, newcameramtx)  ## 함수를 쓴 이미지
    #cv2.imshow('num1', dst)
    cv2.imshow('result', dst2)

    cv2.imshow('frame', frame)  # 프레임 보여주기
    key = cv2.waitKey(frameRate)  # frameRate msec동안 한 프레임을 보여준다

    # 키 입력을 받으면 키값을 key로 저장 -> esc == 27(아스키코드)
    if key == 27:
        break  # while문을 빠져나가기





if cap.isOpened():  # 영상 파일(카메라)이 정상적으로 열렸는지(초기화되었는지) 여부
    cap.release()  # 영상 파일(카메라) 사용을 종료

cv2.destroyAllWindows()