import numpy as np
import cv2
import glob
import yaml
import os


def undistort_func(frame):
    filePath = __file__    
    path, filename = os.path.split(filePath)
    #print("Script file path is {}, filename is {}".format(path, filename))
    yaml_path = path + "/Calibration_result.yaml" 
    # load
    with open(yaml_path) as f:
        yaml_data = yaml.load(f, Loader=yaml.SafeLoader)
    #print(yaml_data)

    camera_matrix = yaml_data['Camera matrix']
    dist_str = yaml_data['Distortion coefficient']

    #print("-----------")

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

    #print(mtx)
    #print(dist)

    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)

    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)  ## 함수를 쓴 이미지

    return dst