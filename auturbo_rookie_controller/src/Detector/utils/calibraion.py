import numpy as np
import cv2
import glob
import yaml

result_Path = './'

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('./img/chessboard*.png')
for fname in images:
    img = cv2.imread(fname)
    origin = img.copy()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)

        winname = "test"
        cv2.namedWindow(winname)  # create a named window
        cv2.moveWindow(winname, 40, 30)  # Move it to (40, 30)
        cv2.imshow(winname, img)
        #cv2.imshow('img',img)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)  ## 왜곡 펴기

        print(mtx)
        print("---------")
        print(dist)

        camera_Matrix = [list(mtx[0]), list(mtx[1]), list(mtx[2])]
        dist_Coefficient = [list(dist[0])]
        distortion_Coefficient = ""
        camera_Matrix_str = ""

        for idx in range(0, 3):
            for cnt in range(0, 3):
                if (len(camera_Matrix_str) == 0):
                    camera_Matrix_str += str(camera_Matrix[idx][cnt])
                else:
                    camera_Matrix_str += "," + str(camera_Matrix[idx][cnt])

        for cnt in range(0, 5):
            if (len(distortion_Coefficient) == 0):
                distortion_Coefficient += str(dist_Coefficient[0][cnt])
            else:
                distortion_Coefficient += "," + str(dist_Coefficient[0][cnt])

        print("\n========================")
        print("Calibration process end.")
        print("========================\n")

        line1 = "<camera_Matrix>\n" + str(camera_Matrix_str) + "\n\n"
        line2 = "<distortion_Coefficient>\n" + str(distortion_Coefficient) + "\n\n"
        print(line1 + line2)

        calibration_Data = {}
        calibration_Data['Camera matrix'] = str(camera_Matrix_str)
        calibration_Data['Distortion coefficient'] = str(distortion_Coefficient)
        yaml_Text = calibration_Data

        with open(f'{result_Path}Calibration_result.yaml', 'w') as result:
            yaml.dump(yaml_Text, result, default_flow_style=False)

        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)
        ## mtx = getOptimalNewCameraMatrix parameter alpha
        ## dist = Free scaling parameter
        ## 4번째 인자 = between 0 (when all the pixels in the undistorted image are valid) and 1 (when all the source image pixels are retained in the undistorted image)
        ## 1에 가까울수록 왜곡을 펼 때 잘라낸 부분들을 더 보여준다
        ## 전체를 보고 싶다면 1, 펴진 부분만 보고 싶다면 0에 가깝게 인자 값을 주면 된다
        #dst = cv2.undistort(origin, mtx, dist)  ## getOptimalNewCameraMatrix 함수를 쓰지 않은 이미지
        dst2 = cv2.undistort(origin, mtx, dist, None, newcameramtx)  ## 함수를 쓴 이미지
        #cv2.imshow('num1', dst)
        cv2.imshow('num2', dst2)

        #cv2.waitKey(0)
cv2.destroyAllWindows()