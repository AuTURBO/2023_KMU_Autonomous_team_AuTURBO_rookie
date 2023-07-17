import numpy as np
import cv2, time

class Stop_Detector :
    def __init__(self) :
        self.cnt = 0
        self.previous_time = time.time() + 5

    def check_time(self):
        if time.time() < self.previous_time + 10:
            return False
        return True

    def mask(self, img):
        nonzeroLst = img.nonzero()
        nony = np.array(nonzeroLst[0])
        nonx = np.array(nonzeroLst[1])

        cv2.rectangle(img, (140, 350), (550, 370), (255, 0, 0), 4)
        stop_line = ((nonx >= 140) & (nonx <= 550) & (nony >= 350) & (nony <= 370)).nonzero()[0]

        #print("crosswalk : ", len(stop_line))
        if len(stop_line) >= 40:
            self.previous_time = time.time()
            #print('detected')
            return True
        else:
           #print('no detected')
           return False
