import cv2
import numpy as np
import time
from SlidingWindow import SlidingWindow

slidingwindow = SlidingWindow()

class Crosswalk_Counter:
    def __init__(self):
        self.cw_cnt = 0

    def check_crosswalk(self, img):
        if slidingwindow.cw_sum >= 4700:#5200
            self.cw_on_detected()
            return True
        return False

    def cw_on_detected(self):
        print('CROSSWALK LINE DETECTED!!!')
        # self.previous_time = time.time()
        self.cw_cnt += 1

if __name__ == '__main__':
    crosswalk_counter = Crosswalk_Counter()

    cap = cv2.VideoCapture('test_video/cross_walk.avi')
    while cap.isOpened():
        ret, frame = cap.read()
        crosswalk_counter.check_crosswalk(frame)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break
