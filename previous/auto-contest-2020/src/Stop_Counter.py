import cv2
import numpy as np
import time

class Stop_Counter:
    def __init__(self):
        self.cnt = 0
        #self.previous_time = time.time() + 5

        #self.lower_yellow = (120, 20, 20)
        #self.upper_yellow = (250, 145, 33)
	self.st_yellow_count = 0

        # self.lower_yellow = (20, 40, 100)
        # self.upper_yellow = (35, 200, 255)
	    # self.st_yellow_count=0


    def check_stop_line(self, img):
        #if time.time() < self.previous_time + 10:
         #   return False

        img = img[400:] #240
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(img_hsv)
        h = cv2.inRange(h, 15, 30)

        # IMSHOW FOR DEBUG
        #print(np.count_nonzero(h))
        #cv2.putText(h, 'NONZERO %d'%np.count_nonzero(h), (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        #cv2.imshow('img_mask', h)

	st_yellow_count = np.count_nonzero(h)

        if st_yellow_count > 4500:
            self.on_detected()
            return True

        return False

    def on_detected(self):
        print('STOP LINE DETECTED!!!')
        #self.previous_time = time.time()
        self.cnt += 1

if __name__ == '__main__':
    stop_counter = Stop_Counter()

    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        stop_counter.check_stop_line(frame)

        cv2.imshow('frame', frame)
        #cv2.imshow('hsv',img_hsg)
        if cv2.waitKey(1) == ord('q'):
            break
