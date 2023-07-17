import cv2
import numpy as np
import time

class Stop_Counter:
    def __init__(self):
        self.cnt = 0
        #self.previous_time = time.time() + 5
        
        # Driving Studio
        self.lower_yellow = (15, 0, 150)
        self.upper_yellow = (30, 255, 200)
	#self.lower_yellow = (20, 50, 150)
        #self.upper_yellow = (32, 190, 200)


        # 2 floor
	#self.lower_yellow = (20, 110, 70)
        #self.upper_yellow = (35, 200, 130)

	self.st_yellow_count = 0


    def check_stop_line(self, img):
        #if time.time() < self.previous_time + 10:
         #   return False

        img = img[400:] #348
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(img_hsv)
        # 15 < h < 30
        # 50 < s < 160
        # 150 < v < 210
        mask = cv2.inRange(img_hsv, self.lower_yellow, self.upper_yellow)

        st_yellow_count = np.count_nonzero(mask)

        cv2.putText(mask, 'YELLOW PIXEL %d' % st_yellow_count, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # ------------------------------------------------------------------- #
        # Before active python Stop_Counter_Test.py
        #cv2.imshow("h",h)
        #cv2.imshow("s",s)
        #cv2.imshow("v",v)
        #cv2.imshow("stop_img", mask)
        #cv2.waitKey(100)
        # ------------------------------------------------------------------- #

        # 3000
        if st_yellow_count > 4300:
            self.on_detected()
            return True

        return False

    def on_detected(self):
        print('-----------------------------STOP LINE DETECTED-----------------------------')
        #self.previous_time = time.time()
        self.cnt += 1

# python Stop_Counter_Test.py
if __name__ == '__main__':
    stop_counter = Stop_Counter()

    cap = cv2.VideoCapture(0)
    #cap = cv2.VideoCapture("test_video/start_line.avi")
    while cap.isOpened():
        ret, frame = cap.read()
        stop_counter.check_stop_line(frame)

        cv2.imshow('frame', frame)
        #cv2.imshow('hsv',img_hsg)
        if cv2.waitKey(1) == ord('q'):
           break
