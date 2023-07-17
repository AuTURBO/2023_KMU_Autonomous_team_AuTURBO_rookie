import cv2
import numpy as np
import matplotlib.pyplot as plt

class SlidingWindow:
    def __init__(self):
        self.cw_sum = 0
        self.left_x_lower = 25
        self.left_x_upper = 230
        self.left_y_lower = 330
	self.left_y_upper = 400

        self.right_x_lower = 480
        self.right_x_upper = 620
        self.right_y_lower = 360
        self.right_y_upper = 400

    def slidingwindow(self, img):
		x_location = None

		out_img = np.dstack((img, img, img))
		out_img = np.uint8(out_img)

		height = img.shape[0]
		width = img.shape[1]

		window_height = 10
		window_num = 15

		nonzero = img.nonzero()

		nonzero_y = np.array(nonzero[0])
		nonzero_x = np.array(nonzero[1])

		margin = 20
		minpix = 10

		left_lane = []
		right_lane = []

		# range of draw line
		#good_left = ((nonzero_x >= 10) & (nonzero_y >= 380) & (nonzero_x <= width/2 - 80)).nonzero()[0]
                good_left = ((nonzero_x >= self.left_x_lower) & (nonzero_y >= nonzero_x*(-0.2)+420) & (nonzero_x <= self.left_x_upper)).nonzero()[0]
		#good_right = ((nonzero_x >= width/2 + 80) & (nonzero_y >= 380) & (nonzero_x <= width-10)).nonzero()[0]
                good_right = ((nonzero_x >= self.right_x_lower) & (nonzero_y >= nonzero_x*(0.2)+420) & (nonzero_x <= self.right_x_upper)).nonzero()[0]

		# draw left square
		square_left = np.array([[self.left_x_lower, height], [self.left_x_lower, self.left_y_lower], [self.left_x_upper, self.left_y_upper], [self.left_x_upper, height]], np.int32)
		cv2.polylines(out_img, [square_left], False, (0,255,0), 1)
		
                # draw right square
		square_right = np.array([[self.right_x_lower, height], [self.right_x_lower, self.right_y_upper], [self.right_x_upper, self.right_y_lower], [self.right_x_upper, height]], np.int32)
		cv2.polylines(out_img, [square_right], False, (255,0,0), 1)
		# draw (0.340)

		draw_line = np.array([[0, 400], [width, 400]], np.int32)
		cv2.polylines(out_img, [draw_line], False, (0,120,120), 1)
		

		self.cw_sum = len(nonzero_x[good_left]) + len(nonzero_x[good_right])

		#line_exist_flag = None
		y_represent = None
		x_represent = None

		#judge good_left and good_right and then make bigger one to criteria
		#if len(good_left) > len(good_right):
		if len(good_left) > 10:
		    flag = 1
		    x_represent = np.int(np.mean(nonzero_x[good_left]))
		    y_represent = np.int(np.mean(nonzero_y[good_left]))
		elif len(good_right) > 10:
		    flag = 2
		    x_represent = np.int(np.mean(nonzero_x[good_right]))
		    y_represent = np.int(np.mean(nonzero_y[good_right]))
                else :
                    return out_img, None

                # visualization--> circle valid inds
                if flag == 1:
                    for i in range(len(good_left)):
                        img = cv2.circle(out_img, (nonzero_x[good_left[i]], nonzero_y[good_left[i]]), 1, (0,255,0), -1)
                    print('nonzero_x[good_left[i]]: ', len(nonzero_x[good_left]))
                elif flag == 2:
                    for i in range(len(good_right)):
                        img = cv2.circle(out_img, (nonzero_x[good_right[i]], nonzero_y[good_right[i]]), 1, (0,0,255), -1)
                    print('nonxero_x[good_right[i]]:', len(nonzero_x[good_right]))


		img = cv2.circle(out_img, (x_represent, y_represent), 15, (0,255,255), -1)

		# window sliding and draw
		for window in range(0, window_num):
		    # left lane
		    if flag == 1:
		        # rectangle x,y range
		        win_y_low = y_represent - (window + 1) * window_height
		        win_y_high = y_represent - (window) * window_height
		        win_x_low = x_represent - margin
		        win_x_high = x_represent + margin
		        # draw rectangle
		        cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
		        cv2.rectangle(out_img, (win_x_low + int(width * 0.54), win_y_low), (win_x_high + int(width * 0.54), win_y_high), (255, 0, 0), 1)

		        good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_x_low) & (nonzero_x < win_x_high)).nonzero()[0]

		        if len(good_left) > minpix:
		            x_represent = np.int(np.mean(nonzero_x[good_left]))
		        elif nonzero_y[left_lane] != [] and nonzero_x[left_lane] != []:
		            p_left = np.polyfit(nonzero_y[left_lane], nonzero_x[left_lane], 3)
		            x_represent = np.int(np.polyval(p_left, win_y_high))

		        if win_y_low >= 380 and win_y_low < 400:
		            x_location = x_represent + 245#150 #0.2
		    else: # right lane
		        win_y_low = y_represent - (window + 1) * window_height
		        win_y_high = y_represent - (window) * window_height
		        win_x_low = x_represent - margin
		        win_x_high = x_represent + margin
		        cv2.rectangle(out_img, (win_x_low - int(width * 0.54), win_y_low), (win_x_high - int(width * 0.54), win_y_high), (0, 255, 0), 1)
		        cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)

		        good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_x_low) & (nonzero_x < win_x_high)).nonzero()[0]

		        if len(good_right) > minpix:
		            x_represent = np.int(np.mean(nonzero_x[good_right]))
		        elif nonzero_y[right_lane] != [] and nonzero_x[right_lane] != []:
		            p_right = np.polyfit(nonzero_y[right_lane], nonzero_x[right_lane], 3)
		            x_represent = np.int(np.polyval(p_right, win_y_high))
		        if win_y_low >= 380 and win_y_low < 400:
		            x_location = x_represent - 245#150 #int(width*0.2) #0.2

		    left_lane.extend(good_left)
		    right_lane.extend(good_right)

		return out_img, x_location
