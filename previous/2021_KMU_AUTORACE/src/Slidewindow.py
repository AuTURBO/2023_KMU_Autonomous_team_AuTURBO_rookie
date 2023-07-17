import cv2
import numpy as np

class Line_Detection:
    def __init__(self):
        self.xl_previous = 0
        self.xr_previous = 0
        self.loc_previous = 0
        self.Offset = 330

    def draw_line(self, img, left_line, right_line, line_flag, line_type, nonx, nony):

        min_pix = 5

        window_height = 10
        window_width = 15

        wcnt = 8

        rectangle_height = 5
        rectangle_width = 5

        #draw line independently(see both line)
        if line_flag == 1:

            for i in range(0, wcnt):
                left_wx_low = left_line - window_width
                left_wx_high = left_line + window_width
                right_wx_low = right_line - window_width
                right_wx_high = right_line + window_width
                wy_low = self.Offset + 10 - (i + 1) * window_height
                wy_high = self.Offset + 10 - i * window_height

                left_good_line = ((nony >= wy_low) & (nony < wy_high) &
                        (nonx >= left_wx_low) & (nonx < left_wx_high)).nonzero()[0]
                right_good_line = ((nony >= wy_low) & (nony < wy_high) &
                        (nonx >= right_wx_low) & (nonx < right_wx_high)).nonzero()[0]

                if len(left_good_line) > min_pix:
                        left_line = np.int(np.mean(nonx[left_good_line]))
                if len(right_good_line) > min_pix:
                        right_line = np.int(np.mean(nonx[right_good_line]))

                cv2.rectangle(img, (left_wx_low, wy_low), (left_wx_high, wy_high), (255, 0, 0), 2)
                cv2.rectangle(img, (right_wx_low, wy_low), (right_wx_high, wy_high), (255, 0, 0), 2)

            return img

        #left dependent
        if line_type == -1:

            for i in range(0, wcnt):
                wx_low = left_line - window_width
                wx_high = left_line + window_width
                wy_low = self.Offset + 10 - (i + 1) * window_height
                wy_high = self.Offset + 10 - i * window_height

                good_line = ((nony >= wy_low) & (nony < wy_high) &
                        (nonx >= wx_low) & (nonx < wx_high)).nonzero()[0]
                if len(good_line) > min_pix:
                        left_line = np.int(np.mean(nonx[good_line]))

                cv2.rectangle(img, (wx_low, wy_low), (wx_high, wy_high), (255, 0, 0), 2)
                cv2.rectangle(img, (wx_low + 270, wy_low), (wx_high + 270, wy_high), (255, 0, 0), 2)

            rx_low = left_line - rectangle_height
            rx_high = left_line + rectangle_height
            ry_low = wy_high - 25 - rectangle_width
            ry_high = wy_high - 25 + rectangle_width

            cv2.rectangle(img, (rx_low, ry_low), (rx_high, ry_high), (0, 255, 0), 2)
            cv2.rectangle(img, (rx_low + 270 , ry_low), (rx_high + 270, ry_high), (0, 255, 0), 2)
            cv2.rectangle(img, (rx_low + 135, ry_low), (rx_high + 135, ry_high), (0, 255, 0), 2)

            return img

        #right dependent
        elif line_type == 1:
            for i in range(0, wcnt):
                wx_low = right_line - window_width
                wx_high = right_line + window_width
                wy_low = self.Offset+10 - (i + 1) * window_height
                wy_high = self.Offset+10 - i * window_height

                good_line = ((nony >= wy_low) & (nony < wy_high) &
                        (nonx >= wx_low) & (nonx < wx_high)).nonzero()[0]
                if len(good_line) > min_pix:
                    right_line = np.int(np.mean(nonx[good_line]))

                cv2.rectangle(img, (wx_low - 270, wy_low), (wx_high - 270, wy_high), (255, 0, 0), 2)
                cv2.rectangle(img, (wx_low, wy_low), (wx_high, wy_high), (255, 0, 0), 2)

            rx_low = right_line - rectangle_height
            rx_high = right_line + rectangle_height
            ry_low = wy_high - 25 - rectangle_width
            ry_high = wy_high - 25 + rectangle_width

            cv2.rectangle(img, (rx_low - 270, ry_low), (rx_high - 270, ry_high), (0, 255, 0), 2)
            cv2.rectangle(img, (rx_low, ry_low), (rx_high, ry_high), (0, 255, 0), 2)
            cv2.rectangle(img, (rx_low - 135, ry_low), (rx_high - 135, ry_high), (0, 255, 0), 2)

            return img

    def line_detect(self, img, a, b):

        nonzeroLst = img.nonzero()
        nony = np.array(nonzeroLst[0])
        nonx = np.array(nonzeroLst[1])

        left_lines = ((nonx >= 70 + a + b) & (nonx <= 170 + a + b) & (nony >= 270) & (nony <= 300)).nonzero()[0]
        right_lines = ((nonx >= 330 + a + b) & (nonx <= 430 + a + b) & (nony >= 270) & (nony <= 300)).nonzero()[0]

        cv2.rectangle(img, (70 + a + b, 270), (170 + a + b, 300), (255, 0, 0), 2)
        cv2.rectangle(img, (330 + a + b, 270), (430 + a + b, 300), (255, 0, 0), 2)

        left_pix = len(left_lines)
        right_pix = len(right_lines)
        #print('left_pix :', left_pix)
        #print('right_pix :', right_pix)

        min_pix = 100

        dflag = None

        if (left_pix >= min_pix) & (right_pix >= min_pix):
            dflag = 1

        elif left_pix >= min_pix:
            dflag = 2

        elif right_pix >= min_pix:
            dflag = 3

        xl_current = 0
        xr_current = 0

        xl_difference = 0
        xr_difference = 0

        ltype = 0
        loc = 0

        show_img = 0

        c_loc_dependent = 0
        c_loc_independent = 0

        if dflag:
            #see both
            if dflag == 1:
                xl_current = np.int(np.mean(nonx[left_lines]))
                xr_current = np.int(np.mean(nonx[right_lines]))

                xl_difference = abs(xl_current - self.xl_previous)
                xr_difference = abs(xr_current - self.xr_previous)

                #ltype = -1 : left, ltype = 1 : right
                if xl_difference >= xr_difference:
                    ltype = 1

                else:
                    ltype = -1

                if ltype == -1:
                    l_loc = xl_current
                    r_loc = xr_current
                    c_loc_dependent = l_loc + 135
                    c_loc_independent = (l_loc + r_loc) / 2
                    show_img = self.draw_line(img, l_loc, r_loc, dflag, ltype, nonx, nony)
                    self.xl_previous = l_loc
                    self.xr_previous = r_loc
                    self.loc_previous = l_loc

                else:
                    l_loc = xl_current
                    r_loc = xr_current
                    c_loc_dependent = r_loc - 135
                    c_loc_independent = (l_loc + r_loc) / 2
                    show_img = self.draw_line(img, l_loc, r_loc, dflag, ltype, nonx, nony)
                    self.xl_previous = l_loc
                    self.xr_previous = r_loc
                    self.loc_previous = r_loc

            #only see left
            elif dflag == 2:
                ltype = -1
                xl_current = np.int(np.mean(nonx[left_lines]))
                l_loc = xl_current
                r_loc = l_loc + 270
                c_loc_dependent = l_loc + 135
                c_loc_independent = l_loc + 135
                show_img = self.draw_line(img, l_loc, r_loc, dflag, ltype, nonx, nony)
                self.xl_previous = l_loc
                self.xr_previous = r_loc
                self.loc_previous = l_loc

            #only see right
            elif dflag == 3:
                ltype = 1
                xr_current = np.int(np.mean(nonx[right_lines]))
                r_loc = xr_current
                l_loc = r_loc - 270
                c_loc_dependent = r_loc - 135
                c_loc_independent = r_loc - 135
                show_img = self.draw_line(img, l_loc, r_loc, dflag, ltype, nonx, nony)
                self.xl_previous = l_loc
                self.xr_previous = r_loc
                self.loc_previous = r_loc

        #can't see
        else:
            xl_current = self.xl_previous
            xr_current = self.xr_previous
            loc = self.loc_previous

            if xl_current == loc:
                ltype = -1
                l_loc = xl_current
                r_loc = l_loc + 270
                c_loc_dependent = l_loc + 135
                c_loc_independent = l_loc + 135
                show_img = self.draw_line(img, l_loc, r_loc, dflag, ltype, nonx, nony)
                self.xl_previous = l_loc
                self.xr_previous = r_loc
                self.loc_previous = l_loc

            else:
                ltype = 1
                r_loc = xr_current
                l_loc = r_loc - 270
                c_loc_dependent = r_loc - 135
                c_loc_independent = r_loc - 135
                show_img = self.draw_line(img, l_loc, r_loc, dflag, ltype, nonx, nony)
                self.xl_previous = l_loc
                self.xr_previous = r_loc
                self.loc_previous = r_loc

        return show_img, l_loc, r_loc, c_loc_independent, c_loc_dependent, dflag
