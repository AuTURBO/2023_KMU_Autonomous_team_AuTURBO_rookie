import cv2

class RLineDetection:
    def __init__(self):
        self.prev_r = 411

    def roi_detect(self, img):
        value_threshold = 60
        image_width = 640
        scan_width, scan_height = 200, 20
        lmid, rmid = scan_width, image_width - scan_width # 200, 440
        area_width, area_height = 10, 10
        roi_vertical_pos = 300
        row_begin = (scan_height - area_height) // 2 # 5
        row_end = row_begin + area_height # 15
        pixel_cnt_threshold = 0.1 * area_width * area_height
        left, right = -1, -1
        RGB_thres_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        for r in range(rmid - 170, image_width - area_width - 230, 1):
            area = img[300 + row_begin: 300 + row_end, r:r + area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                right = r
                break
            else:
                right = -1

        return RGB_thres_img, right

    def draw_line(self, img, rpos):
        value_threshold = 60
        image_width = 640
        scan_width, scan_height = 200, 20
        lmid, rmid = scan_width, image_width - scan_width # 200, 440
        area_width, area_height = 10, 10
        roi_vertical_pos = 300
        row_begin = (scan_height - area_height) // 2 # 5
        row_end = row_begin + area_height # 15
        pixel_cnt_threshold = 0.1 * area_width * area_height
        left, right = -1, -1

        if rpos != -1:
            rsquare = cv2.rectangle(img,
                                    (rpos, 300 + row_begin),
                                    (rpos + area_width, 300 + row_end),
                                    (0, 255, 0), 3)

            r_roi = cv2.rectangle(img,
                                    (rmid-170, 300),
                                    (image_width - 230, 300 + 20),
                                    (150, 155, 100), 3)
            self.prev_r = rpos

        else:
            r_roi = cv2.rectangle(img,
                                    (rmid-170, 300),
                                    (image_width - 230, 300 + 20),
                                    (150, 155, 100), 3)

            lost_rsquare = cv2.rectangle(img,
                                    (self.prev_r, 300 + row_begin),
                                    (self.prev_r + area_width, 300 + row_end),
                                    (0, 255, 0), 3)
        return self.prev_r
