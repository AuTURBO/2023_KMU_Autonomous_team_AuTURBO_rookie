#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor


# PID 제어 변수
error = 0

# PID 제어 변수
error_sum = 0  # 오차의 적분 값
prev_error = 0  # 이전 오차 값

# PID 제어 게인 값
Kp = 3  # 비례 제어 게인

# 외란이 없는 시뮬레이션 환경이므로 적분 및 미분 제어는 고려하지 않음
Ki = 0.0  # 적분 제어 게인
Kd = 0.0  # 미분 제어 게인


class obstacle_detector_node:
    def __init__(self):
        global error_sum, prev_error, error, Kp, Ki, Kd

        rospy.init_node("obstacle_detector_node", anonymous=True)
        self.sub_lidar = rospy.Subscriber("scan", LaserScan, self.lidar_callback, queue_size=1)

        xycar_motor_msg = xycar_motor()
        xycar_motor_msg.angle = 0
        xycar_motor_msg.speed = 0
        self.pub_steering_angle = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
        self.error_sum = 0  # 오차의 적분 값
        self.prev_error = 0  # 이전 오차 값
        
        #라이다 /scan 토픽 값을 이용하여 오차 계산
        self.error = 0

    def pid_control(self, error):  # self 매개변수 추가
        global error_sum, prev_error, Kp, Ki, Kd

        # 오차의 적분 값 업데이트
        self.error_sum += error
        # 오차의 변화율 계산
        error_diff = error - self.prev_error
        # PID 제어값 계산
        pid_output = Kp * error + Ki * self.error_sum + Kd * error_diff
        # 이전 오차 값 업데이트
        self.prev_error = error

        return pid_output
    
    def lidar_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        quarter = num_ranges // 2

        # 오른쪽의 최소 거리 계산
        right_quarter_ranges = [range_val for range_val in ranges[:quarter] if range_val > 0]
        if len(right_quarter_ranges) > 0:
            right_quarter_min_distance = min(right_quarter_ranges)
            print("오른쪽 최소 거리: {}cm".format(right_quarter_min_distance * 100))
        else:
            print("오른쪽에 장애물이 없습니다.")
            right_quarter_min_distance = 0
            

        # 왼쪽의 최소 거리 계산
        left_quarter_ranges = [range_val for range_val in ranges[quarter:] if range_val > 0]
        if len(left_quarter_ranges) > 0:
            left_quarter_min_distance = min(left_quarter_ranges)
            print("왼쪽의 최소 거리: {}cm".format(left_quarter_min_distance * 100))
        else:
            print("왼쪽에 장애물이 없습니다.")
            left_quarter_min_distance = 0

        # 오차 계산
        self.error -= right_quarter_min_distance
        self.error += left_quarter_min_distance


        # 장애물이 없을 때 0 조향각으로 초기화
        if len(right_quarter_ranges) == 0 and len(left_quarter_ranges) == 0:
            steer = 0
            self.error = 0
        else:
            steer = self.pid_control(self.error)

        if steer > 15:
            steer = 15
        elif steer < -15:   
            steer = -15


        #조향각 출력하기
        print("조향각: {}도".format(int(steer)))

        #조향각 발행하기
        xycar_motor_msg = xycar_motor()
        xycar_motor_msg.angle = int(steer)
        xycar_motor_msg.speed = 4
        self.pub_steering_angle.publish(xycar_motor_msg)


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = obstacle_detector_node()
    node.run()

