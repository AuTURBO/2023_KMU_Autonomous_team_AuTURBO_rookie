
import rospy
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32
import numpy as np
import math

# PID control variables
error_sum = 0
prev_error = 0
Kp = 30
Ki = 0.0
Kd = 0.0

# Pure Pursuit variables
WB = 0.24
Lf = 0.20
diff_angle = 0

#rotation 1일 때 : 왼쪽 장애물이 오른쪽 장애물 보다 가까이 있음
#rotation -1일 때 : 오른쪽 장애물이 왼쪽 장애물 보다 가까이 있음
# 초기값 : 0
rotation = 0
prev_rotation = 0

class integration_node():
    def __init__(self):
        global error_sum, prev_error, error, Kp, Ki, Kd

        rospy.init_node("obstacle_detector_node", anonymous=True)
        self.sub_lidar = rospy.Subscriber("scan", LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber("xycar_angle", Int32, self.angle_callback, queue_size=1)


        xycar_motor_msg = xycar_motor()
        xycar_motor_msg.angle = 0
        xycar_motor_msg.speed = 0
        self.pub_steering_angle = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
        
        self.error_sum = 0  # 오차의 적분 값
        self.prev_error = 0  # 이전 오차 값
        
        #라이다 /scan 토픽 값을 이용하여 오차 계산
        self.error = 0

        self.rotation = 0
        self.prev_rotation = 0

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
    

    def angle_callback(self, msg):
            current_angle = msg.data

            if self.obstacle_detected == 0:
                self.diff_angle = self.target_angle - current_angle
                self.diff_angle = self.diff_angle * math.pi / 180

                delta = math.atan2(2.0 * WB * math.sin(self.diff_angle), Lf)
                delta = delta * 180 / math.pi

                self.publish_steering_angle(-1 * delta)
            else:
                if self.obstacle_detected == -1:
                    self.publish_steering_angle(-1* delta + 30)
                elif self.obstacle_detected == 1:
                    self.publish_steering_angle(-1*delta - 30)



    def lidar_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
         

        quarter = num_ranges // 2


        # 오른쪽의 장애물의 최소 거리 계산
        right_quarter_ranges = [range_val for range_val in ranges[:quarter] if range_val > 0]
        if len(right_quarter_ranges) > 0:
            right_quarter_min_distance = min(right_quarter_ranges)
            print("right min distnace: {} cm".format(right_quarter_min_distance * 100))
        else:
            print("right, no obstacle.")
            right_quarter_min_distance = 0
            

        # 왼쪽의 최소 거리 계산
        left_quarter_ranges = [range_val for range_val in ranges[quarter:] if range_val > 0]
        if len(left_quarter_ranges) > 0:
            left_quarter_min_distance = min(left_quarter_ranges)
            print("left min distance: {} cm".format(left_quarter_min_distance * 100))
        else:
            print("left, no obstacle.")
            left_quarter_min_distance = 0




            
        if left_quarter_min_distance > right_quarter_min_distance:
            self.rotation = -1
        else:
            self.rotation = 1

        # 오차 계산
        self.error -= 0.5 - right_quarter_min_distance
        self.error += 0.5 - left_quarter_min_distance

        # 태그 설정
        if self.rotation != self.prev_rotation:
            self.error = 0
        self.prev_rotation = self.rotation

        # 장애물이 없을 때 0 조향각으로 초기화
        if len(right_quarter_ranges) == 0 and len(left_quarter_ranges) == 0:
            steer = 0
            self.error = 0
        else:
            steer = self.pid_control(self.error)

        if steer > 30:
            steer = 30
        elif steer < -30:   
            steer = -30



        #조향각 출력하기
        print("rotation : {}".format(self.rotation))
        print("angle_steer: {} ".format(int(steer)))


    
    #조향각 발행하기
    def publish_steering_angle(self, angle):
            xycar_motor_msg = xycar_motor()
            xycar_motor_msg.angle = int(angle)
            xycar_motor_msg.speed = 4
            self.pub_steering_angle.publish(xycar_motor_msg)
            print("조향각: {}도".format(int(angle)))

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = obstacle_detector_node()
    node.run()