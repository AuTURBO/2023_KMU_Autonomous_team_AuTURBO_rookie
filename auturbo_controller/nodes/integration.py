#!/usr/bin/env python

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


class XycarControlNode:
    def __init__(self):
        rospy.init_node("xycar_control_node", anonymous=True)
        rospy.Subscriber("scan", LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber("xycar_angle", Int32, self.angle_callback, queue_size=1)

        self.pub_steering_angle = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

        self.error_sum = 0
        self.prev_error = 0
        self.error = 0
        self.target_angle = 0
        self.obstacle_detected = 0  # 0: No obstacle, -1: Obstacle detected on the left, 1: Obstacle detected on the right

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[:505 // 4] = 0.0
        ranges[505 * 4 // 3:] = 0.0

        quarter = int(len(ranges) / 4)

        right_quarter_ranges = [range_val for range_val in ranges[:2 * quarter] if range_val > 0]
        left_quarter_ranges = [range_val for range_val in ranges[2 * quarter:3 * quarter] if range_val > 0]

        if len(right_quarter_ranges) > 0 and len(left_quarter_ranges) > 0:
            right_quarter_min_distance = min(right_quarter_ranges)
            left_quarter_min_distance = min(left_quarter_ranges)

            if right_quarter_min_distance < left_quarter_min_distance:
                self.target_angle = 30
                self.obstacle_detected = 1
            else:
                self.target_angle = -30
                self.obstacle_detected = -1

        elif len(right_quarter_ranges) > 0:
            self.target_angle = -30
            self.obstacle_detected = -1

        elif len(left_quarter_ranges) > 0:
            self.target_angle = 30
            self.obstacle_detected = 1

        else:
            self.target_angle = 0
            self.obstacle_detected = 0

    def pid_control(self, error):
        global error_sum, prev_error, Kp, Ki, Kd

        self.error_sum += error
        error_diff = error - self.prev_error
        pid_output = Kp * error + Ki * self.error_sum + Kd * error_diff
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

    def publish_steering_angle(self, angle):
        xycar_motor_msg = xycar_motor()
        xycar_motor_msg.angle = int(angle)
        xycar_motor_msg.speed = 4
        self.pub_steering_angle.publish(xycar_motor_msg)
        print("조향각: {}도".format(int(angle)))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = XycarControlNode()
    node.run()
