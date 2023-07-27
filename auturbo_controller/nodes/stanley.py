import tkinter as tk
from std_msgs.msg import Int32
import rospy
from PIL import Image, ImageTk
import math
import numpy as np
from xycar_msgs.msg import xycar_motor


class StanleyNode:
    def __init__(self):
        rospy.init_node("stanlet_node", anonymous=True)
        rospy.Subscriber("xycar_angle", Int32, self.angle_callback, queue_size=10)

        self.pub_steering_angle = rospy.Publisher("xycar_cmd", xycar_motor, queue_size=10)

        # Tkinter 창 설정
        self.root = tk.Tk()
        self.root.title("Yaw Value Simulator")
        # 게인값 설정
        self.k = 0.1
        # 전방 주시거리 설정
        self.L = 0.3
        # 캔버스 생성
        self.canvas = tk.Canvas(self.root, width=800, height=600)
        self.canvas.pack()

        # 이미지 로드
        self.car_image = Image.open("car.png")
        self.car_photo = ImageTk.PhotoImage(self.car_image)

        # 이미지 초기 위치
        self.car_x, self.car_y = 400, 300

        # 라벨을 위한 변수와 라벨 생성
        self.diff_angle = 0
        self.label = tk.Label(self.root, text="Yaw Value: {}".format(self.diff_angle))
        self.label.pack(pady=10)

    def draw_car(self, x, y, angle):
        # 이미지 회전을 위해 이미지 복사
        rotated_image = self.car_image.copy()
        rotated_image = rotated_image.rotate(angle)

        # 이미지 크기 조정
        size = (rotated_image.width // 2, rotated_image.height // 2)
        rotated_image.thumbnail(size, Image.LANCZOS)

        # 이미지 업데이트
        self.car_photo = ImageTk.PhotoImage(rotated_image)
        self.canvas.create_image(x, y, image=self.car_photo)

    previous_angle = 0

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def angle_callback(self, msg):
        current_angle = msg.data
        # 여기에 Pure Pursuit 알고리즘을 구현합니다.
        # 현재 조향각인 current_angle과 목표값(0으로 가정)을 이용하여 알고리즘을 적용합니다.
        # 다음 줄을 알고리즘에 맞게 수정해주세요.
        target_angle = -5  # 여기에 Pure Pursuit 알고리즘을 적용하여 목표 조향각을 계산합니다.

        # 현재 조향각과 목표 조향각과의 차이를 계산합니다.
        self.diff_angle = target_angle - current_angle

        # 디그리 투 라디안
        self.diff_angle = self.diff_angle * math.pi / 180

        # yaw : 현재 차량의 각도, map_yaw : 목표 지점의 각도, dx, dy : 차량과 목표 지점의 거리
        perp_vec = [np.cos(self.diff_angle + np.pi / 2), np.sin(self.diff_angle + np.pi / 2)]
        #  Cross Track Error (CTE)를 계산합니다. cte는 현재 차량의 위치와 주행 경로 사이의 수직 거리를 나타냅니다.
        # Dx만 넣어도 무방함
        # cte = np.dot([dx, dy], perp_vec)
        cte = self.L * math.cos(self.diff_angle)

        # a = L*cos(theta) : 목표 지점과 차량의 거리
        # a = L * math.cos(self.diff_angle)

        # control law
        # Yaw (heading angle) 오차를 계산합니다. -5는 원하는 yaw 오차를 나타내며, 현재 yaw와의 차이를 계산합니다.
        yaw_term = self.normalize_angle(-5 - self.diff_angle)
        # CTE에 대한 제어 값을 계산합니다. k는 조절 상수이며, k * cte를 속도 v로 나눈 값의 아크탄젠트를 계산합니다.
        v = 10
        cte_term = np.arctan2(self.k * cte, v)

        # steering
        steer = yaw_term + cte_term

        # 계산된 조향각을 디그리 투 라디안
        delta = steer * 180 / math.pi

        # 계산된 조향각을 메시지에 담아 발행합니다.
        xycar_motor_msg = xycar_motor()
        xycar_motor_msg.angle = int(delta)

        self.pub_steering_angle.publish(xycar_motor_msg)

        print("steering_msg.data: ", xycar_motor_msg.angle)

        # 라벨 업데이트
        # 이전 값과 받은 값이 다를 시 업데이트
        if self.previous_angle != xycar_motor_msg.angle:
            self.label.config(text="Yaw Value: {}".format(xycar_motor_msg.angle))
            self.previous_angle = xycar_motor_msg.angle

            # 캔버스 클리어
            self.canvas.delete("all")
            # 차량 이미지 그리기
            self.draw_car(self.car_x, self.car_y, -1 * xycar_motor_msg.angle)

        # 경로 그리기
        # 경로를 그리는 로직을 추가하세요.

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    node = StanleyNode()
    node.run()
