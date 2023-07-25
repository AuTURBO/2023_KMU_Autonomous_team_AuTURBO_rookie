import rospy
from std_msgs.msg import Int32
from xycar_msgs.msg import xycar_motor
import math


class PurePursuitNode:
    def __init__(self):
        rospy.init_node("pure_pursuit_node", anonymous=True)
        self.sub_steering_angle = rospy.Subscriber(
            "xycar_angle", Int32, self.angle_callback, queue_size=10
        )
        self.pub_steering_angle = rospy.Publisher("steering_angle", xycar_motor, queue_size=10)
        self.WB = 15
        self.Lf = 30

    def angle_callback(self, msg):
        current_angle = msg.data
        # 여기에 Pure Pursuit 알고리즘을 구현합니다.
        # 현재 조향각인 current_angle과 목표값(0으로 가정)을 이용하여 알고리즘을 적용합니다.
        # 다음 줄을 알고리즘에 맞게 수정해주세요.

        target_angle = -5  # 여기에 Pure Pursuit 알고리즘을 적용하여 목표 조향각을 계산합니다.

        # 현재 조향각과 목표 조향각과의 차이를 계산합니다.
        diff_angle = target_angle - current_angle
        # 디그리 투 라디안
        diff_angle = diff_angle * math.pi / 180

        delta = math.atan2(2.0 * self.WB * math.sin(diff_angle), self.Lf)
        # 계산된 조향각을 메시지에 담아 발행합니다.
        delta = delta * 180 / math.pi
        xycar_motor_msg = xycar_motor()
        xycar_motor_msg.angle = int(delta)
        if xycar_motor_msg.angle > 10:
            xycar_motor_msg.speed = 25
        else:
            xycar_motor_msg.speed = 50
        self.pub_steering_angle.publish(xycar_motor_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = PurePursuitNode()
    node.run()
