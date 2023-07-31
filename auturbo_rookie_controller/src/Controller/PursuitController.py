import math

class PurePursuitController(object):
    def __init__(self):
        self.WB = 0.24
        self.Lf = 0.15
        self.diff_angle = 0
        self.speed = 4


    def __call__(self, target):
        current_angle = target
        # 여기에 Pure Pursuit 알고리즘을 구현합니다.
        # 현재 조향각인 current_angle과 목표값(0으로 가정)을 이용하여 알고리즘을 적용합니다.
        # 다음 줄을 알고리즘에 맞게 수정해주세요.
        target_angle = 0  # -5  # 여기에 Pure Pursuit 알고리즘을 적용하여 목표 조향각을 계산합니다.

        # 현재 조향각과 목표 조향각과의 차이를 계산합니다.
        self.diff_angle = target_angle - current_angle

        # 디그리 투 라디안
        self.diff_angle = self.diff_angle * math.pi / 180

        delta = math.atan2(2.0 * self.WB * math.sin(self.diff_angle), self.Lf)

        # 계산된 조향각을 디그리 투 라디안
        delta = -1 * delta * 180 / math.pi
        return int(delta), int(self.speed)

         
