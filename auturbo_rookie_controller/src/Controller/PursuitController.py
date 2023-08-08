import math

class PurePursuitController(object):
    def __init__(self):
        self.WB = 0.24
        self.diff_angle = 0
        self.speed = 4
        self.timer = timer

        self.Lf = {
            'long straight': 0.15,
            'short straight': 0.15,
            'curve': 0.15,
            'findparking': 0.15,
            'stopline': 0.15,
            'obstacle': 0.15
        }
        self.target_speed = {
            'long straight': 4,
            'short straight': 4,
            'curve': 4,
            'findparking': 4,
            'stopline': 4,
            'obstacle': 4
        }
        self.acc = {
            'long straight': 0.5,
            'short straight': 0.5,
            'curve': 0.5,
            'findparking': 0.5,
            'stopline': 0.5,
            'obstacle': 0.5
        }
        self.delay = {
            'long straight': 0.5,
            'short straight': 0.5,
            'curve': 0.5,
            'findparking': 0.5,
            'stopline': 0.5,
            'obstacle': 0.5
        }


    def __call__(self, target, mode):

        if self.timer < self.delay[mode]:
            if self.acc[mode] is None:
                self.speed = self.target_speed[mode]
            # 가속도가 0보다 크면
            elif self.acc[mode] > 0:
                self.speed = min(self.speed + self.acc[mode] * self.timer, self.target_speed[mode])
            # 가속도가 0보다 작으면 
            else:
                self.speed = max(self.speed + self.acc[mode] * self.timer, self.target_speed[mode])

            

        current_angle = target
        # 여기에 Pure Pursuit 알고리즘을 구현합니다.
        # 현재 조향각인 current_angle과 목표값(0으로 가정)을 이용하여 알고리즘을 적용합니다.
        # 다음 줄을 알고리즘에 맞게 수정해주세요.
        target_angle = 0  # -5  # 여기에 Pure Pursuit 알고리즘을 적용하여 목표 조향각을 계산합니다.

        # 현재 조향각과 목표 조향각과의 차이를 계산합니다.
        self.diff_angle = target_angle - current_angle

        # 디그리 투 라디안
        self.diff_angle = self.diff_angle * math.pi / 180

        delta = math.atan2(2.0 * self.WB * math.sin(self.diff_angle), self.Lf[mode])

        # 계산된 조향각을 디그리 투 라디안
        delta = -1 * delta * 180 / math.pi
        return int(delta), int(self.speed)

         
