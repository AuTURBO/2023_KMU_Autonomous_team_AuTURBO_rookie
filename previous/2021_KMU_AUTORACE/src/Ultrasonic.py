class Ultrasonic:
    def __init__(self):
        self.br = 0
        self.b = 0
        self.bl = 0

    def ultra_data(self, data):
        self.br = data[5]
        self.b = data[6]
        self.bl = data[7]

    def ultra_steer():
        global ultra
        # 40
        if self.b < 22 or self.bl < 22:
            return True
        else:
            return False
