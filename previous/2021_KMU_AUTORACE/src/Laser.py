import numpy as np

class Laser:
    def __init__(self):
        self.init_count = 0
        self.count = 0
        self.count_nonzero = 0


    def roi(self, data):
        data = np.array(data)
        data = data[90: 180]
        for lid in data:
            if lid < 0.55:
                self.count += 1
        self.count_nonzero = self.count
        self.count = self.init_count

        return self.count_nonzero
