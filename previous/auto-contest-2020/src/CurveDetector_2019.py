import time


class CurveDetector:
    def __init__(self):
        self.curve_count = 0
        self.time_old = 0
        self.pid_list = [0.0 for i in range(30)]

    def check_time(self):
        if time.time() - self.time_old < 3:
            return False
        else:
            return True

    def list_update(self, pid):
        self.pid_list.pop(0)
        self.pid_list.append(pid)

    def count_curve(self):
        if self.check_time():
            if sum(self.pid_list) > 2:
                self.time_old = time.time()
                self.curve_count += 1
