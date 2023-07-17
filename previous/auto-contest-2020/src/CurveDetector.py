import time


class CurveDetector:
    def __init__(self):
        self.curve_count = 0
        self.time_old = 0
        self.pid_list = [0.0 for i in range(30)]
	self.pid_sum_list = []
    def check_time(self):
        if time.time() - self.time_old < 4:
            return False
        else:
            return True

    def list_update(self, pid):
        self.pid_list.pop(0)
        self.pid_list.append(pid)

    def count_curve(self):
        print("len(pid_list) : ", len(self.pid_list))
        print("sum(self.pid_list) : ", sum(self.pid_list))
	self.pid_sum_list.append(sum(self.pid_list))
        
	if self.check_time():
            #if sum(self.pid_list) > 37:
	    #if sum(self.pid_list) > 3.4:
	    if sum(self.pid_list) > 4.5:
                #print("sum(self.pid_list) : ", sum(self.pid_list))
                self.time_old = time.time()
                #self.curve_count = 2
		self.curve_count += 1
        return sum(self.pid_list)
