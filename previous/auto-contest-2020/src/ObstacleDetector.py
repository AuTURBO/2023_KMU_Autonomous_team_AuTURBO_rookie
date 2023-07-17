import time
import enum

class Position(enum.Enum):
    NO = 0
    LEFT = 1
    RIGHT = 2
    #PARK_RIGHT = 3

class ObstacleDetector:
    def __init__(self):
        self.mode = Position.NO
	self.rightX = 0
	self.rightY = 0
	self.frontX = 0
	self.frontY = 0
        self.previous_time = 0
        self.cnt = 0

    def parking_check(self, obstacles):
        for circle in obstacles.circles:
            p = circle.center
	    print("asdasdasd: x: %f	y: %f"%(p.x, p.y))
	    if abs(p.y) <= 0.1:
		if p.x >= 0.1 and p.x <= 0.6:
		    self.rightX = p.x
		    self.rightY = p.y
	    if p.x >= -0.1 and p.x <= 0.1:
		print("0.1============= %f, %f"%(p.x, p.y)) 
		if p.y <= -0.1 and p.y >= -0.9:
		    self.frontX = p.x
		    self.frontY = p.y


    # return EnumClass Position
    def check(self, obstacles):
        for circle in obstacles.circles:
            p = circle.center

	    #print("x: %f	y: %f"%(p.x, p.y))
            if -0.5 < p.y < 0:
                if abs(p.x) < 0.3:
                    if p.x >= 0.05:
                        self.mode = Position.RIGHT
                        self.cnt += 1
                    else:
                        self.mode = Position.LEFT
                        self.cnt += 1
                    
		    break
	    #elif p.x <= 0.2 && abs(p.y) <= 0.1:

            else:
                self.mode = Position.NO
        return self.mode

obstacles = None

def obstacle_callback(data):
    global obstacles
    obstacles = data

if __name__ == '__main__':
    from obstacle_detector.msg import Obstacles
    import rospy
    rospy.init_node('TEST')
    rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
    ob = ObstacleDetector()

    time.sleep(3)

    while not rospy.is_shutdown():
        ob.check(obstacles)
        print(ob.mode)
        time.sleep(0.1)

    print('Done')
