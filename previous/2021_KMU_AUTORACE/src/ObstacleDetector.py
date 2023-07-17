import enum
import math

class Position(enum.Enum):
    ON = 0
    LEFT = 1
    RIGHT = 2
    NO = 3

class ObstacleDetector:
    def __init__(self):
        self.mode = Position.NO
        self.previous_time = 0

    def mode_on(self, obstacles):
        for circle in obstacles.circles:
            c = circle.center
            if -0.5 <= c.y < 0:
                if abs(c.x) < 0.2:
                    self.mode = Position.ON

        return self.mode

    def check(self, obstacles):
        center = None
        for circle in obstacles.circles:
            c = circle.center
            p = circle.center
            if -1.0 < c.y < -0.05:
                if abs(c.x) < 0.5:
                    #print("ok")
                    if c.x >= 0:
                        self.mode = Position.RIGHT
                    elif c.x < 0:
                        self.mode = Position.LEFT
                    break
                else:
                    #print("no")
                    self.mode = Position.NO
                    #print("NO1")
            else:
                continue

        return self.mode

obstacles = None

def obstacle_callback(data):
    global obstacles
    obstacles = data
