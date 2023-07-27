import rospy
from std_msgs.msg import Int32
import math


class PurePursuitNode:
    def __init__(self):
        rospy.init_node("pure_pursuit_node", anonymous=True)
        rospy.Subscriber("xycar_angle", Int32, self.angle_callback, queue_size=10)
        self.pub_steering_angle = rospy.Publisher("steering_angle", Int32, queue_size=10)

        # 경로 좌표를 설정합니다. (경로 좌표는 필요에 따라 변경 가능합니다)
        self.path_points = [(0, 0), (1, 1), (2, 3), (4, 6), (5, 7), (6, 8)]

    def angle_callback(self, msg):
        current_angle = msg.data
        # Pure Pursuit 알고리즘을 구현합니다.
        # 경로 상에서 가장 가까운 지점과의 거리를 계산하여 해당 지점까지의 조향각을 계산합니다.

        # 현재 위치를 받아옵니다. (현재 위치는 필요에 따라 다른 방식으로 받아올 수 있습니다.)
        current_pose = (0, 0)  # (x, y) 형태로 가정합니다.

        # 현재 위치와 경로 상 가장 가까운 지점을 계산합니다.
        closest_point = self.find_closest_point(current_pose)

        # 가장 가까운 지점까지의 거리를 계산합니다.
        distance_to_closest_point = self.calculate_distance(current_pose, closest_point)

        # 목표로 할 지점의 앞에 있는 지점을 계산합니다.
        target_point = self.find_target_point(closest_point, distance_to_closest_point)

        # 차량과 목표 지점과의 각도를 계산합니다.
        target_angle = self.calculate_target_angle(current_pose, target_point)

        # 현재 조향각과 목표 각도와의 차이를 계산합니다.
        diff_angle = target_angle - current_angle

        # 계산된 조향각을 메시지에 담아 발행합니다.
        steering_msg = Int32()
        steering_msg.data = diff_angle
        self.pub_steering_angle.publish(steering_msg)

    def find_closest_point(self, current_pose):
        # 경로 상에서 가장 가까운 지점을 계산합니다.
        closest_point = None
        min_distance = float("inf")
        for point in self.path_points:
            distance = self.calculate_distance(current_pose, point)
            if distance < min_distance:
                closest_point = point
                min_distance = distance
        return closest_point

    def calculate_distance(self, pose1, pose2):
        # 두 위치 사이의 거리를 계산합니다.
        x1, y1 = pose1
        x2, y2 = pose2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def find_target_point(self, closest_point, distance):
        # 현재 위치와 경로 상 가장 가까운 지점과의 거리를 기준으로 목표 지점을 계산합니다.
        target_distance = 2.0  # 목표 지점과의 거리를 설정합니다.
        for i in range(len(self.path_points)):
            point = self.path_points[i]
            if point == closest_point:
                break

        # 경로 상에서 현재 지점으로부터 거리가 target_distance인 지점을 찾습니다.
        while i < len(self.path_points):
            next_point = self.path_points[i]
            distance += self.calculate_distance(point, next_point)
            if distance >= target_distance:
                target_point = next_point
                break
            point = next_point
            i += 1
        else:
            target_point = self.path_points[-1]  # 경로 상 마지막 지점을 목표 지점으로 설정합니다.
        return target_point

    def calculate_target_angle(self, current_pose, target_point):
        # 차량과 목표 지점과의 각도를 계산합니다.
        x1, y1 = current_pose
        x2, y2 = target_point
        return self.rad_to_degrees(math.atan2(y2 - y1, x2 - x1))

    def rad_to_degrees(self, rad):
        # 라디안을 각도로 변환합니다.
        return rad * 180.0 / math.pi

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = PurePursuitNode()
    node.run()
