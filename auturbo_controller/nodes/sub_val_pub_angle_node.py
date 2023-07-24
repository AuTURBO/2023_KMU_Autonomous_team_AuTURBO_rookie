import rospy
from std_msgs.msg import String, Int32, Float32

class 패키지이름Node:
    def __init__(self):
        print('Hello package')
        rospy.Subscriber("distance_to_center", Float32, self.메세지콜백, queue_size=10)
        self.pub_steering_angle = rospy.Publisher("steering_angle", Float32, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
    def timer_callback(self, event):
        # 여기서 pure pursuit 알고리즘을 구현합니다.
        # distance_to_center를 이용하여 조향각을 계산합니다.
        # 예를 들어, 간단히 현재 거리에 0.1을 더한 값을 조향각으로 사용하겠습니다.


        #카메라 
        
        steering_angle = self.current_distance + 0.1

        # 계산된 조향각을 메시지에 담아 발행합니다.
        msg = Float32()
        msg.data = steering_angle
        self.pub_steering_angle.publish(msg)
    
    def 메세지콜백(self, msg):
        # 구독한 메시지인 distance_to_center를 클래스 변수로 저장합니다.
        self.current_distance = msg.data
    
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('패키지이름_node')
    node = 패키지이름Node()
    node.main()
