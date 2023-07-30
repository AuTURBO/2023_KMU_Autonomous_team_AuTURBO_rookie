#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers

ack_msg = xycar_motor()
ack_publisher = None

ar_tag_msg = AlvarMarkers()

def ar_tag_callback(msg):
    global ar_tag_msg
    ar_tag_msg = msg
    #print(ar_tag_msg)

def start():
    global ack_publisher, ar_tag_msg    
    ar_tag_topic_name = "/ar_pose_marker"

    ack_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber(ar_tag_topic_name, AlvarMarkers, ar_tag_callback)

    rospy.init_node('ar_tag_follwer')

    rate = rospy.Rate(10) # 10hz
    target = 0.43#0.12
    ack_msg.speed = int(5)
    while not rospy.is_shutdown():
        markers_x_point_list = []
        # print(len(ar_tag_msg.markers))
        for i, marker in enumerate(ar_tag_msg.markers):
            #print(f"marker{i}_x : {marker.pose.pose.position.x}")
            markers_x_point_list.append(marker.pose.pose.position.x)
        markers_x_point_list.sort(reverse=True)
        #print(markers_x_point_list)
        
        if len(markers_x_point_list) > 0:
            error = markers_x_point_list[0] - target
            print(f"error: {error}")
            angle = error * 170 
            print(angle)

            #ack_msg.speed = int(15)
            ack_msg.angle = int(angle)

        ack_publisher.publish(ack_msg)

        rate.sleep()



if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass