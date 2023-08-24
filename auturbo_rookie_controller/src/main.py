#! /usr/bin/env python
# -*- coding:utf-8 -*-
import sys
import rospy
import signal  # Import the signal module
from Xycar import Xycar

mode_dict = {
    '0': 'long straight',
    '1': 'findparallelparking',
    '2': 'ar_curve',
    '3': 'findverticalparking',
    '4': 'obstacle',
    '5': 'stopline',
    '6': 'rubbercon',
    '10': 'zgzg'
}

def cleanup(signal, frame):
    rospy.loginfo("Shutting down robot controller...")
    # Add any cleanup operations you need here
    rospy.signal_shutdown("Ctrl+C interrupt")
    rospy.loginfo("Robot controller has shut down.")
    exit(0)

def main(mode='0'):
    rospy.init_node('AuTOBO_rookie', log_level=rospy.DEBUG)
    rospy.loginfo("Initializing robot controller...")

    xycar = Xycar(mode_dict[mode])

    # Register the cleanup function for Ctrl+C signal
    signal.signal(signal.SIGINT, cleanup)

    while not rospy.is_shutdown():
        xycar.control()

if __name__ == "__main__":
    if len(sys.argv) != 1:
        main(sys.argv[1])
    else:
        main()
