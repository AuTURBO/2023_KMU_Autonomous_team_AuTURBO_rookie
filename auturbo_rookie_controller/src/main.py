#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import signal  # Import the signal module
from Xycar import Xycar

def cleanup(signal, frame):
    rospy.loginfo("Shutting down robot controller...")
    # Add any cleanup operations you need here
    rospy.signal_shutdown("Ctrl+C interrupt")
    rospy.loginfo("Robot controller has shut down.")
    exit(0)

rospy.init_node('AuTOBO_rookie', log_level=rospy.DEBUG)
rospy.loginfo("Initializing robot controller...")

xycar = Xycar()

# Register the cleanup function for Ctrl+C signal
signal.signal(signal.SIGINT, cleanup)

while not rospy.is_shutdown():
    xycar.control()