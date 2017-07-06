#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import argparse
import random
import std_msgs
from std_msgs.msg import ColorRGBA, Float32, Bool
from nav_msgs.msg import Odometry
import time
import math
import numpy
import ViconTrackerPoseHandler as vt
import sys

if __name__=="__main__":
    #parser = argparse.ArgumentParser(description="Test Sphero location")
    #args, unknown = parser.parse_known_args()
    sphero_name = str(sys.argv[1])
    rospy.init_node(sphero_name+"_color")

    # publish to topics
    pub = rospy.Publisher(sphero_name+'/set_color', ColorRGBA, queue_size=1, latch=True)
    rate = rospy.Rate(3) # set publish rate
    msg = std_msgs.msg.ColorRGBA(b=255)
    msg2 = std_msgs.msg.ColorRGBA(g=255)
    col_msg = msg2
    flash = True
    start = time.time()
    while True:
    	if True:
	    	if flash:
	    		col_msg=msg
	    		flash = False
	    		print "blue"
	    	else:
	    		col_msg=msg2
	    		flash = True
	    		print "green"
	    	start = time.time()
    	rate.sleep()
    	pub.publish(col_msg)

