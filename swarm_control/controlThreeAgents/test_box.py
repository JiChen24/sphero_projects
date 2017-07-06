#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import argparse
import random
from nav_msgs.msg import Odometry
import time
import sys

odometry = None
init_odom = None

def save_odometry(data):
    global odometry

    # save initial
    if odometry is None:
        global init_odom
        init_odom = data.pose.pose

    odometry =data.pose.pose

if __name__=="__main__":
    # Script takes sphero namespace as an input
    sphero = str(sys.argv[1])
    # Initiate box test node
    rospy.init_node(sphero+"_box_test")

    # publish to topics
    pub = rospy.Publisher(sphero +'/cmd_vel', geometry_msgs.msg.Twist, queue_size=10, latch=True)
    rate = rospy.Rate(10) # set publish rate

    # subscribe to odometry data
    rospy.Subscriber(sphero+'/odom', Odometry, callback=save_odometry)

    # min 5 for lab ground
    # min 30 for lab carpet
    vel_msg_f = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(60,0,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))
    vel_msg_l = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,60,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))
    vel_msg_b = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(-60,0,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))
    vel_msg_r = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,-60,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))

    vel_msgs = [vel_msg_f,vel_msg_l,vel_msg_b,vel_msg_r]
    vel_msg_zero = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))

    #rospy.loginfo("Velocity: {0}".format(vel_msg))
    start = time.time()
    travel_timer = time.time()
    travel_ind = 0;
    while time.time()-start < 120: # not rospy.is_shutdown():
        pub.publish(vel_msgs[travel_ind])

        if time.time()-travel_timer > 2:
            if travel_ind > 2:
                travel_ind = 0
            else:
                travel_ind = travel_ind + 1
            travel_timer = time.time()

    pub.publish(vel_msg_zero)
    rospy.loginfo("Init Odometry: {0}".format(init_odom))
    rospy.loginfo("Odometry: {0}".format(odometry))

    time.sleep(1.5)
    rospy.loginfo("Last Odometry: {0}".format(odometry))

