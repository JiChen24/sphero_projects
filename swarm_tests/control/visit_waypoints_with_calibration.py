#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import argparse
import random
from nav_msgs.msg import Odometry
import time
import math
import numpy
import ViconTrackerPoseHandler as vt
import sys

odometry = None
init_odom = None
theta_cal=0

def save_odometry(data):
    global odometry

    # save initial
    if odometry is None:
        global init_odom
        init_odom = data.pose.pose

    odometry =data.pose.pose

#Define the waypoints to be visited and the speed at which to visit them
    # magnitude min 5 for lab ground
    # magnitude min 30 for lab carpet
    # magnitude min 150 for lab carpet with vicon tracker cup
#waypoints=numpy.array([ [1,1], [-1,1] ])
waypoints=numpy.array([ [2,2],[-2,2],[0,0] ])
size_waypoints=waypoints.shape #[m,n] length and width of waypoints
num_waypoints=size_waypoints[0] #first entry is length (width is 2-(x,y))
vel_mag=90

#loc_goal=[]
#loc_goal.append(1)
#loc_goal.append(1)

if __name__=="__main__":
    #parser = argparse.ArgumentParser(description="Test Sphero location")
    #args, unknown = parser.parse_known_args()
    sphero_name = str(sys.argv[1])
    portNum = int(sys.argv[2])
    vtName = str(sys.argv[3])

    rospy.init_node(sphero_name+"_test_vel")

    # publish to topics
    pub = rospy.Publisher(sphero_name+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
    rate = rospy.Rate(2) # set publish rate

    # subscribe to odometry data
    rospy.Subscriber(sphero_name+'/odom', Odometry, callback=save_odometry)
    
    #Initialize the sphero as a vicon tracked object
    a = vt.ViconTrackerPoseHandler(None, None, "",portNum, vtName)

    #Calibrate sphero frame
    
    rate.sleep()
    sphero_theta =0
    startCalib = time.time()
    init_calib_loc = a.getPose()
    print init_calib_loc[0]
    print init_calib_loc[1]
    while (time.time() - startCalib < 2):
        vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(vel_mag,0,0), \
                                          geometry_msgs.msg.Vector3(0,0,0))
        pub.publish(vel_msg)
        rate.sleep()

    vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                          geometry_msgs.msg.Vector3(0,0,0))
    pub.publish(vel_msg)
    fin_calib_loc = a.getPose()
    print fin_calib_loc[0]
    print fin_calib_loc[1]
    calibVect = numpy.array([fin_calib_loc[0]-init_calib_loc[0],fin_calib_loc[1]-init_calib_loc[1]])
    print calibVect[0]
    print calibVect[1]
    calib_theta = numpy.arctan2(calibVect[1],calibVect[0])
    sphero_theta = calib_theta
    print "Sphero Theta is "+str(180/math.pi*sphero_theta)+"\n"

    locTimer = time.time()
#loop through the goal points and move to all of them
for ind in range(0,num_waypoints):
#index of first goal is 0, last is num_waypoints-1
    #extract the current goal location
    loc_goal=[]
    loc_goal.append(waypoints[ind][0])
    loc_goal.append(waypoints[ind][1])	
    print "Going to new waypoint"
    #get the sphero's current location from vicon
    rate.sleep()
    loc_init=a.getPose()
    loc=loc_init
    prevLoc = loc
    vect2goal = numpy.array([(loc_goal[0]-loc_init[0]),(loc_goal[1]-loc_init[1])])
    dist2goal =  numpy.sqrt(vect2goal.dot(vect2goal))
    start = time.time()

    while (dist2goal > 0.2): # not at goal point
        
        loc=a.getPose()
        if (time.time()-locTimer >1):
            print "X: "+str(loc[0])
            print "Y: "+str(loc[1])
            locTimer = time.time()
        vect2goal = numpy.array([(loc_goal[0]-loc[0]),(loc_goal[1]-loc[1])])
        dist2goal =  numpy.sqrt(vect2goal.dot(vect2goal))
        #Calculate the required vel_msg to reach the goal point from the current location in the vicon frame

        theta=numpy.arctan2((vect2goal[1]),(vect2goal[0]))
        v_x_V=vel_mag*math.cos(theta)
        v_y_V=vel_mag*math.sin(theta)
        v_vect_V = numpy.array([[v_x_V],[v_y_V]])

        R_SV=numpy.array([ [math.cos(sphero_theta),math.sin(sphero_theta)],[-math.sin(sphero_theta), math.cos(sphero_theta)] ])
        v_vect_S=numpy.dot(R_SV,v_vect_V)
        v_x_S=v_vect_S[0]
        v_y_S=v_vect_S[1]
        #print v_vect_S

        #Define the vel_msg command to send to the sphero

        #vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(v_vect_S[0],v_vect_S[1],0), \
                                          #geometry_msgs.msg.Vector3(0,0,0))
        vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(v_vect_S[0],v_vect_S[1],0), \
                                          geometry_msgs.msg.Vector3(0,0,0))
        vel_msg_zero = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                          geometry_msgs.msg.Vector3(0,0,0))

        #rospy.loginfo("Velocity: {0}".format(vel_msg))
        if (loc[0] == 0 and loc[1] ==0):
            vel_msg = vel_msg_zero
        pub.publish(vel_msg)
        rate.sleep()
rospy.loginfo("Last Odometry: {0}".format(odometry))
rospy.loginfo("Last Odometry: {0}".format(odometry))
print loc
