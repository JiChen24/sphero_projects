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

# collision avoidance
import swarmParams
from collisionFreeDecentralized import *


odometry = None
init_odom = None
theta_cal=0

MAP = numpy.matrix([[-8.0,-8.0,8.0,-8.0],[8.0,-8.0,8.0,8.0],[8.0,8.0,-8.0,8.0],[-8.0,8.0,-8.0,-8.0]])

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


#loc_goal=[]
#loc_goal.append(1)
#loc_goal.append(1)

if __name__=="__main__":
    #parser = argparse.ArgumentParser(description="Test Sphero location")
    #args, unknown = parser.parse_known_args()
    sphero_name = str(sys.argv[1])

    obj = swarmParams.sysParams(8)

    waypoints=numpy.array([ [1.0,1.0] ])

    neighborRadius = 10.0

    size_waypoints=waypoints.shape #[m,n] length and width of waypoints
    num_waypoints=size_waypoints[0] #first entry is length (width is 2-(x,y))
    vel_mag=100

    rospy.init_node(sphero_name+"_test_vel")

    # publish to topics
    pub = rospy.Publisher(sphero_name+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
    rate = rospy.Rate(2) # set publish rate

    # subscribe to odometry data
    rospy.Subscriber(sphero_name+'/odom', Odometry, callback=save_odometry)
    
    #Initialize the sphero as a vicon tracked object
    a1 = vt.ViconTrackerPoseHandler(None, None, "",51017, 'Sphero1')
    a2 = vt.ViconTrackerPoseHandler(None, None, "",51018, 'Sphero2')
    a3 = vt.ViconTrackerPoseHandler(None, None, "",51019, 'Sphero3')
    a4 = vt.ViconTrackerPoseHandler(None, None, "",51020, 'Sphero4')

    b1 = vt.ViconTrackerPoseHandler(None, None, "",51021, 'Sphero5')
    b2 = vt.ViconTrackerPoseHandler(None, None, "",51022, 'Sphero6')
    b3 = vt.ViconTrackerPoseHandler(None, None, "",51023, 'Sphero7')
    b4 = vt.ViconTrackerPoseHandler(None, None, "",51024, 'Sphero8')

    obj.initPose()
    obj.goalPose()
    obj.neighborRadius()

    obj.goalPose[0,0] = waypoints[0,0]
    obj.goalPose[1,0] = waypoints[0,1]

    #Calibrate sphero frame
    
    rate.sleep()
    sphero_theta =0
    startCalib = time.time()
    init_calib_loc = a1.getPose()
    print init_calib_loc[0]
    print init_calib_loc[1]
    while (time.time() - startCalib < 2):
        vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(100,0,0), \
                                          geometry_msgs.msg.Vector3(0,0,0))
        pub.publish(vel_msg)
        rate.sleep()

    vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                          geometry_msgs.msg.Vector3(0,0,0))
    pub.publish(vel_msg)
    fin_calib_loc = a1.getPose()
    print fin_calib_loc[0]
    print fin_calib_loc[1]
    calibVect = numpy.array([fin_calib_loc[0]-init_calib_loc[0],fin_calib_loc[1]-init_calib_loc[1]])
    print calibVect[0]
    print calibVect[1]
    calib_theta = numpy.arctan2(calibVect[1],calibVect[0])
    sphero_theta = calib_theta
    print "Sphero Theta is "+str(180/math.pi*sphero_theta)+"\n"
#loop through the goal points and move to all of them
for ind in range(0,num_waypoints):
#index of first goal is 0, last is num_waypoints-1
    #extract the current goal location
    loc_goal=[]
    loc_goal.append(waypoints[ind][0])
    loc_goal.append(waypoints[ind][1])	

    #get the sphero's current location from vicon
    rate.sleep()
    loc_init=a1.getPose()
    loc=loc_init
    prevLoc = loc
    vect2goal = numpy.array([(loc_goal[0]-loc_init[0]),(loc_goal[1]-loc_init[1])])
    dist2goal = numpy.sqrt(vect2goal.dot(vect2goal))
    start = time.time()

    while (dist2goal > 0.2): # not at goal point
        
        loc = a1.getPose()
        loc2 = a2.getPose()
        loc3 = a3.getPose()
        loc4 = a4.getPose()

        loc5 = b1.getPose()
        loc6 = b2.getPose()
        loc7 = b3.getPose()
        loc8 = b4.getPose()


        statesNeighbor = numpy.array([[loc2[0],loc3[0], loc4[0], loc5[0], loc6[0], loc7[0], loc8[0]],[loc2[1],loc3[1],loc4[1], loc5[1], loc6[1],loc7[1], loc8[1]]])
        statesNeighbor = numpy.matrix(statesNeighbor)

        poseAll = numpy.array([[loc[0],loc2[0],loc3[0], loc4[0], loc5[0], loc6[0],loc7[0], loc8[0]],[loc[1],loc2[1],loc3[1], loc4[1], loc5[1], loc6[1], loc7[1], loc8[1]]])
        poseAll = numpy.matrix(poseAll)
        obj.getCurrentPose(poseAll)

        indNeighbor = numpy.array([1,2,3,4,5,6,7])
        indNeighbor = numpy.matrix(indNeighbor)

        vect2goal = numpy.array([(loc_goal[0]-loc[0]),(loc_goal[1]-loc[1])])
        dist2goal =  numpy.sqrt(vect2goal.dot(vect2goal))
        #Calculate the required vel_msg to reach the goal point from the current location in the vicon frame

        '''theta=numpy.arctan2((vect2goal[1]),(vect2goal[0]))
        v_x_V=math.cos(theta)
        v_y_V=math.sin(theta)
        v_vect_V = numpy.array([[v_x_V],[v_y_V]])

        v_vect_V = numpy.matrix(v_vect_V)

        vect2goal = numpy.matrix(vect2goal)

        print "v_vect_V="
        print v_vect_V

        print "vect2goal="
        print vect2goal'''

        v_vect_V_actual = actualController(poseAll[:,0],0,statesNeighbor,indNeighbor,obj,MAP)
        print "actual_vel="
        print v_vect_V_actual

        v_vect_V_actual = numpy.array([[v_vect_V_actual[0]],[v_vect_V_actual[1]]])

        thetaNew = numpy.arctan2((v_vect_V_actual[1]),(v_vect_V_actual[0]))
        v_x_V_new = vel_mag*math.cos(thetaNew)
        v_y_V_new = vel_mag*math.sin(thetaNew)
        v_vect_V_update = numpy.array([[v_x_V_new],[v_y_V_new]])

        print "actual_vel new is"
        print v_vect_V_update

        R_SV=numpy.array([ [math.cos(sphero_theta),math.sin(sphero_theta)],[-math.sin(sphero_theta), math.cos(sphero_theta)] ])
	# convert global frame to sphero frame
        v_vect_S=numpy.dot(R_SV,v_vect_V_update)
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

        pub.publish(vel_msg)
        rate.sleep()
     

rospy.loginfo("Last Odometry: {0}".format(odometry))
rospy.loginfo("Last Odometry: {0}".format(odometry))
print loc
