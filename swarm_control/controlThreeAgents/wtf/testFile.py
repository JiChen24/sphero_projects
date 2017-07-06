# test file

import numpy as np
import scipy as sp

import math
import cvxopt

import swarmParams
from collisionFreeDecentralized import *

def testFun():
	obj = swarmParams.sysParams(4)
	
	obj.initPose()
	obj.goalPose()
	obj.neighborRadius()
	
	# I assigned this based on agent number
	pose = np.matrix([[-1.0,-0.0,-0.0,-2.0],[2.0,0.0,-0.0,4.0]])
	
	obj.getCurrentPose(pose)
	
	map = np.matrix([[-5.0,-5.0,5.0,-5.0],[5.0,-5.0,5.0,5.0],[5.0,5.0,-5.0,5.0],[-5.0,5.0,-5.0,-5.0]])
	
	for i in xrange(obj.n):
		statei = obj.currentPose[:,i]
		
		statesNeighbor = np.array([]).reshape(2,0)
		indNeighbor = np.array([]).reshape(1,0)
		
		for j in xrange(obj.n):
			if j != i:
				statejTmp = obj.currentPose[:,j]
				statejTmp = np.array(statejTmp)
				
				if np.linalg.norm(statejTmp-statei) <= obj.Dneighbor[i]:
					
					print statejTmp
					statesNeighbor = np.hstack([statesNeighbor,statejTmp])
					
					print indNeighbor
					print np.array([j])
					indNeighbor = np.hstack([indNeighbor,np.array([[j]])])
		
		statesNeighbor = np.matrix(statesNeighbor)
		indNeighbor = np.matrix(indNeighbor)
		
		uActuali = actualController(statei,i,statesNeighbor,indNeighbor,obj,map)
		print "uActuali=",i
		print uActuali
		
