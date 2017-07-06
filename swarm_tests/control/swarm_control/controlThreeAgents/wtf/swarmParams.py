import numpy as np
import scipy as sp
import math


class sysParams(object):
	
	def __init__(self,n):
		self.n = n
		self.inputLim = np.ones(n)
		self.kp = 10.0
		self.Ds = 1.0
		self.DsWall = 0.8
		self.gamma = 0.1
		self.closeEnoughPose = 0.1
		
		# deadlock
		self.deadlockThreInput = 0.08
		self.kDelta = 2.0
		self.controlTurnAng = 3.0*math.pi/4
		
		
		
		
	# get initial poses
	def initPose(self):
		self.theta0 = 0
		self.radius = 2.0
		self.initPose = np.zeros([2,self.n])
		for i in xrange(self.n):
			self.initPose[0,i] = self.radius*math.cos(2*math.pi*i/self.n+self.theta0)
			self.initPose[1,i] = self.radius*math.sin(2*math.pi*i/self.n+self.theta0)
			
		#return self
	
	# get goal poses
	def goalPose(self):
		self.goalPose = np.zeros([2,self.n])
		for i in xrange(self.n):
			if i<=self.n/2-1:
				self.goalPose[:,i] = self.initPose[:,self.n/2+i]
			else:
				self.goalPose[:,i] = self.initPose[:,i-self.n/2]
				
		#return self
		
	# get neighborhood radius
	def neighborRadius(self):
		self.Dneighbor = np.zeros(self.n)
		for i in xrange(self.n):
			betaMax = max(self.inputLim)
			self.Dneighbor[i] = self.Ds+((self.inputLim[i]+betaMax)/self.gamma)**(1.0/3)
			
		#return self
		
	# get current positions
	def getCurrentPose(self,pose):
		self.currentPose = pose
		#return self
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	