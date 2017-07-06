import numpy as np
import scipy as sp

import math

import cvxopt


	
def ZCBFfun(statei,statej,safeDis):

	print statei
	print statej
	
	sTmp1 = statei[0,0]-statej[0,0]
	sTmp2 = statei[0,1]-statej[0,1]
	
	deltaPij = np.matrix([[sTmp1],[sTmp2]])
	
	h = np.linalg.norm(deltaPij)-safeDis
	
	return h
	

def singleNominalController(currentState,goalPosition,kp,uLim):

	if len(currentState) < 2:
		currentState = currentState.transpose()
	
	if len(goalPosition) < 2:
		goalPosition = goalPosition.transpose()
	
	px = currentState[0]
	py = currentState[1]
	
	pxGoal = goalPosition[0]
	pyGoal = goalPosition[1]
	
	vx = kp*(pxGoal-px)
	vy = kp*(pyGoal-py)
	
	if math.fabs(vx) > uLim or math.fabs(vy) > uLim:
		uLarge = max(math.fabs(vx),math.fabs(vy))
		vx = uLim*vx/uLarge
		vy = uLim*vy/uLarge
		
	return (vx,vy)
	
	
def closeStaticPoint(singleState,wall):

	print "singleState="
	print singleState

	x0 = singleState[0,0]
	y0 = singleState[1,0]
	
	print "wall="
	print wall
	
	x1 = wall[0,0]
	y1 = wall[0,1]
	x2 = wall[0,2]
	y2 = wall[0,3]
	
	t = ((x2-x1)*(x0-x1)+(y2-y1)*(y0-y1))/((x2-x1)**2+(y2-y1)**2)
	
	if t < 0:
		xClose = x1
		yClose = y1
	elif t <= 1:
		xClose = x1+t*(x2-x1)
		yClose = y1+t*(y2-y1)
	else:
		xClose = x2
		yClose = y2
		
	point = np.matrix([[xClose],[yClose]])
	
	return point



def quadProgramConstraints(statei,i,statesNeighbor,indNeighbor,sphero,map):
	lb = -sphero.inputLim[i]*np.ones((2,1))
	ub = -lb
	
	if len(statesNeighbor) != 0:
		# get the number of columns of statesNeighbor
		nNeighbor = int(statesNeighbor.shape[1])
		
		print "statesNeighbor="
		print statesNeighbor
		
		A1 = np.zeros((nNeighbor,2))
		b1 = np.zeros((nNeighbor,1))
		
		for jj in xrange(nNeighbor):
			A1[jj,0] = -(statei[0,0]-statesNeighbor[0,jj])
			A1[jj,1] = -(statei[1,0]-statesNeighbor[1,jj])
			
			statej = statesNeighbor[:,jj].transpose()
			hij = ZCBFfun(statei.transpose(),statej,sphero.Ds)
			
			print "indNeighbor="
			print indNeighbor
		
			jjInd = indNeighbor[0,jj].astype(int)
		
			print "jjInd="
			print jjInd
			
			print "inputLim="
			print sphero.inputLim
		
			factor = sphero.inputLim[i]/(sphero.inputLim[i]+sphero.inputLim[jjInd-1])
			
			print b1[[jj],:]
			print b1[jj,:]
			print sphero.gamma
			print hij**3
			print np.linalg.norm(statei.transpose()-statej)
			print factor
			
			b1[[jj],:] = sphero.gamma*(hij**3)*np.linalg.norm(statei.transpose()-statej)*factor
		
			print b1[jj,:]
			
	else:
		A1 = np.matrix([])
		b1 = np.matrix([])
		
	print "A1="
	print A1
	print "b1="
	print b1
		
	
	nWall = len(map[:,0])
	
	A2 = np.zeros((nWall,2))
	b2 = np.zeros((nWall,1))
	
	for jj in xrange(nWall):
		closePoint = closeStaticPoint(statei,map[jj,:])
		
		A2[jj,0] = -(statei[0]-closePoint[0])
		A2[jj,1] = -(statei[1]-closePoint[1])
		
		hij = ZCBFfun(statei.transpose(),closePoint.transpose(),sphero.DsWall)
		
		b2[jj,:] = sphero.gamma*(hij**3)*np.linalg.norm(statei-closePoint)
		
	print "A2="
	print A2
	print "b2="
	print b2
	
	A3 = np.identity(2)
	b3 = ub
	A4 = -np.identity(2)
	b4 = -lb
	
	A = np.vstack((A1,A2,A3,A4))	
	b = np.vstack((b1,b2,b3,b4))
	
	'''for k in xrange(len(A[:,0])):
		ATmp = np.sum(abs(A[k,:]))
		A[k,0] = A[k,0]/ATmp
		A[k,1] = A[k,1]/ATmp
		
		b[k,0] = b[k,0]/ATmp'''
		
	
	print "A="
	print A
	print "b="
	print b
	
	return (A,b)
	

def actualController(statei,i,statesNeighbor,indNeighbor,sphero,map):

	print "statei="
	print statei
	
	(uxTmp,uyTmp) = singleNominalController(statei.transpose(),sphero.goalPose[:,i].transpose(),sphero.kp,sphero.inputLim[i])
	
	uxTmp = np.asscalar(uxTmp)
	uyTmp = np.asscalar(uyTmp)
	
	uNominal = np.matrix([[uxTmp],[uyTmp]])
	
	H = np.identity(2)
	f = -uNominal
	
	(A,b) = quadProgramConstraints(statei,i,statesNeighbor,indNeighbor,sphero,map)
	
	uSol = cvxopt.solvers.qp(cvxopt.matrix(H),cvxopt.matrix(f),cvxopt.matrix(A),cvxopt.matrix(b))
	uActual = uSol['x']
	
	# for deadlock
	'''if np.linalg.norm(statei-sphero.goalPose[:,i]) > 2*sphero.closeEnoughPose and np.linalg.norm(uActual) < sphero.deadlockThreInput:
		matTmp = np.matrix([[math.cos(sphero.controlTurnAng),-math.sin(sphero.controlTurnAng)],[math.sin(sphero.controlTurnAng),math.cos(sphero.controlTurnAng)]])
		uActual = uActual+sphero.kDelta*np.dot(matTmp,uNominal)
		print "deadlock!!!" '''
	
	return uActual
	
	




