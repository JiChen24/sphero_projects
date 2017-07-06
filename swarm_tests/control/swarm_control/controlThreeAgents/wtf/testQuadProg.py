# test

import math
import numpy as np
import scipy as sp
import cvxopt


def testQP():
	Q = np.array([[6.0,1.0],[1.0,2.0]])
	P = np.array([[1.0],[1.0]])
	G = np.array([[2.0,-1.5],[0.0,-1.0]])
	h = np.array([[0.0],[0.0]])
	A = np.array([[0.0,0.0],[0.0,0.0]])
	b = np.array([0.0,0.0])

	sol = cvxopt.solvers.qp(cvxopt.matrix(Q),cvxopt.matrix(P),cvxopt.matrix(G),cvxopt.matrix(h))
	
	return sol

	
