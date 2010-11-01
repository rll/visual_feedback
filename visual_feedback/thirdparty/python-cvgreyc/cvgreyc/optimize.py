from numpy.random import rand
import numpy as np

def spsa(x0,loss,niter,a0,A,c0,alpha=0.602,gamma=0.101,xmin = None,xmax = None, params = [] ):
	p = len(x0)
	x = x0.copy()
	if xmin is not None and xmax is not None :
		rang = (xmax-xmin)/2.
	else : rang = np.ones(p)/2.
	for k in xrange(niter):
		ak = a0/((k+1+A)**alpha)
		ck = c0/(k+1)**gamma
		delta = 2*np.round(rand(p))-1
		xplus  = x+ck*rang*delta
		xminus = x-ck*rang*delta
		print "xp : ",xplus
		print "xm : ",xminus
		yplus  = loss(xplus ,*params)
		yminus = loss(xminus,*params)
		
		ghat = (yplus-yminus)/(2*ck*delta)/rang
		x -= ak*ghat
		print "ck: ", ck
		print "yplus :", yplus, "yminus:", yminus
		print "gradient : ", ghat 
		print "new x ",x
		if xmin is not None :
			x = np.concatenate([x.reshape(1,p),xmin.reshape(1,p)],0).max(0)
		if xmax is not None :
			x = np.concatenate([x.reshape(1,p),xmax.reshape(1,p)],0).min(0)
	return x

