"""
   Conversion formulae from http://www.easyrgb.com
"""
import numpy as np
from scipy.linalg import inv

_xyz2rgb = np.array([
	[ 3.2406, -1.5372, -0.4986], 	
	[-0.9689,  1.8758,  0.0415],
	[ 0.0557, -0.2040,  1.0570]
])

_rgb2xyz = inv(_xyz2rgb)

def _apply_matrix(mat,arr):
	oldshape = a.shape
	newshape = (np.array(oldshape[-1]).prod(),3)	
	return np.dot(_xyz2rgb,a.reshape(newshape).T).T.reshape(oldshape)

def xyz2rgb(a):
	if a.shape[-1] != 3 : raise TypeError("Bad number of channels for a XYZ color image")
	oldshape = a.shape
	newshape = (np.array(oldshape[-1]).prod(),3)
	a = a/100.
	a = _apply_matrix(_xyz2rgb,a)
	cond = a>0.0031308
	return ((cond*(1.055*a**(1./2.4)-0.055) + (1-cond)*(12.92*a))*255.).astype('uint8')

def rgb2xyz(a):
	atype = a.dtype
	if a.shape[-1] != 3 : raise TypeError("Bad number of channels for a XYZ color image")
	oldshape = a.shape
	newshape = (np.array(oldshape[-1]).prod(),3)
	a = a/255.
	cond = a > 0.04045
	a = (cond*((a+0.055)/1.055)**2.4 + (1-cond)/12.92)*100.
	return _apply_matrix(_rgb2xyz,a)

def rgb2hsv(a):
	a = a/255.
	a_min = a.min(2)
	a_max = a.max(2)
	d_max = a_max - a_min
	
	sl = [slice(None)]*(a.ndim-1)
	R = H = sl+[0]
	G = S = sl+[1]
	B = V = sl+[2]
	
	na = np.zeros(a.shape)
	na[V] = a_max
	
	cond0 = d_max > 0.0
		
	
	
	na[S] =  (cond0)*d_max/a_max 
	
	d_cr = ((a_max - a)/6. + d_max/2.)/d_max
	
	cond0 = (a[R]==a_max)
	cond1 = (1-cond0)*(a[G]==a_max)
	cond2 = (1-cond1)*(a[B]==a_max)
	na[H] =    cond0*(d_cr[B]-d_cr[G]) \
	         + cond1*(1./3. + d_cr[R]-d_cr[B]) \
	         + cond2*(2./3. + d_cr[G]-d_cr[R])
	cond0 = na[H]>=0 and na[H]<=1
	cond1 = na[H]<0
	cond2 = na[H]>1
	na[H] = cond0*na[H]+cond1(na[H]+1)+cond2*(na[H]-1)
	return na

def hsv2rgb(a):
	sl = [slice(None)]*(a.ndim-1)
	R = H = sl+[0]
	G = S = sl+[1]
	B = V = sl+[2]
	
	h = a[H]*6
	
