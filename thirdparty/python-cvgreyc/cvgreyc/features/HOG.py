########################################################################
# Histogram of oriented gradients                                      #
#                                                                      #
# Author : Alexis Mignon                                               #
# e-mail : alexis.mignon@gmail.com                                     #
# Date   : 12/04/2010                                                  #
########################################################################

"""
	Module to compute histograms of oriented gradients.
"""


from cvgreyc.imgproc import grad_mag_ori
import numpy as np
from scipy.misc import imread

_2pi = 2*np.pi

def check_angle(angle):
	# clip between -2*pi and +2*pi
	angle-= int(angle/_2pi)*_2pi
	# clip between 0. and 2*pi
	return angle if angle < _2pi and angle >= 0 else _2pi + angle
	
def hog(image,nori,cell = (8,8), block = (3,3), damp = True, sigma = 1.5 ):
	"""
		Computes the Histogram of oriented gradients (HOG) for the given
		image.
		
		arguments : 
		- image : image to process
		- nori  : number of orientation bins
		- cell  : 2-tuple size in pixel of a cell
		- block : 2-tuple size of a block in cell
		- damp  : do we damp magnitude values far from block center
		- sigma : sigma parameter of the gaussian used for damping
		
		returns :
			3D-array with for each "pixel" the HOG for the block.
		
	"""
	from scipy.signal import gaussian
	if type(image) is str :
		image = imread(image)
	if image.ndim == 3 : image = image.mean(2)
	mag,ori = grad_mag_ori(image)
	
	sy,sx = mag.shape
	by,bx = block
	cy,cx = cell
		
	bsx = bx*cx
	bsy = by*cy 
	
	nx = int(sx/bsx)
	ny = int(sy/bsy)
	
	dx = (sx - bsx*nx)/2
	dy = (sy - bsy*ny)/2
	
	start_x = dx
	end_x = dx+nx*bsx
	
	start_y = dy
	end_y = dy+ny*bsy

	mag = mag[start_y:end_y,start_x:end_x].copy()
	mbl = mag.reshape(ny,bsy,nx,bsx).swapaxes(1,2)
	if damp :
		mbl*= gaussian(bsy,sigma).reshape(1,1,bsy,1)
		mbl*= gaussian(bsx,sigma).reshape(1,1,1,bsx)
	
	ori = ori[start_y:end_y,start_x:end_x].copy()
	obl = ori.reshape(ny,bsy,nx,bsx).swapaxes(1,2)

	mcel = mbl.reshape(ny,nx,by,cy,bx,cx).swapaxes(3,4)
	ocel = obl.reshape(ny,nx,by,cy,bx,cx).swapaxes(3,4)

	orients = []
	for i in xrange(nori):
		if i == 0 :
			ori_sl = mcel*((ocel <= check_angle((i+0.5)*_2pi/nori ) ) | (ocel > check_angle((i-0.5)*_2pi/nori )))
		else :
			ori_sl = mcel*((ocel <= check_angle((i+0.5)*_2pi/nori ) ) & (ocel > check_angle((i-0.5)*_2pi/nori )))
		orients.append( ori_sl.sum(5).sum(4).reshape(ny,nx,by,bx,1))
	return np.concatenate(orients,4).reshape(nx,ny,by*bx*nori)
