from scipy import array,sqrt,pi,arccos

def gradXY(image):
	"""
		gradXY(image) -> (gradx,grady)

		Computes the gradient of an image along X and Y axis.
		
		argument :
			- image : the image as a numpy array
		
		returns:
			- tuple containing :
				- gradx  : map of gradient along x axis
				- grady  : map of gradient along y axis

		example :
		>>> gradx,grady = gradXY(image)
	
	"""
	return image[:-1,:-1]-image[:-1,1:],image[:-1,:-1]-image[1:,:-1]

def grad_mag_ori(image):
	"""
		grad_mag_ori(image) -> (magnitude,orientation)
		Computes the magnitude and orientation of the image gradient.
	
		arguments :
			- image : the image as a numpy array
		
		returns:
			- tuple containing :
				- magnitude   : map of gradient magnitude
				- orientation : map of gradient orientation

		example :
		>>> mag,ori = grad_mag_ori(image)
	"""
	gx,gy = gradXY(image)
	mag = sqrt(gx**2+gy**2)
	gx_n = gx/(mag+1e-15)
	agx = arccos(gx_n)
	cond = gy>=0
	ori = agx*(2*cond-1)+2*pi*(1-cond)
	return mag,ori

