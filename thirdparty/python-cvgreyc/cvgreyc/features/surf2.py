"""
	This module uses SURF capabilities of opencv 2.x.
	This module was created, since the regular opencv 2.x python
	interface doesn't allow	the use of user-provided keypoints.
"""

import numpy.ctypeslib as npC
import numpy as np
import ctypes as C
import os

_path = os.path.dirname(__file__)
_lib = npC.load_library('_surf2.so',_path)



# void extract_surf_samples(int height, int width, unsigned char* image, int npoints, float* points, float* descriptors, 
#    						double hessian_threshold , int nOctaves , int octaveLayers,bool extended);

_lib.extract_surf_samples.restype = None
_lib.extract_surf_samples.argtypes = [
	C.c_int, C.c_int,
	npC.ndpointer(ndim=2,dtype = 'uint8',flags = 'c_contiguous'),
	C.c_int,
	npC.ndpointer(ndim=2,dtype = 'float32', flags = 'c_contiguous'),
	npC.ndpointer(ndim=2,dtype = 'float32', flags = 'c_contiguous'),
	C.c_double, C.c_int, C.c_int, C.c_bool
]

# void extract_surf_grid(int height, int width, unsigned char* image, double size,
#					    int xmin, int xmax, int ymin, int ymax,
#                       int nstepsx, int nstepsy, double* descriptors,
#                      double hessian_threshold , int nOctaves , int octaveLayers, bool extended)	

_lib.extract_surf_grid.restype = None
_lib.extract_surf_grid.argtypes = [
	C.c_int, C.c_int,
	npC.ndpointer(ndim=2,dtype = 'uint8',flags = 'c_contiguous'),
	C.c_double,
	C.c_int, C.c_int, C.c_int, C.c_int, C.c_int, C.c_int, 
	npC.ndpointer(ndim=3,dtype = 'float32', flags = 'c_contiguous'),
	C.c_double, C.c_int, C.c_int, C.c_bool
]

# double* extract_surf(int height, int width, unsigned char * image ,
#                      double thres, int nOctaves, int octaveLayers, bool extended,
#					   int& nkeypoints)

_lib.extract_surf.restype = C.POINTER(C.c_double)
_lib.extract_surf.argtypes = [
	C.c_int, C.c_int,
	npC.ndpointer(ndim=2,dtype = 'uint8',flags = 'c_contiguous'),
	C.c_double, C.c_int, C.c_int, C.c_bool,
	C.POINTER(C.c_int)
]

_lib.free_res.restype = None
_lib.free_res.argtypes = [
	C.POINTER(C.c_double)
]

def surf(image,keypoints = None, bbox = None, grid_size = None, size = None,
         hessian_threshold = 100., nOctaves = 3, octaveLayers = 4, extended = True):
	"""
		Compute Speed-up robust features descritpors.
		
		By default the function first detects keypoints and then computes
		the descriptors at found locations.
		
		If keypoints is not None, descriptors are computed for the
		given keypoints.
		
		If bbox and grid_size are not None keypoints are computed on
		a grid.
		
		args :
			- image : image on which SURF are computed
			     it can be either a numpy array, a PIL image or
				 a file name.
			- keypoints : array of keypoints. One keypoint per line and
			      each line contains 4 values : (x,y,size,angle).
				  If 'keypoints' is not None the detection of keypoints
				  is skipped and the descriptors are computed only 
				  for the given keypoints. (default None)

			- bbox, grid_size, size : if these arguments are not None
			     then keypoint detection is skipped and SURF descriptors
				 are computed for points sampled over a grid.
				 (default None for all)
				 
				 - bbox : tuple giving (xmin,ymin,xmax,ymax)
				 - grid_size : (nstepsx,nstepy) number of samples along
				     x and y axes.
			     - size : size parameter given for each sample point
			
			- hessian_threshold : threshold given to the detection 
			      algorithm to decide if a point is a keypoint
				  according to the value of hessian response.
				  (default 100)
			
			- nOctaves : number of octaves to compute (default 3)
			
			- octaveLayers : number of layers for each octave.
			                 (default 4)
			- extended : if true , then we compute the 128 components
			             SURF descriptor otherwise the 64 components
						 one. (default True)
		
		
	"""

	from scipy.misc import imread,fromimage
	
	if type(image) is str :
		image = imread(image)
	elif type(image) is not np.ndarray : # may be it's a PIL Image
		image = fromimage(image)
	
	if image.ndim > 2 :
		image = image.mean(2)
	
	if image.dtype != 'uint8' :
		image = image.astype('uint8')
	
	dsize = 128 if extended else 64
	
	height,width = image.shape
	
	if keypoints is not None :
		if bbox is not None or grid_size is not None :
			raise ValueError("bbox or grid_size parameters were not None but keypoint parameter was not None either")
		if type(keypoints) is not np.ndarray :
			keypoints = np.array(keypoints)
		if keypoints.dtype != 'float32':
			keypoints = keypoints.astype('float32')
		nkeypoints = len(keypoints)
		descr = np.zeros((nkeypoints,dsize),'float32')
		_lib.extract_surf_samples(height,width,image,nkeypoints,keypoints,
		                          descr,hessian_threshold,nOctaves,octaveLayers,
								  extended)
		return descr
	
	if bbox is not None :
		xmin,ymin,xmax,ymax = bbox
		if grid_size is None :
			grid_size = (xmax-xmin,ymax-ymin)
		if size is None : raise ValueError("No value given for 'size'")
		nstepx,nstepy = grid_size
		descr = np.zeros((nstepx,nstepy,dsize),'float32')
		_lib.extract_surf_grid(height,width,image,size,
		                       xmin,xmax,ymin,ymax,
							   nstepx,nstepy,
							   descr,hessian_threshold,nOctaves,octaveLayers,
							   extended)
		return descr
		
	nkeypoints = C.c_int(0)
	res_ = _lib.extract_surf(height,width,image,
                            hessian_threshold,
							nOctaves,octaveLayers, extended,
							C.pointer(nkeypoints))
	nkeypoints = nkeypoints.value
	res = np.array(res_[:nkeypoints*(dsize+4)]).reshape(nkeypoints,dsize+4)
	_lib.free_res(res_)
	return res[:,:4],res[:,4:]


