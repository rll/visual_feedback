"""
	Module to compute SURF features.
	This is just wrapper around the OpenSURF library by Chris Evans.
"""
import numpy.ctypeslib as npC
import ctypes as C
import numpy as np
import os
import Image

_path = os.path.dirname(__file__)
_lib = npC.load_library('_surf.so',_path)


#	void extract_surf_samples(const char* filename, int npoints, double* points, bool upright, double* descriptors);

_lib.extract_surf_samples.restype = None
_lib.extract_surf_samples.argtypes = [
	C.c_char_p,
	C.c_int,
	npC.ndpointer(ndim = 2, dtype = 'float64', flags = 'c_contiguous'),
	C.c_bool,
	npC.ndpointer(ndim = 2, dtype = 'float64', flags = 'c_contiguous')
]

def extract_surf_samples(filename, samples, upright = False):
	"""
		Extract surf descriptors for the points given in 'samples'.
		
		arguments :
		- filename : filename of the image to process
		- samples : array of samples given as a 2d array with
		            one samples per line and for each samples
					its coordinates (x and y), its orientation
					and its scale.
		-upright : if True we compute the upright version of SURF (faster)
		           otherwise the orientation independant version is
				   computed.
		returns :
			2D array with one descriptor (64 elements) per line

	"""
	if samples.ndim != 2 or samples.shape[1] != 4 : raise ValueError("Bad shape for 'samples' array")
	nsamples = samples.shape[0]
	descriptors = np.zeros((nsamples,64),'float64')
	_lib.extract_surf_samples(filename,nsamples,samples,upright,descriptors)
	return descriptors

#	void extract_surf_dense(const char* filename, double scale, int xmin, int xmax, int ymin, int ymax, bool upright, double* descriptors);
_lib.extract_surf_dense.restype = None
_lib.extract_surf_dense.argtypes = [
	C.c_char_p,
	C.c_double,
	C.c_int, C.c_int, C.c_int, C.c_int,
	C.c_bool,
	npC.ndpointer(ndim = 3, dtype = 'float64', flags = 'c_contiguous'),
]
def extract_surf_dense(filename, scale = 1.0, xmin = None,xmax = None,ymin = None,ymax = None, upright = False):
	"""
		Extract dense surf descriptors in an image.
		
		arguments :
		- filename : filename of the image to process
		- xmin,xmax,ymn,ymax : the bounding box in which we compute
		            the descriptors. If none is given, then the
					whole image is processed.

		-upright : if True we compute the upright version of SURF (faster)
		           otherwise the orientation independant version is
				   computed.
		returns :
			3D array with one descriptor (64 elements) per pixel.

	"""
	
	if xmin is None or xmin is None or ymin is None or ymax is None :
		img = Image.open(filename)
		width,height = img.size

	else : img = None
	if xmin is None : xmin = 0
	if xmax is None : xmax = width
	if ymin is None : ymin = 0
	if ymax is None : ymax = height
	if img is not None : del img
	
	width  = xmax-xmin+1
	height = ymax-ymin+1
	descriptors = np.zeros((width,height,64),'float64')
	_lib.extract_surf_dense(filename,scale,xmin,xmax,ymin,ymax,upright,descriptors)
	return descriptors

#	double* extract_surf(const char* filename, bool upright, int octaves, int intervals, int init_samples, double thres, int& nkeypoints);
_lib.extract_surf.restype = C.POINTER(C.c_double)
_lib.extract_surf.argtypes = [C.c_char_p,C.c_bool, C.c_int, C.c_int, C.c_int, C.c_double, C.POINTER(C.c_int)]

def extract_surf(filename, upright = False, octaves = 4, intervals = 4, init_sample = 2, thres = 0.0004):
	"""
		Performs keypoints detection and computes SURF descriptors one the image file with 
		path 'filename'.
	
		arguments :
		- upright : run in upright or rotation invariant mode ?
		- octaves : number of octaves to compute
		- intervals : number of intervals per octave
		- init_sample : initial sampling step
		- thres : blob threshold
		
		returns :
		- a tuple (keypoints,descriptors)
		- keypoints : 2D array with one keypoint per line and for 
		              each line : (x,y,orientation,scale)
		- descriptors :
		              2D array with one descriptor per line
	"""
	nkeypoints = C.c_int(0)
	
	result = _lib.extract_surf(filename,upright,octaves,intervals,init_sample,thres,C.pointer(nkeypoints))
	nkeypoints = nkeypoints.value
	keypoints = np.array(result[:4*nkeypoints])
	dkpts = 4*nkeypoints
	descr = 64*nkeypoints
	keypoints = np.array(result[:dkpts])
	descriptors = np.array(result[dkpts:dkpts+descr])
	_lib.free(result)
	return keypoints.reshape(nkeypoints,4),descriptors.reshape(nkeypoints,64)


