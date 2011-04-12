import numpy.ctypeslib as npC
import ctypes as C
import numpy as np
import os 
import Image
from scipy.misc import imread


pattern_t = 'int%i'%(C.sizeof(C.c_long)*8)
size_t = 'uint%i'%(C.sizeof(C.c_size_t)*8)
double_t = 'float%i'%(C.sizeof(C.c_double)*8)
int_t = 'int%i'%(C.sizeof(C.c_int)*8)
uchar_t = 'uint8'

LBP_TYPES = ['raw','ri','riu2']

# Wrapping 'nvalues'

_path = os.path.dirname(__file__)
_lib = npC.load_library('_lbp.so',_path)

_lib.lbp_nvalues.restype = C.c_size_t
_lib.lbp_nvalues.argtypes = [
	C.c_int,
	C.c_char_p
]


def nvalues(P,lbp_type):
	"""
		return the number of different values expected for a given
		lbp operator of type 'lbp_type' with P samples.
	"""
	return _lib.lbp_nvalues(P,lbp_type)

# Wrapping 'compute_map'

_lib.lbp_map.restype = C.c_int
_lib.lbp_map.argtypes = [
	C.c_int,C.c_int,
	npC.ndpointer(ndim=2,dtype=double_t,flags='c_contiguous'),
	npC.ndpointer(ndim=2,dtype=pattern_t,flags='c_contiguous'),
	C.c_int,C.c_double,
	C.c_char_p
]

def compute_map(image,P,R,lbp_type):
	"""
		Computes the LBP map of the image.
	
	"""
	if lbp_type not in LBP_TYPES : raise ValueError("Unknown value for 'lbp_type' : '%s'"%str(lbp_type))
	lbp_map = np.zeros(image.shape,pattern_t)
	
	if image.dtype != double_t :
		image = image.astype(double_t)
	res = _lib.lbp_map(image.shape[0],image.shape[1],image,lbp_map,P,R,lbp_type.lower())
	if res != -1  : raise RuntimeError("UnexpectedError in compute_map : code %i"%res)
	return lbp_map

# Wrapping 'compute_hist'

_lib.lbp_histogram.restype = C.c_int
_lib.lbp_histogram.argtypes = [
	C.c_int,C.c_int,
	npC.ndpointer(ndim=2,dtype=double_t,flags='c_contiguous'),
	C.c_int,
	npC.ndpointer(ndim=1,dtype=double_t,flags='c_contiguous'),	
	C.c_int,C.c_double,
	C.c_char_p,
	C.c_double
]

def compute_hist(image, P, R, lbp_type, damp = 0, normalize=False, add_smooth = 0,bbox = None):
	"""
		Computes the LBP histogram of the image.
	
	"""
	
	if lbp_type not in LBP_TYPES : raise ValueError("Unknown value for 'lbp_type' : '%s'"%str(lbp_type))
	
	
	if isinstance(image,str):
		image = imread(image)
	elif isinstance(image,Image.Image):
		image = fromimage(image)
	
	if image.ndim == 3 : image = image.mean(2)
	
	if image.dtype != double_t or not image.flags["C_CONTIGUOUS"]:
		image = image.astype(double_t)

	nvals = nvalues(P,lbp_type.lower())
	hist = np.zeros(nvals,double_t)
	res = _lib.lbp_histogram(image.shape[0],image.shape[1],image,nvals,hist,P,R,lbp_type.lower(),damp)
	
	if res != -1  : raise RuntimeError("UnexpectedError in compute_map : code %i"%res)
	if normalize :
		if add_smooth > 0.0 : hist+=add_smooth
		hsum = hist.sum()
		if hsum != 0.0 : hist /= hist.sum()
	return hist

_lib.lbp_histogram_map.restype = C.c_int
_lib.lbp_histogram_map.argtypes = [
	C.c_int,C.c_int,
	npC.ndpointer(ndim=2,dtype=double_t,flags='c_contiguous'),
	C.c_int,
	npC.ndpointer(ndim=3,dtype=double_t,flags='c_contiguous'),
	C.c_int,C.c_double,
	C.c_char_p,
	C.c_int,C.c_double,
]

def compute_hist_map(image,P,R,lbp_type,size,damp = 0,normalize = False, add_smooth = 0):
	"""
		Computes the LBP histogram of the image.
	
	"""
	
	if lbp_type not in LBP_TYPES : raise ValueError("Unknown value for 'lbp_type' : '%s'"%str(lbp_type))

	if isinstance(image,str):
		image = imread(image)
	elif isinstance(image,Image.Image):
		image = fromimage(image)
	
	if image.ndim == 3 : image = image.mean(2)
	

	
	height,width = image.shape

	if image.dtype != double_t or not image.flags["C_CONTIGUOUS"] :
		image = image.astype(double_t)
	
	nvals = nvalues(P,lbp_type.lower())
	hist_map = np.zeros((height,width,nvals),double_t)
	res = _lib.lbp_histogram_map(height,width,image,nvals,hist_map,P,R,lbp_type.lower(),size,damp)

	if res != -1  : raise RuntimeError("UnexpectedError in compute_map : code %i"%res)
	if normalize :
		if add_smooth > 0.0 : hist_map+=add_smooth
		hist_map/=hist_map.sum(2).reshape(height,width,1)
	return hist_map

# Wrapping 'compute_histogram_grid'

_lib.lbp_histogram_grid.restype = C.c_int
_lib.lbp_histogram_grid.argtypes = [
	C.c_int,C.c_int,
	npC.ndpointer(ndim=2,dtype=double_t,flags='c_contiguous'),
	C.c_int,
	npC.ndpointer(ndim=3,dtype=double_t,flags='c_contiguous'),	
	C.c_int,C.c_double,
	C.c_char_p,C.c_int,C.c_int,
	C.c_double
]

def compute_hist_grid(image, P, R, lbp_type, grid, damp = 0, normalize=False, add_smooth = 0):
	"""
		Computes the LBP histogram of the image.
	
	"""
	
	if lbp_type not in LBP_TYPES : raise ValueError("Unknown value for 'lbp_type' : '%s'"%str(lbp_type))
	
	if isinstance(image,str):
		image = imread(image)
	elif isinstance(image,Image.Image):
		image = fromimage(image)
	
	if image.ndim == 3 : image = image.mean(2)
	
	if image.dtype != double_t or not image.flags["C_CONTIGUOUS"]:
		image = image.astype(double_t)

	grid_j,grid_i = grid
	nvals = nvalues(P,lbp_type.lower())
	hist = np.zeros((grid_i,grid_j,nvals),double_t)
	res = _lib.lbp_histogram_grid(image.shape[0],image.shape[1],image,nvals,hist,P,R,lbp_type.lower(),grid_i,grid_j,damp)
	
	if res != -1  : raise RuntimeError("UnexpectedError in compute_map : code %i"%res)
	if normalize :
		if add_smooth > 0.0 : hist_map+=add_smooth
		hist /= hist.sum()
	return hist

_lib.lbp_hist_grid_map.restype = C.c_int
_lib.lbp_hist_grid_map.argtypes = [
	C.c_int,C.c_int,
	npC.ndpointer(ndim=2,dtype=double_t,flags='c_contiguous'),
	C.c_int,
	npC.ndpointer(ndim=5,dtype=double_t,flags='c_contiguous'),
	C.c_int,C.c_double,
	C.c_char_p,C.c_int,C.c_int,
	C.c_int,C.c_double,
]

def compute_hist_grid_map(image,P,R,lbp_type,grid,size,damp = 0,normalize = False, add_smooth = 0):
	"""
		Computes the gridded LBP histogram map of the image.
	
	"""
	
	if lbp_type not in LBP_TYPES : raise ValueError("Unknown value for 'lbp_type' : '%s'"%str(lbp_type))
	
	if isinstance(image,str):
		image = imread(image)
	elif isinstance(image,Image.Image):
		image = fromimage(image)
	
	if image.ndim == 3 : image = image.mean(2)
	
	height,width = image.shape
	gridj,gridi = grid

	if image.dtype != double_t or not image.flags["C_CONTIGUOUS"] :
		image = image.astype(double_t)
	
	nvals = nvalues(P,lbp_type.lower())
	hist_map = np.zeros((height,width,gridi,gridj,nvals),double_t)
	res = _lib.lbp_hist_grid_map(height,width,image,nvals,hist_map,P,R,lbp_type.lower(),gridi,gridj,size,damp)

	if res != -1  : raise RuntimeError("UnexpectedError in compute_map : code %i"%res)
	hist_map = hist_map.reshape(height,width,gridi*gridj*nvals)
	if normalize :
		if add_smooth > 0.0 : hist_map+=add_smooth
		hist_map/=hist_map.sum(2).reshape(height,width,1)
	return hist_map


