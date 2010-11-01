import numpy.ctypeslib as npC
import numpy as np
import ctypes as C
import os

_location = os.path.dirname(__file__)
_lib = npC.load_library("_interpolation.so",_location)

_lib.interpolate_double.restype = C.c_double
_lib.interpolate_double.argtypes = [ npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  C.c_double,C.c_double,C.c_int ]

_lib.interpolate_list_double.restype = None
_lib.interpolate_list_double.argtypes = [ npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  npC.ndpointer(ndim=2,dtype='float64',flags="c_contiguous"),
							  C.c_int,
							  npC.ndpointer(ndim=2,dtype='float64',flags="c_contiguous")]


_lib.interpolate_map_double.restype = None
_lib.interpolate_map_double.argtypes = [ npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous"),
							  npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous")]

_lib.interpolate_uchar.restype = C.c_double
_lib.interpolate_uchar.argtypes = [ npC.ndpointer(ndim=3,dtype='uint8',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  C.c_double,C.c_double,C.c_int ]

_lib.interpolate_list_uchar.restype = None
_lib.interpolate_list_uchar.argtypes = [ npC.ndpointer(ndim=3,dtype='uint8',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  npC.ndpointer(ndim=2,dtype='float64',flags="c_contiguous"),
							  C.c_int,
							  npC.ndpointer(ndim=2,dtype='float64',flags="c_contiguous")]


_lib.interpolate_map_uchar.restype = None
_lib.interpolate_map_uchar.argtypes = [ npC.ndpointer(ndim=3,dtype='uint8',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous"),
							  npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous")]

_lib.interpolate_int.restype = C.c_int
_lib.interpolate_int.argtypes = [ npC.ndpointer(ndim=3,dtype='int32',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  C.c_int,C.c_int,C.c_int ]

_lib.interpolate_list_int.restype = None
_lib.interpolate_list_int.argtypes = [ npC.ndpointer(ndim=3,dtype='int32',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  npC.ndpointer(ndim=2,dtype='float64',flags="c_contiguous"),
							  C.c_int,
							  npC.ndpointer(ndim=2,dtype='float64',flags="c_contiguous")]


_lib.interpolate_map_int.restype = None
_lib.interpolate_map_int.argtypes = [ npC.ndpointer(ndim=3,dtype='int32',flags="c_contiguous"),
                              C.c_int,C.c_int,C.c_int,
							  npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous"),
							  npC.ndpointer(ndim=3,dtype='float64',flags="c_contiguous")]

def _check_image(image):
	oldshape = image.shape
	kind = image.dtype.kind
	if image.ndim == 2 :
		image = image.reshape(image.shape[0],image.shape[1],1)
	if   kind == 'f' and image.dtype != 'float64' :
		image = image.astype('float64')
	elif kind == 'u' and image.dtype != 'uint8' :
		image = image.astype('uint8')
	elif kind == 'i' and image.dtype != 'int32' :
		image = image.astype('int32')
	return image,oldshape,kind

def interpolate(image,i,j,c=0):
	"""
		interpolate(image,i,j,c=0) -> float
		
		Computes value of image at position (i,j) and channel c
		by bilinear interpolation.
		
		arguments :
		 - image : original image. It can be either a gray or a 
		           multichannel image.
				   
	"""
	image,oldshape,kind = _check_image(image)
	si,sj,sc = image.shape
	if   kind == 'f' :
		res = _lib.interpolate_double(image,si,sj,sc,i,j,c)
	elif kind == 'u' :
		res = _lib.interpolate_uchar(image,si,sj,sc,i,j,c)
	elif kind == 'i' :
		res = _lib.interpolate_int(image,si,sj,sc,i,j,c)
	else : raise ValueError("Unexpected type for image")
	return res.reshape(oldshape)

def interpolate_list(image,coords):
	"""
		interpolate(image,coords) -> array
		
		Computes inteprolated values of image at positions given in
		'coords'	by bilinear interpolation.
		
		arguments :
		 - image : original image. It can be either a gray or a 
		           multichannel image.
		 - coords : list of coordinates at which the image value
		           is interpolated.
		
		res :
			array with interpolated image value for each point
			in 'coords'
				   
	"""
	coords = np.array(coords,'float')
	image,oldshape,kind = _check_image(image)
	si,sj,nchan = image.shape
	nsamples = len(coords)
	if coords.shape[1]!=2 : raise ValueError("Bad shape for coordinate list")
	values = np.zeros((nsamples,nchan),'float')
	
	if   kind == 'f' :
		_lib.interpolate_list_double(image,si,sj,nchan,coords,nsamples,values)
	elif kind == 'u' :
		_lib.interpolate_list_uchar(image,si,sj,nchan,coords,nsamples,values)
	elif kind == 'i' :
		_lib.interpolate_list_int(image,si,sj,nchan,coords,nsamples,values)
	else : raise ValueError("Unexpected type for image")

	return values
	
	

def interpolate_map(image,defmap):
	"""
		interpolate(image,defmap) -> new image
		
		Computes new image from 'image' and a dense deformation map
		
		arguments :
		 - image : original image. It can be either a gray or a 
		           multichannel image.
		 - defmap : deformation map, which gives, for each pixel the
		           translation vector that must be added to the pixel
				   position to gives the positions where the image
				   value must be evaluated.
		
		res :
			the new image
				   
	"""
	image,oldshape,kind = _check_image(image)
		
	si,sj,sc = image.shape
	mi,mj,mc = defmap.shape
	if si!=mi or sj!=mj or mc!=2 : raise ValueError("Incorrect size for deformation map")
	
	new_image = np.zeros(image.shape,'float64')
	if   kind == 'f' :
		_lib.interpolate_map(image,si,sj,sc,defmap,new_image)
	elif kind == 'u' :
		_lib.interpolate_map(image,si,sj,sc,defmap,new_image)
	elif kind == 'i' :
		_lib.interpolate_map(image,si,sj,sc,defmap,new_image)
	else : raise ValueError("Unexpected type for image")

	return new_image.reshape(oldshape)

class Interpolator(object):
	def __init__(self,image):
		self.image = image

	def interpolate(self,*args):
		if len(args) == 2 :
			i,j = args
			return interpolate_list(self.image,[(i,j)])[0]
		elif args[0].ndim == 2 :
			return interpolate_list(self.image,args[0])
		elif args[0].ndim == 3 :
			return interpolate_map(self.image,args[0])
		else : raise ArgumentError("Wrong number of arguments")

