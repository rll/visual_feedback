import numpy.ctypeslib as npC
import ctypes as C
import numpy as np
from scipy.misc import imread
import os
import Image

_path = os.path.dirname(__file__)
_lib = npC.load_library("_sift.so",_path)

# float* sift_run(
#	int height,
#	int width,
#	float* image,
#	int& nkeypoints,
#	float* keypoints,
#
#	int    first          ,
#	int    octaves        ,
#	int    levels         ,
#	float  threshold      ,
#	float  edgeThreshold  ,
#	float  magnif         ,
#	int    noorient       ,
#	int    stableorder    ,
#	int    savegss        ,
#	int    verbose        ,
#	int    binary         ,
#	int    haveKeypoints  ,
#	int    unnormalized )

_lib.run_sift.restype = C.POINTER(C.c_float)
_lib.run_sift.argtypes = [
	C.c_int, # height
	C.c_int, # width
	npC.ndpointer(ndim = 2, dtype = "float32", flags='c_contiguous'), # image
	C.POINTER(C.c_int), # nkeypoints
	npC.ndpointer(ndim = 2, dtype = "float32", flags='c_contiguous'), # keypoints
	C.c_int, C.c_int, C.c_int, # first, octaves, levels,
	C.c_float, C.c_float, C.c_float, # threshold, edgeThreshold, magnif
	C.c_int, C.c_int, # noorient, stableorder
	C.c_int, C.c_int, C.c_int, # savegss, verbose, binary
	C.c_int, C.c_int # haveKeypoints, unnormalized
]

initLevels = 3

def sift(image,keypoints = None,
			first          = -1 ,
			octaves        = -1 ,
			levels         = initLevels ,
			threshold      = -1 ,
			edgeThreshold  = 10.0,
			magnif         = 3.0 ,
			noorient       = 0 ,
			stableorder    = 0 ,
			savegss        = 0 ,
			verbose        = 0 ,
			binary         = 0 ,
			unnormalized   = 0 ):

	from scipy.misc import imread,fromimage		

	if threshold == -1 : threshold = 0.04 / levels / 2.
	if keypoints is None :
		keypoints = np.zeros((0,0),'float32')
		haveKeypoints = False
	else :
		haveKeypoints = True
		if type(keypoints) is not np.ndarray :
			keypoints = np.array(keypoints)
		if keypoints.dtype != 'float32' :
			keypoints = keypoints.astype('float32')
	
	nkeypoints = C.c_int(len(keypoints))
	
	if type(image) is str :
		image = imread(image)
	elif type(image) is not np.ndarray :
		image = fromimage(image)

	if type(image) is str :
		image = imread(image)
	if image.ndim > 2 :
		image = image.mean(2).astype('float32')
	if image.dtype != 'float32' :
		image = image.astype('float32')
	
	height,width = image.shape
	
	
	res_ = _lib.run_sift(height,width,image,
	           nkeypoints, keypoints,
			   first, octaves, levels,
			   threshold, edgeThreshold, magnif,
			   noorient, stableorder,
			   savegss, verbose,binary,
			   haveKeypoints,unnormalized)
	nkeypoints = nkeypoints.value
	res = np.array(res_[:132*nkeypoints]).reshape(nkeypoints,132)
	_lib.free(res_)
	return res[:,:4],res[:,4:]
			 
#~ if __name__ == '__main__' :
	#~ import Image
	#~ import sys 
	#~ im = Image.open(sys.argv[1])
	#~ k,d = run_sift(im,keypoints = [[10.,10.,21.,0.]])

	
