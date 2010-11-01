import numpy.ctypeslib as npC
import ctypes as C
import numpy as np
import os

_path = os.path.dirname(__file__)
_lib = npC.load_library('_pLaplacian.so',_path)

_lib.regularize.restype = C.c_double
_lib.regularize.argtypes = [
	C.c_int,C.c_int,
	npC.ndpointer(ndim=2,dtype='float64',flags='c_contiguous'),
	C.c_int,
	npC.ndpointer(ndim=1,dtype='int32',flags='c_contiguous'),
	npC.ndpointer(ndim=1,dtype='int32',flags='c_contiguous'),
	npC.ndpointer(ndim=1,dtype='float64',flags='c_contiguous'),
	npC.ndpointer(ndim=2,dtype='float64',flags='c_contiguous'),
	C.c_double,
	C.c_double,
	C.c_int,
	C.c_double,
	C.POINTER(C.c_int),
	npC.ndpointer(ndim=1,dtype='bool',flags='c_contiguous'),
]

_lib.regularize2.restype = C.c_double
_lib.regularize2.argtypes = [
	C.c_int,C.c_int,
	npC.ndpointer(ndim=2,dtype='float64',flags='c_contiguous'),
	C.c_int,
	npC.ndpointer(ndim=1,dtype='int32',flags='c_contiguous'),
	npC.ndpointer(ndim=1,dtype='int32',flags='c_contiguous'),
	npC.ndpointer(ndim=1,dtype='float64',flags='c_contiguous'),
	npC.ndpointer(ndim=2,dtype='float64',flags='c_contiguous'),
	C.c_double,
	C.c_double,
	C.c_int,
	C.c_double,
	C.POINTER(C.c_int)
]

def regularize(graph,f0,p,lambd,itermax,fixed=None,dftol=0.0):
	if f0.ndim > 2 : raise ValueError("f0 can have only 1 or 2 dimensions")
	if f0.ndim == 1 :
		f0 = f0.reshape(1,f0.shape[0])
	if graph.nnodes != f0.shape[1] :
		raise ValueError("The number of nodes in the graph and the number of values in f0 mismatch")
	if f0.dtype != 'float64' :
		f0 = f0.astype('float64')

	ft = f0.copy()
	adj = graph.adj.tocoo()
	row = adj.row.astype('int32')
	col = adj.col.astype('int32')
	weights = adj.data.astype('float64')
	iter = C.c_int(0)
	if fixed is None : 
		df = _lib.regularize2(graph.nnodes,f0.shape[0],f0,
						len(row),row,col,weights,
						ft, p, lambd, itermax, dftol, C.pointer(iter))
	else :
		if fixed.dtype != 'bool' : fixed = fixed.astype('bool')
		df = _lib.regularize(graph.nnodes,f0.shape[0],f0,
						len(row),row,col,weights,
						ft, p, lambd, itermax, dftol, C.pointer(iter),fixed)
		
	print "Computed in %i iterations, final function variation: %f"%(iter.value,df)
	return ft

