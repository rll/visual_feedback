import numpy.ctypeslib as npC
import numpy as np
from scipy.sparse import dia_matrix
import ctypes as C
import os

"""
	Module with functions to compute minimum spanning trees,
	and k-nearest neighbourhood.

	Available functions are :
		- create_ranks(distances) : computes the ranks of each node in 
		         the neighbourhood of others.
				 
		- build_mst(distances) : compute the minimum spanning tree
		
		- build_eps(distances,eps) : computes the epsilon neighbourhood
		      for a distance 'eps' (i.e. nodes that are at a maximum 
			  distance of 'eps' of a given node).
			  
		- build_knn(distances,k) : computes the k-nearest neighbourhood.
		
		- build_mknn(distances,k) : computes the mutual k-nearest
		                            neighbourhood.

		- build_incr_eps(distances) : computes the minimum epsilon
		      neighbourhood that connects the graph.
		
		- build_incr_knn(distances) : computes the minimum k-nearest
		      neighbourhood that connects the graph.

		- build_incr_mknn(distances) : computes the minimum mutual 
		      k-nearest neighbourhood that connects the graph.
		


	rem : all the 'build_*' functions return as set of edges as a list 
	        of tuples. Each tuple contains the index of the nodes
			belonging to the edge.
"""

_location = os.path.dirname(__file__)
_lib = npC.load_library("_mst.so",_location)

_lib.create_ranks.restype = None
_lib.create_ranks.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t,
							  npC.ndpointer(ndim=2,dtype='uint64')]


_lib.build_mst.restype = C.py_object
_lib.build_mst.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t]

_lib.build_incr_eps.restype = C.py_object
_lib.build_incr_eps.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t]
							  
_lib.build_incr_knn.restype = C.py_object
_lib.build_incr_knn.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t]
							  
_lib.build_incr_mknn.restype = C.py_object
_lib.build_incr_mknn.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t]


_lib.build_eps.restype = C.py_object
_lib.build_eps.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t, C.c_double]


_lib.build_knn.restype = C.py_object
_lib.build_knn.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t, C.c_size_t]

_lib.build_mknn.restype = C.py_object
_lib.build_mknn.argtypes = [ npC.ndpointer(ndim=2,dtype='float64'),
                              C.c_size_t, C.c_size_t, C.c_size_t]


def _create_ranks(distances):
	"""
		_create_ranks(distances) -> ranks matrix
	
		Computes the matrix R of relative ranks which as
		the rank of j in the nearest neighbourhood of i as element
		R_ij.
		
		arguments :
			- distances : matrix of distances
		
		result :
			- matrix of ranks (see above for explanations).
		
		N.B.: this functions does exactly the same as
		      distances.argsort(1).argsort(1)
			  It is just there because we had to implement it in the 
			  c++ code.
	"""
	ranks = np.zeros(distances.shape,'uint64')
	row,col = distances.shape
	_lib.create_ranks(distances,row,col,ranks)
	return ranks

def create_ranks(distances):
	return distances.argsort(1).argsort(1)

def build_mst(distances):
	"""
		Computes the minimum spanning tree of nodes seperated by the
		distances given in the 'distances' matrix.
		
		arguments :
			- distances : matrix of distances, i.e. [distances]_ij = d(i,j)
		
		result :
			- liste of edges of the tree.
	
	"""
	row,col = distances.shape
	return _lib.build_mst(distances,row,col)

def build_eps(distances,eps):
	"""
		Computes the epsilon-neighbourhood graph of nodes seperated by the
		distances given in the 'distances' matrix.
		
		arguments :
			- distances : matrix of distances, i.e. [distances]_ij = d(i,j)
			- eps : epsilon threshold distance
		
		result :
			- liste of edges of the tree.
		
		N.B. : given a distances 'eps' the espilon-neighbourhood graph
		      connects each nodes to the other nodes that are at a 
			  maximum distance of 'eps'.
	
	"""
	row,col = distances.shape
	return _lib.build_mst(distances,row,col,eps)
	
def build_knn(distances,k):
	"""
		Computes the k-nearest neighbourhood graph of nodes seperated by
		the distances given in the 'distances' matrix.
		
		arguments :
			- distances : matrix of distances, i.e. [distances]_ij = d(i,j)
			- k : number of neighbours
		
		result :
			- liste of edges of the tree.
	
	"""
	row,col = distances.shape
	return _lib.build_mst(distances,row,col,eps)
	
def build_mknn(distances,k):
	"""
		Computes the mutual k-nearest neighbourhood graph of nodes 
		seperated by the distances given in the 'distances' matrix.
		
		arguments :
			- distances : matrix of distances, i.e. [distances]_ij = d(i,j)
			- k : number of neighbours
		
		result :
			- liste of edges of the tree.
	
		N.B. : a node i is in the mutual k-nearest neighbourhood of
		       another node j iff :
			   - i is in the k-nearest neighbourhood of j
			   - j is in the k-nearest neighbourhood of i
	
	"""
	row,col = distances.shape
	return _lib.build_mst(distances,row,col,eps)
	

	
def build_incr_eps(distances):
	"""
		Computes the epsilon-neighbourhood graph of nodes seperated by the
		distances given in the 'distances' matrix corresponding to the
		minimum value of epsilon allowing to connect the graph.
		
		arguments :
			- distances : matrix of distances, i.e. [distances]_ij = d(i,j)
		
		result :
			- liste of edges of the tree.
		
		N.B. : given a distances 'eps' the espilon-neighbourhood graph
		      connects each nodes to the other nodes that are at a 
			  maximum distance of 'eps'.
	
	"""
	row,col = distances.shape
	return _lib.build_incr_eps(distances,row,col)

def build_incr_knn(distances):
	"""
		Computes the k-nearest neighbourhood graph of nodes seperated by
		the distances given in the 'distances' matrix corresponding to the
		minimum value of k allowing to connect the graph.
		
		arguments :
			- distances : matrix of distances, i.e. [distances]_ij = d(i,j)
		
		result :
			- liste of edges of the tree.
	
	"""
	row,col = distances.shape
	return _lib.build_incr_knn(distances,row,col)


	
def build_incr_mknn(distances):
	"""
		Computes the mutual k-nearest neighbourhood graph of nodes 
		seperated by the distances given in the 'distances' matrix 
		corresponding to the minimum value of k allowing to connect the
		graph.
		
		arguments :
			- distances : matrix of distances, i.e. [distances]_ij = d(i,j)
		
		result :
			- liste of edges of the tree.
	
		N.B. : a node i is in the mutual k-nearest neighbourhood of
		       another node j iff :
			   - i is in the k-nearest neighbourhood of j
			   - j is in the k-nearest neighbourhood of i
	
	"""
	row,col = distances.shape
	return _lib.build_incr_mknn(distances,row,col)
	

def compute_distances2(data,W=None):
    """ Compute the euclidian distance matrix from data.
        
        usage : compute_distances2(data)
        
        argument:
            - data : a numpy array with one row per sample and one
                     column per feature.
                     
            - W : a Mahalanobis matrix

        return :
            - element-wise square of the euclidian distance
              matrix as a numpy array.
        
        Notes :
        
        This uses the fact that the euclidian distance d_ij is:
        (d_ij)^2 = (e_i-ej)^T*data*W*data^T*(e_i-e_j)
    """
    ndata,dim = data.shape
    mD = np.matrix(data)
    if W is None :
        DDt = mD*mD.T
    else :
        if type(W) == numpy.ndarray : W = matrix(W)
        DDt = mD*W*mD.T
    diag = dia_matrix((DDt.diagonal(),0),shape = DDt.shape)
    oneD = np.matrix(np.ones(DDt.shape))*diag
    DDt*=-2.0 ; DDt+=oneD; DDt+=oneD.T;
    return np.array(DDt-DDt.min())

def compute_distances(data,W=None):
    """ Compute the euclidian distance matrix from data.
        
        usage : compute_distances(data)
        
        argument:
            - data : a numpy array with one row per sample and one
                     column per feature.
            - W : a Mahalanobis matrix
        return :
            - euclidian distance matrix as a numpy array.
        
        Notes :
        
        Compute the elementwise square root matrix returned by
        'compute_distances2(data)'
        This uses the fact that the euclidian distance d_ij is:
        d_ij = sqrt( (e_i-ej)^T*data*data^T*(e_i-e_j) )
    """
    return np.sqrt( compute_distances2(np.array(data),W) )
