from scipy import ones,identity,array,zeros
from cvxopt.cholmod import linsolve,splinsolve
from cvxopt.base import spmatrix, matrix,spdiag,sparse

def Lplus(L):
	try:
		nrow,ncol = L.shape
		Lcoo = L.tocoo()
		L = spmatrix(Lcoo.data.tolist(),Lcoo.row.tolist(),Lcoo.col.tolist())
	except AttributeError:
		nrow,ncol = L.size
	ones = matrix(-1.0/nrow,(nrow,ncol))
	ones+= spdiag(matrix(1.0,(1,nrow)))
	red = L[:-1,:-1]
	sol = ones[:-1,:]
	linsolve(red,sol,uplo='L')
	sol = array(sol)
	s = matrix(0.0,(1,ncol))
	s[0,:]=sol.sum(0)/nrow
	lplus = zeros((nrow,ncol))
	lplus[nrow-1,:] = -s[0,:]
	lplus[:-1,:] = sol-s
	return array(lplus)
