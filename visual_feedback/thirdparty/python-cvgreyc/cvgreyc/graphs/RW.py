from Lplus import Lplus
import Graph
from scipy.sparse import dia_matrix

class Graph(Graph.Graph):
	def __init__(self,nnodes, edges = None):
		Graph.Graph.__init__(self,nnodes, edges)
		self._lplus = None
	
	@property
	def lplus(self):
		""" Pseudo invert of graph laplacian
		"""
		if self._lplus is None or self._laplacian is None :
			self._lplus = Lplus(self.laplacian)
	
	def CTEDs(self):
		""" Get matrix of commute time euclidian distances between nodes
		"""
		diag = dia_matrix((self.lplus.diagonal(),0),shape = lplus.shape)
		oneD = matrix(ones(lplus.shape))*diag
		return self.volume*self.ReffDists()
		
	def ReffDists(self):
		""" Get matrix of Resistence distances between nodes
		"""
		lplus = self.lplus
		diag = dia_matrix((lplus.diagonal(),0),shape = lplus.shape)
		oneD = matrix(ones(lplus.shape))*diag
		return array( oneD + oneD.T - 2*lplus )

	def ReffDist(self,i,j):
		lplus = self.lplus
		return lplus[i,i]+lplus[i,j]-2*lplus[i,j]

	def CTED(self,i,j):
		return self.volume*self.ReffDist(i,j)

	def hittime(self,i,j):
	   	lplus = self.lplus
		d = self.laplacian.diagonal()
		D = dia_matrix((self.degrees,0),shape = (self.nnodes,self.nnodes))
		sums = (lplus*D).sum(1)
		vol = self.volume
		return sums[i] - sums[j] - vol * lplus[i,j] + vol * lplus[j,j]
		
	def hittimes(self):
		lplus = self.lplus
		D = dia_matrix((self.degrees,0),shape = (self.nnodes,self.nnodes))
		diag = dia_matrix((lplus.diagonal(),0),shape = lplus.shape)
		sums = dia_matrix( ((lplus*D).sum(1),0),shape = lplus.shape)
		one = matrix(ones(lplus.shape))
		oneS = sums*one
		vol = self.volume
		return array(oneS - oneS.T - vol*lplus + vol*one*diag)

