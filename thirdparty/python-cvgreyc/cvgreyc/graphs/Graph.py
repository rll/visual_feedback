from scipy.sparse import lil_matrix,dia_matrix
from scipy import array,ones,zeros,dot,diag,matrix
import scipy

class Graph:
    """
        Class for coding graphs.
        It uses a sparse matrix to code edges.
    """
    def __init__(self,nnodes, edges = None):
        if type(nnodes) == str :
            self.read(nnodes)
        else :
            if type(nnodes) == int :
                self.nnodes = nnodes
                self.nodes = None
            else :
                self.nnodes = len(nnodes)
                self.nodes = array(nnodes)

            self.adj = lil_matrix((self.nnodes , self.nnodes))
            
            if edges is not None :
                if type(edges) == list :
                    for i,j in edges :
                        self.add_edge(i,j)
                elif type(edges) == dict :
                    for (i,j),w in edges.iteritems() :
                        self.add_edge(i,j,w)
            self.nlinks = 0
        
        self.neighbours = self.adj.rows
        self._laplacian = None
  
    def degree(self,i):
        return scipy.sum(self.adj[i,:].toarray())

    @property
    def degrees(self):
        return self.adj.sum(0)
  
    @property
    def laplacian(self):
        if self._laplacian is None :
            self._laplacian = dia_matrix((self.degrees,0),shape =(self.nnodes,self.nnodes))
            self._laplacian -= self.adj
        return self._laplacian

    @property
    def volume(self):
        return self.adj.sum()
    

    def identity(self):
        """ Returns the identy matrix with same dimension as the adjacency matrix.
        """
        return dia_matrix( (ones(self.nnodes),0),shape=(self.nnodes,self.nnodes))
    
    
    def add_edge(self,i,j,w=1.0):
        self.set_weight(i,j,w)
    
    def remove_edge(self,i,j):
        self.set_weight(i,j,0.0)
        
    def add_edges(self,edges):
        try :
            for i,j in edges :
                self.add_edge(i,j)
        except ValueError:
            for i,j,w in edges :
                self.add_edge(i,j,w)
            

    def set_weight(self,i,j,w):
        self._laplacian = None
        if i!=j :
            if w == 0.0 : self.nlinks -= 1
            elif not self.has_edge(i,j) : self.nlinks+=1
            self.adj[i,j] = w
            self.adj[j,i] = w
 
    def update_weight(self,i,j,dw):
        self._laplacian = None
        if i!=j :
            w = self.weight(i,j)
            if dw == 0.0 : return
            self.set_weight(i,j,w+dw)
  
    
    def has_edge(self,i,j):
        return self.weight(i,j) == 0.0
        
    def weight(self,i,j):
        return self.adj[i,j]

    @property
    def edges(self):
        A = self.adj.tocoo()
        edges = []
        for k in xrange(A.nnz):
            i = A.row[k]
            j = A.col[k]
            if i<j : edges.append((i,j))
        return edges

    def copy(self):
        gr = Graph(self.nnodes)
        gr.adj = self.adj.copy()
        gr.nodes = self.nodes[:,:]
        gr.nlinks = self.nlinks
        return gr
 
    
