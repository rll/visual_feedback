from Graph import Graph
from pLaplacian import regularize
import numpy as np

nnodes = 5
ndims = 5
p = 2.
lambd = 0.1

edges = [(i,i+1) for i in xrange(nnodes-1)]
print edges
gr = Graph(nnodes,edges)

f0 = np.zeros((ndims,nnodes))
f0[:,0]=np.array(range(5))+1.
f0[:,-1] = -(np.array(range(5))+1.)

print f0
ft = regularize(gr,f0,p,lambd,100)
print ft
print "with fixed values"
fixed = np.zeros(nnodes,'bool')
fixed[0]=fixed[-1]=True
print regularize(gr,f0,p,lambd,100,fixed=fixed)

