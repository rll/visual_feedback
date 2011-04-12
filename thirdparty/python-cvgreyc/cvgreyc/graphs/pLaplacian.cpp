#include "pLaplacian.h"


extern "C" {
    double regularize(int nnodes,
                         int ndims,
                         double* fvalues_0, 
                         int nweights, 
                         int* rows, 
                         int* cols, 
                         double* weights, 
                         double* fvalues, 
                         double lambda, 
                         double p, 
                         int itermax, 
                         double dftol,
                         int* iter,
                         bool* fixed
                         );

double regularize2(int nnodes,
                         int ndims,
                         double* fvalues_0, 
                         int nweights, 
                         int* rows, 
                         int* cols, 
                         double* weights, 
                         double* fvalues, 
                         double p, 
                         double lambda, 
                         int itermax, 
                         double dftol,
                         int* iter
                         );
}

double regularize(int nnodes,
                         int ndims,
                         double* fvalues_0, 
                         int nweights, 
                         int* rows, 
                         int* cols, 
                         double* weights, 
                         double* fvalues, 
                         double p, 
                         double lambda, 
                         int itermax, 
                         double dftol,
                         int* iter,
                         bool* fixed
                         ){

    Graph graph(nnodes);
    for (int n = 0; n<nweights; n++) graph.addEdge(rows[n],cols[n],weights[n],false);
    Graph::neighbour_iterator ni;
    MatrixCArray<double> f0(ndims,nnodes,fvalues_0);
	MatrixCArray<double> ft(ndims,nnodes,fvalues);
	
    Regularizer reg(graph);
    
    if (fixed) reg.init(f0,ft,fixed);
    else       reg.init(f0,ft);

    double df = reg.compute(p,lambda,itermax,dftol);
    *iter = (int)reg.getIter();

    return df;
}

double regularize2(int nnodes,
                         int ndims,
                         double* fvalues_0, 
                         int nweights, 
                         int* rows, 
                         int* cols, 
                         double* weights, 
                         double* fvalues, 
                         double p, 
                         double lambda, 
                         int itermax, 
                         double dftol,
                         int* iter
                         ){
	return 	regularize(nnodes,ndims, fvalues_0, nweights, rows, cols, 
                       weights, fvalues, p, lambda, itermax, dftol, iter, 0);
}
