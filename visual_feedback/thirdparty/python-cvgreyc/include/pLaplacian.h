#ifndef _PLAPLACIAN_H_
#define _PLAPLACIAN_H_

#include "Graph.h"
#include "Matrix.h"

class Regularizer {
    private :
    const Graph& graph;
    const Matrix<double>* f0;
    Matrix<double>* ft;
    Matrix<double>* gnt;
    bool          * fixed;
    size_t iter;
	bool own_ft;
    
    
    
    void compute_gnorms();

    public :
    Regularizer(const Graph& _graph):graph(_graph),f0(0),ft(0),gnt(0),fixed(0),iter(0),own_ft(0){};
    ~Regularizer();
    double step(double p, double lambda);
    double compute(double p, double lambda, size_t itermax, double df_tol);
    void init(const Matrix<double>& _f0);
    void init(const Matrix<double>& _f0, Matrix<double>& _ft);
    void init(const Matrix<double>& _f0, bool* _fixed);
    void init(const Matrix<double>& _f0, Matrix<double>& _ft, bool* _fixed);
    Matrix<double>& values(){return *ft;}
    size_t getIter()const{return iter;}
};

#endif
