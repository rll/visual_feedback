#include "pLaplacian.h"
#include <cassert>
#include <cmath>

double EPS = 1.e-8;

Regularizer::~Regularizer(){
    if (ft and own_ft) delete ft;
    if (gnt) delete gnt;
}

void Regularizer::init(const Matrix<double>& _f0){
    assert(_f0.ncol()==graph.nnodes);
    f0 = &_f0;
    
    if (ft and own_ft) delete ft;
    if (gnt) delete gnt;
    
    ft =    new MatrixCArray<double>(*f0); own_ft=true;
    gnt =   new MatrixCArray<double>(_f0.nrow(), _f0.ncol());
    fixed = 0;
    iter = 0;
}
void Regularizer::init(const Matrix<double>& _f0, Matrix<double>& _ft){
    assert(_f0.ncol()==graph.nnodes);
    f0 = &_f0;
    
    if (ft and own_ft) delete ft;
    if (gnt) delete gnt;
    
    ft =    &_ft; own_ft=false;
    gnt =   new MatrixCArray<double>(_f0.nrow(), _f0.ncol());
    fixed = 0;
    iter = 0;
}

void Regularizer::init(const Matrix<double>& _f0, bool* _fixed){
    assert(_f0.ncol()==graph.nnodes);
    f0 = &_f0;
    
    if (ft and own_ft) delete ft;
    if (gnt) delete gnt;
    
    ft =    new MatrixCArray<double>(*f0); own_ft=true;
    gnt =   new MatrixCArray<double>(_f0.nrow(), _f0.ncol());
    fixed = _fixed;
    iter = 0;
}

void Regularizer::init(const Matrix<double>& _f0, Matrix<double>& _ft, bool* _fixed){
    assert(_f0.ncol()==graph.nnodes);
    f0 = &_f0;
    
    if (ft and own_ft) delete ft;
    if (gnt) delete gnt;
    
    ft =    &_ft; own_ft=false;
    gnt =   new MatrixCArray<double>(_f0.nrow(), _f0.ncol());
    fixed = _fixed;
    iter = 0;
}

void Regularizer::compute_gnorms(){
    if (!f0) return;
    double ngu,fu,fv,diff,wuv;
    size_t d,u,v;
    
    Matrix<double>& _ft = *ft;
    Matrix<double>& _gnt = *gnt;
    
    Graph::neighbour_iterator n;
 
    
    for (d=0;d<_ft.nrow();d++){
        for (u=0;u<_ft.ncol();u++){
            ngu = 0.0;
            fu = _ft(d,u);
            const Graph::neighbour_list& neighbs = graph.neighbours[u];
            for (n=neighbs.begin(); n!=neighbs.end(); n++){
                const Graph::neighbour& neighb = *n;
                v = neighb.first;
                wuv = neighb.second;
                fv = _ft(d,v);
                diff = fu-fv;
                ngu+=wuv*diff*diff;
            }
            _gnt(d,u) = sqrt(ngu);
        }
    }
}

double Regularizer::step(double p, double lambda){
    Matrix<double>& _gnt = *gnt;
    const Matrix<double>& _f0 = *f0;
    Matrix<double>& _ft = *ft;
    
    size_t d,u,v;
    double fv,wuv,gamma,gammaf, gamma_uv, gnu, gnv;
    
    Graph::neighbour_iterator n;
    
    double ftt_,fdiff,df2, df2max=0.0;
    double ftt[graph.nnodes];
    const Graph::neighbour_list* neighbs=0;
    compute_gnorms();

    for (d = 0; d<_f0.nrow(); d++){
        for (u = 0; u<graph.nnodes; u++){
			if (fixed and fixed[u]) continue;
            neighbs = &graph.neighbours[u];
            gamma  = 0.0;
            gammaf = 0.0;
            gnu = _gnt(d,u);
            if (gnu == 0.0) gnu = EPS;
            for (n=neighbs->begin();n!=neighbs->end();n++){
                v = (*n).first;
                wuv = (*n).second;
                fv = _ft(d,v);
                gnv = _gnt(d,v);
                if (gnv == 0.0) gnv = EPS;
                gamma_uv = wuv*(pow(gnu,p-2.)+pow(gnv,p-2.));
                gamma += gamma_uv;
                gammaf += gamma_uv*fv;
                
            }
            ftt[u] = (lambda*_f0(d,u)+gammaf)/(lambda+gamma);
        }
        df2=0.0;
        for (u = 0; u<graph.nnodes; u++){
			if (fixed and fixed[u]) continue;
            ftt_ = ftt[u];
            fdiff = _ft(d,u)-ftt_;
            df2+= fdiff*fdiff;
            _ft(d,u)=ftt_;
        }
        if (df2>df2max) df2max=df2;
    }
    return sqrt(df2max);
}

double Regularizer::compute(double p, double lambda, size_t itermax, double df_tol){
    double df=-1.0;
    
    for (iter=0;iter<itermax;iter++){
        df = step(p,lambda);
        if (df<=df_tol) break;
    }

    return df;
}


