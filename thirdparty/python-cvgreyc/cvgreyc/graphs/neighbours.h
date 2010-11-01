#include "Matrix.h"
#include <list>
#include <utility>
#include "ConnComp.h"
typedef std::pair< size_t , size_t > Edge;
typedef std::list< Edge > EdgeSet;

//DistMatrix<double>* compute_distances(const Matrix<double>& data);
Matrix<size_t>*  create_ranks(const Matrix<double>& distances);
EdgeSet* build_mst(const Matrix<double>& distances);
EdgeSet* build_incr_eps(const Matrix<double>& distances);
EdgeSet* build_incr_knn(const Matrix<double>& distances);
EdgeSet* build_incr_mknn(const Matrix<double>& distances);
EdgeSet* build_eps(const Matrix<double>& distances,double eps);
EdgeSet* build_knn(const Matrix<double>& distances,size_t k);
EdgeSet* build_mknn(const Matrix<double>& distances,size_t k);

