#include <Python.h>
#include "neighbours.h"


extern "C" {
	void create_ranks(double* distances, int row, int col, size_t* ranks);
	PyObject* build_mst(double* distances, int row, int col);
	PyObject* build_incr_eps(double* distances, int row, int col);
	PyObject* build_incr_knn(double* distances, int row, int col);
	PyObject* build_incr_mknn(double* distances, int row, int col);
	PyObject* build_eps(double* distances, int row, int col, double eps);
	PyObject* build_knn(double* distances, int row, int col, size_t k);
	PyObject* build_mknn(double* distances, int row, int col, size_t k);
}

PyObject* edgeset2list(const EdgeSet* edges, bool del = false){
    
    PyObject* list = PyList_New(edges->size());
    int count = 0;
    for (EdgeSet::const_iterator e=edges->begin(); e!=edges->end(); e++){
        PyObject* epair = Py_BuildValue("(ii)",(*e).first,(*e).second);
        PyList_SET_ITEM(list,count,epair);
        count++;
    }
	if (del) delete edges;
    return list;
}

PyObject* clusterset2list(ConnComp::ClusterSet* clusterset){
    PyObject* clusterlist = PyList_New(clusterset->size());
    int count = 0;
    for (ConnComp::ClusterSet::const_iterator c = clusterset->begin(); c!=clusterset->end(); c++){
        PyObject* cluster = PyList_New((*c).size());
        int count2 = 0;
        for (ConnComp::Cluster::const_iterator item = (*c).begin(); item!=(*c).end(); item++){
            PyList_SET_ITEM(cluster,count2,Py_BuildValue("i",(*item)));
            count2++;
        }
        PyList_SET_ITEM(clusterlist,count,cluster);
        count++;
    }
    return clusterlist;
}

void create_ranks(double* distances, int row, int col, size_t* ranks){
	MatrixCArray<double> dist_mat(row,col,distances);
	Matrix<size_t>* ranks_mat =  create_ranks(dist_mat);
	size_t i,j;
	for (i = 0; i<(size_t)row; i++){
		for (j=0; j<(size_t)col; j++){
			ranks[i*col+j]=(*ranks_mat)(i,j);
		}
	}
	delete ranks_mat;
}


PyObject* build_mst(double* distances, int row, int col){
	return edgeset2list(build_mst(MatrixCArray<double>(row,col,distances)),true);
}

PyObject* build_incr_eps(double* distances, int row, int col){
	return edgeset2list(build_incr_eps(MatrixCArray<double>(row,col,distances)),true);
}

PyObject* build_incr_knn(double* distances, int row, int col){
	return edgeset2list(build_incr_knn(MatrixCArray<double>(row,col,distances)),true);
}

PyObject* build_incr_mknn(double* distances, int row, int col){
	return edgeset2list(build_incr_mknn(MatrixCArray<double>(row,col,distances)),true);
}

PyObject* build_eps(double* distances, int row, int col, double eps){
	return edgeset2list(build_eps(MatrixCArray<double>(row,col,distances),eps),true);
}

PyObject* build_knn(double* distances, int row, int col, size_t k){
	return edgeset2list(build_knn(MatrixCArray<double>(row,col,distances),k),true);
}

PyObject* build_mknn(double* distances, int row, int col, size_t k){
	return edgeset2list(build_mknn(MatrixCArray<double>(row,col,distances),k),true);
}
