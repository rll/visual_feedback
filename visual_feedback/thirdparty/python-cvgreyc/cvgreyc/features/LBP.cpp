/**********************************************************************
 *                    Local Binary Patterns                           *
 *              Interface for 'ctypes'                                * 
 *                                                                    *
 *   author : Alexis Mignon (february 2010)                           *
 *   email  : alexis.mignon@info.unicaen.fr                           *
***********************************************************************/
#include "Python.h"
#include "LBP.h"
#include "NDArray.h"
#include <string>

using namespace CVGreyc;
using namespace LBP;
using namespace NDArray;
using namespace std;

typedef unsigned char uchar; 

extern "C" {
	size_t lbp_nvalues       (int P, const char* lbp_type);
	int    lbp_map           (int height, int width, double* image, Pattern::pattern_t* map,       int P, double R, const char* lbp_type);
	int    lbp_histogram     (int height, int width, double* image, int nvalues, double* hist,     int P, double R, const char* lbp_type, double damp);
	int    lbp_histogram_map (int height, int width, double* image, int nvalues, double* hist_map, int P, double R, const char* lbp_type, int size, double damp);
	int    lbp_histogram_grid(int height, int width, double* image, int nvalues, double* hist,     int P, double R, const char* lbp_type, int grid_i, int grid_j, double damp);
	int    lbp_hist_grid_map (int height, int width, double* image, int nvalues, double* hist,     int P, double R, const char* lbp_type, int grid_i, int grid_j,int size, double damp);
}
size_t lbp_nvalues(int P, const char* lbp_type){
	string type = lbp_type;
	
	if (lbp_type == string("raw"))        return DummyConverter(P).nvalues();
	else if (lbp_type == string("ri"))    return RiLUT(P).nvalues();
	else if (lbp_type == string("riu2"))  return Riu2LUT(P).nvalues();
	else return 0;
}

int lbp_map(int height, int width, double* image, Pattern::pattern_t* map, int P, double R, const char* lbp_type){
	typedef double PixType;
	CArray<PixType> _image(image,height,width);
	CArray<Pattern::pattern_t> _map(map,height,width);
	
	if (lbp_type == string("raw")) {
		Operator<DummyConverter,PixType>	op(P,R);
		op.compute_map(_image,_map);
	}
	else if (lbp_type == string("ri"))  {
		Operator<RiLUT,PixType>	op(P,R);
		op.compute_map(_image,_map);
	}
	else if (lbp_type == string("riu2")) {
		Operator<Riu2LUT,PixType>	op(P,R);
		op.compute_map(_image,_map);
	}
	else {
		return 1;
	}
	return -1;
}



int lbp_histogram(int height, int width, double* image,
               int nvalues, double* hist,
               int P, double R, const char* lbp_type,double damp){
	typedef double PixType;
	
	CArray<PixType> _image(image,height,width);
	CArray<double>  _hist(hist,nvalues);
	if (lbp_type == string("raw")) {
		Operator<DummyConverter,PixType> op(P,R);
		op.compute_hist(_image,_hist,damp);
	}
	else if (lbp_type == string("ri"))  {
		Operator<RiLUT,PixType>	op(P,R);
		op.compute_hist(_image,_hist,damp);
	}
	else if (lbp_type == string("riu2")) {
		Operator<Riu2LUT,PixType>	op(P,R);
		op.compute_hist(_image,_hist,damp);
	}
	else {
		return 1;
	}
	return -1;
}

int lbp_histogram_map(int height, int width, double* image,int nvalues,double* hist_map,
					int P, double R, const char* lbp_type, int size, double damp){
	typedef double PixType;
	
	CArray<PixType> _image(image,height,width);
	CArray<double>  _hist(hist_map,height,width,nvalues);
	if (lbp_type == string("raw")) {
		Operator<DummyConverter,PixType> op(P,R);
		op.compute_hist_map(_image,_hist,size,damp);
	}
	else if (lbp_type == string("ri"))  {
		Operator<RiLUT,PixType>	op(P,R);
		op.compute_hist_map(_image,_hist,size,damp);
	}
	else if (lbp_type == string("riu2")) {
		Operator<Riu2LUT,PixType>	op(P,R);
		op.compute_hist_map(_image,_hist,size,damp);
	}
	else {
		return 1;
	}
	return -1;
}
 
int lbp_histogram_grid(int height, int width, double* image,
               int nvalues, double* hist,
               int P, double R, const char* lbp_type, int grid_i, int grid_j,double damp){
	typedef double PixType;
	
	CArray<PixType> _image(image,height,width);
	CArray<double>  _hist(hist,grid_i,grid_j,nvalues);
	
	if (lbp_type == string("raw")) {
		Operator<DummyConverter,PixType> op(P,R);
		op.compute_hist_grid(_image,_hist,damp);
	}
	else if (lbp_type == string("ri"))  {
		Operator<RiLUT,PixType>	op(P,R); 
		op.compute_hist_grid(_image,_hist,damp);
	}
	else if (lbp_type == string("riu2")) {
		Operator<Riu2LUT,PixType>	op(P,R);
		op.compute_hist_grid(_image,_hist,damp);
	}
	else {
		return 1;
	}
	return -1;
}

int lbp_hist_grid_map(int height, int width, double* image,int nvalues,double* hist_map,
					  int P, double R, const char* lbp_type, int grid_i, int grid_j, int size, double damp){
	typedef double PixType;
	CArray<PixType> _image(image,height,width);
	size_t shape[5];
	shape[0] = height;
	shape[1] = width;
	shape[2] = grid_i;
	shape[3] = grid_j;
	shape[4] = nvalues;
	CArray<double>  _hist(hist_map,5,shape);
	if (lbp_type == string("raw")) {
		Operator<DummyConverter,PixType> op(P,R);
		op.compute_hist_grid_map(_image,_hist,size,damp);
	}
	else if (lbp_type == string("ri"))  {
		Operator<RiLUT,PixType>	op(P,R);
		op.compute_hist_grid_map(_image,_hist,size,damp);
	} 
	else if (lbp_type == string("riu2")) {
		Operator<Riu2LUT,PixType>	op(P,R);
		op.compute_hist_grid_map(_image,_hist,size,damp);
	}
	else {
		return 1;
	}
	return -1;
}
 
 
