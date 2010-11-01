/**********************************************************************
 *        Library to compute bilinear interpolation on images         *
 *                                                                    *
 *   author : Alexis Mignon (february 2010)                           *
 *   email  : alexis.mignon@info.unicaen.fr                           *
 **********************************************************************/

#ifndef _INTERPOLATION_H_
#define _INTERPOLATION_H_

#include <exception>
#include <string>
#include <cmath>

//#include <iostream>
//using namespace std;

#include "NDArray.h"

namespace CVGreyc {
namespace Interpolation {

double EPS = 1.0e-6 ;

class InterpolationError : public std::exception {
	private:
	std::string _message;
	public:
	InterpolationError() throw() {};
	InterpolationError(const std::string& message):_message(message){};
	virtual ~InterpolationError() throw() {};
	virtual const char* what() const throw() {return _message.c_str();}
};

class OutOfRange : public InterpolationError {
	public :
	OutOfRange() : InterpolationError("Index out of range"){};
	OutOfRange(const std::string& message):InterpolationError(message){};
	virtual ~OutOfRange() throw() {};
}; 

template<typename T>
double linear_interpolation(T value0, T value1, double t){
	return (1.0-t)*value0 + t*value1 ;
}

template<typename T>
double bilinear_interpolation(T value00, T value01, T value10, T value11, double u, double v){
	return  (1.0-u)*(1.0-v)*value00
	       +(1.0-u)*v*value01
		   +u*(1.0-v)*value10
		   +u*v*value11;
}

template<typename T>
class ImageInterpolator {
	private :
	const NDArray::ArrayBase<T>& _image;  // The image
	const NDArray::ArrayBase<T>* _pimage;  // Wrapper so the image is always a 3-dimansional image
	bool _own_pimage;                      // Do we own the pimage (i.e. did we create a wrapper)

	virtual void _compute_list(const NDArray::ArrayBase<double>& points, NDArray::ArrayBase<double>& result);
	virtual void _compute_map(const NDArray::ArrayBase<double>& points, NDArray::ArrayBase<double>& result);

	public:
	ImageInterpolator(const NDArray::ArrayBase<T>& image);
	virtual ~ImageInterpolator() {if (_own_pimage) delete _pimage; };
	
	virtual double operator () (double i, double j, size_t channel = 0) const ;
	virtual void operator() (const NDArray::ArrayBase<double>& points, NDArray::ArrayBase<double>& result);
};

template<typename T>
ImageInterpolator<T>::ImageInterpolator(const NDArray::ArrayBase<T>& image):_image(image),_pimage(0){
	if (image.ndim() == 2) {
		size_t shape[3];
		shape[0] = image.shape(0);	shape[1] = image.shape(1); shape[2] = 1 ;
		_pimage = image.reshape(3,shape);
		_own_pimage = true;
	}
	else {
		_pimage = &image;
		_own_pimage = false;
	}
};

template<typename T>
double ImageInterpolator<T>::operator () (double i, double j, size_t channel) const {

	if (i<0 or j<0) {
		//cerr<<"Bad indices : ("<<i<<","<<j<<")"<<endl;
		throw OutOfRange("double ImageInterpolator<T>::operator () (double i, double j, channel c) const : Index out of range error");
	}

	size_t i0 = 0 ;
	size_t j0 = 0 ;
	double u = 0.0 ;
	double v = 0.0 ;

	i0 = (size_t) floor(i);
	j0 = (size_t) floor(j);
	if      ( i-i0 < EPS ) u = 0.0 ;
	else if ( i0+1.0-i < EPS ) {
		u = 0.0 ;
		i0 += 1.0 ;
	}
	if      ( j-j0 < EPS ) v = 0.0 ;
	else if ( j0+1.0-j < EPS ) {
		v = 0.0 ;
		j0+=1;
	}
	if (i0 > _image.shape(0)-1  or j0 > _image.shape(1)-1 ) {
		//cerr<<"Bad indices : ("<<i<<","<<j<<")"<<endl;
		throw OutOfRange("double ImageInterpolator<T>::operator () (double i, double j, channel c) const : Index out of range error");
	}
	double f00,f01,f10,f11;
	double value ;
	
	if (u==0.0) {
		if 	(v==0.0) {
			value = (*_pimage)(i0,j0,channel);
		}		
		else {
			if (j0+1 > _image.shape(1)-1){
		//cerr<<"Bad indices : ("<<i<<","<<j<<")"<<endl;
		throw OutOfRange("double ImageInterpolator<T>::operator () (double i, double j, channel c) const : Index out of range error");
	}
			f00 = (double)(*_pimage)(i0,j0,channel)	;
			f01 = (double)(*_pimage)(i0,j0+1,channel) ;
			value = linear_interpolation(f00 , f01 , v);
		}
	}
	else {
		if 	(v==0.0) {
			if (i0+1 > _image.shape(0)-1) {
		//cerr<<"Bad indices : ("<<i<<","<<j<<")"<<endl;
		throw OutOfRange("double ImageInterpolator<T>::operator () (double i, double j, channel c) const : Index out of range error");
	}
			f00 = (double)(*_pimage)(i0,j0,channel)	;
			f10 = (double)(*_pimage)(i0+1,j0,channel) ;
			value = linear_interpolation(f00 , f10 , u);
		}		
		else {
			if (i0+1 > _image.shape(0)-1 or j0+1 > _image.shape(1)-1) {
		//cerr<<"Bad indices : ("<<i<<","<<j<<")"<<endl;
		throw OutOfRange("double ImageInterpolator<T>::operator () (double i, double j, channel c) const : Index out of range error");
	}
			f00 = (double)(*_pimage)(i0,j0,channel)	;
			f01 = (double)(*_pimage)(i0,j0+1,channel) ;
			f10 = (double)(*_pimage)(i0+1,j0,channel) ;
			f11 = (double)(*_pimage)(i0+1,j0+1,channel) ;
			value = bilinear_interpolation(f00 , f01 , f10 , f11 , u, v);
		}
	}
		
	return value;
}

/**
 * Interpolate image values given a set of sample points
 * 
 */

template<typename T>
void ImageInterpolator<T>::_compute_list(const NDArray::ArrayBase<double>& points, NDArray::ArrayBase<double>& result){
	
	size_t nsamples = points.shape(0);
	double i,j;
	size_t c;

	for (size_t s = 0; s<nsamples; s++){
		i = points(s,0);
		j = points(s,1);
		// loop over channels
		for (c = 0; c<result.shape(2); c++){
				try {
					result(i,j,c) = (*this)(i,j,c);
				}
				catch (OutOfRange){
					throw OutOfRange("void ImageInterpolator<T>::_compute_list(const NDArray::ArrayBase<double>&, NDArray::ArrayBase<double>&) const : Index out of range error");
				}
		}
	}
}

/**
 * Interpolate a new image from a displacement map
 * 
 */

template<typename T>
void ImageInterpolator<T>::_compute_map (const NDArray::ArrayBase<double>& points, NDArray::ArrayBase<double>& result){
	size_t i,j,c ;
	double ni,nj;
	for (i=0 ; i<result.shape(0) ; i++){
		for (j=0 ; j<result.shape(1) ; j++){
			ni = i + points(i,j,0);
			nj = j + points(i,j,1);
			for (c = 0 ; c<result.shape(2) ; c++) {
				try {
					result(i,j,c) = (*this)(ni,nj,c);
				}
				catch (OutOfRange){
					throw OutOfRange("void ImageInterpolator<T>::_compute_map(const NDArray::ArrayBase<double>&, NDArray::ArrayBase<double>&) const : Index out of range error");
				}
			}
		}
	}
}

template<typename T>
void ImageInterpolator<T>::operator()(const NDArray::ArrayBase<double>& points, NDArray::ArrayBase<double>& result){
	NDArray::ArrayBase<double>* presult;
	bool new_presult = false;
	
	if (points.ndim() == 2) { // Interpolation over some samples
		if (result.ndim() == 1){
			size_t shape[2] ; shape[0] = result.shape(0); shape[1] = 1 ;
			presult = result.reshape(2,shape);
			new_presult = true;
		}
		else presult = &result;
		_compute_list(points,*presult);
	}
	if (points.ndim() == 3) { // Interpolation with a displacement map
		if (result.ndim() == 2){
			size_t shape[3] ; shape[0] = result.shape(0); shape[1] = result.shape(1); shape[2] = 1 ;
			presult = result.reshape(3,shape);
			new_presult = true;
		}
		else presult = &result;
		_compute_map(points,*presult);
	}
	else {
		throw InterpolationError("void ImageInterpolator<T>::operator(const NDArray::ArrayBase<double>&, NDArray::ArrayBase<T>&) : Unexpected number of dimensions for 'points'.");
	}
	
	if (new_presult){
		 delete presult;
	}
}

}// end of namespace Interpolation
}// end of namespace CVGreyc

#endif
