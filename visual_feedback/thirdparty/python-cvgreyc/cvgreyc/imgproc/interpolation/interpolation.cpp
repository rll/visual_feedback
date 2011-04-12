#include "Interpolation.h"
#include "NDArray.h"

typedef unsigned char uchar;

extern "C" {
	double linear_interpolation(double f0, double f1, double t);
	double bilinear_interpolation(double f00, double f01, double f10, double f11, double u, double v);

	double interpolate_double(int height, int width, int nchannels, double* image, double i, double j, int channel);
	void interpolate_list_double(int height, int width, int nchannels, double* image, int nsamples, double* coords, double* result);
	void interpolate_map_double(int height, int width, int nchannels, double* image,  double* map, double* result);
	
	double interpolate_uchar(int height, int width, int nchannels, uchar* image, double i, double j, int channel);
	void interpolate_list_uchar(int height, int width, int nchannels, uchar* image, int nsamples, double* coords, double* result);
	void interpolate_map_uchar(int height, int width, int nchannels, uchar* image,  double* map, double* result);

	double interpolate_int(int height, int width, int nchannels, int* image, double i, double j, int channel);
	void interpolate_list_int(int height, int width, int nchannels, int* image, int nsamples, double* coords, double* result);
	void interpolate_map_int(int height, int width, int nchannels, int* image,  double* map, double* result);
}


using namespace CVGreyc;
namespace I = Interpolation;

inline double linear_interpolation(double f0, double f1, double t){
	return I::linear_interpolation(f0,f1,t);
}

inline double bilinear_interpolation(double f00, double f01, double f10, double f11, double u, double v){
	return I::bilinear_interpolation(f00,f01,f10,f11,u,v);	
}

using namespace NDArray;

double interpolate_double(int height, int width, int nchannels, double* image, double i, double j, int channel){
	CArray<double> _image(image,height,width,nchannels);
	I::ImageInterpolator<double> interp(_image);
	return interp(i,j,channel);
}

void interpolate_list_double(int height, int width, int nchannels, double* image, int nsamples, double* coords, double* result){
	CArray<double> _image(image,height,width,nchannels);
	CArray<double> _result(result,nsamples,nchannels);
	CArray<double> _coords(coords,nsamples,2);
	
	I::ImageInterpolator<double> interp(_image);
	interp(_coords,_result);
}

void interpolate_map_double(int height, int width, int nchannels, double* image,  double* map, double* result){
	CArray<double> _image(image,height,width,nchannels);
	CArray<double> _result(result,height,width,nchannels);
	CArray<double> _map(map,height,width,2);
	
	I::ImageInterpolator<double> interp(_image);
	interp(_map,_result);
}

double interpolate_uchar(int height, int width, int nchannels, uchar* image, double i, double j, int channel){
	CArray<uchar> _image(image,height,width,nchannels);
	I::ImageInterpolator<uchar> interp(_image);
	return interp(i,j,channel);
}

void interpolate_list_uchar(int height, int width, int nchannels, uchar* image, int nsamples, double* coords, double* result){
	CArray<uchar> _image(image,height,width,nchannels);
	CArray<double> _result(result,nsamples,nchannels);
	CArray<double> _coords(coords,nsamples,2);
	
	I::ImageInterpolator<uchar> interp(_image);
	interp(_coords,_result);
}

void interpolate_map_uchar(int height, int width, int nchannels, uchar* image,  double* map, double* result){
	CArray<uchar> _image(image,height,width,nchannels);
	CArray<double> _result(result,height,width,nchannels);
	CArray<double> _map(map,height,width,2);
	
	I::ImageInterpolator<uchar> interp(_image);
	interp(_map,_result);
}

double interpolate_int(int height, int width, int nchannels, int* image, double i, double j, int channel){
	CArray<int> _image(image,height,width,nchannels);
	I::ImageInterpolator<int> interp(_image);
	return interp(i,j,channel);
}

void interpolate_list_int(int height, int width, int nchannels, int* image, int nsamples, double* coords, double* result){
	CArray<int> _image(image,height,width,nchannels);
	CArray<double> _result(result,nsamples,nchannels);
	CArray<double> _coords(coords,nsamples,2);
	
	I::ImageInterpolator<int> interp(_image);
	interp(_coords,_result);
}

void interpolate_map_int(int height, int width, int nchannels, int* image,  double* map, double* result){
	CArray<int> _image(image,height,width,nchannels);
	CArray<double> _result(result,height,width,nchannels);
	CArray<double> _map(map,height,width,2);
	
	I::ImageInterpolator<int> interp(_image);
	interp(_map,_result);
}
