/**********************************************************************
 *               Local Binary Patterns (LBP)                          * 
 *   Library to compute LBP described in :                            *
 *                                                                    *
 * 	     'Multiresolution Gray-Scale and Rotation Invariant Texture   *
 *   Classification with Local Binary Patterns' by Timo Ojala,        *
 *   Matti Pietika inen and Topi Maenpaan in 'IEEE TRANSACTIONS ON    *
 *   PATTERN ANALYSIS AND MACHINE INTELLIGENCE'.                      *
 *                                                                    *
 *   author : Alexis Mignon (february 2010)                           *
 *   email  : alexis.mignon@info.unicaen.fr                           *
 **********************************************************************/

#ifndef _LBP_H_
#define _LBP_H_

#include "NDArray.h"

#include <exception>
#include <cmath>
#include <string>
#include <algorithm>

namespace CVGreyc {
namespace LBP {
using std::min;
using std::max;
/**********************************************************************
 *                         Exceptions                                 *
 * ********************************************************************/

class LBPError : public std::exception {
private:
	std::string _message;
public:
  LBPError() throw() {};
  LBPError(const std::string& message):_message(message){};
  virtual ~LBPError() throw() {};
  virtual const char* what() const throw() {return _message.c_str();}
};

/**********************************************************************
 *                    The Pattern object                              *
 * ********************************************************************/

/**
 * class Pattern
 * A pattern object.
 * A pattern has a size and a value.
 * We define basic operations list the left and right circular shift
 */

class Pattern {
	public:
	
	typedef long int pattern_t;
	
	private:
	size_t     _size;
	pattern_t  _mask;
	pattern_t  _last_mask;
	pattern_t  _value;

	public:
	
	Pattern():_size(1), _mask(0), _last_mask(1), _value(0) {}
	Pattern(size_t size, pattern_t value);
	Pattern(const Pattern& pattern);
	Pattern& operator = (const Pattern& pattern) ;
	
	size_t size() const { return _size; }
	pattern_t value() const  { return _value; }
	operator pattern_t() const { return _value; }
	bool operator == (const Pattern& pattern) { return _value == pattern._value; }
	bool operator == (int pattern) { return _value == pattern ; }
	Pattern lshift() const ; // left circular shift
	Pattern rshift() const ; // right circular shift
	Pattern min() const ;    // min value obtained by repeating circular shifts
	bool is_uniform() const; // is the pattern uniform ?
};

Pattern::Pattern(size_t size, pattern_t value):
	_size(size<1 ? 1 : size),_mask(~(~0<<_size)),
	_last_mask(1<<(_size-1)),
	_value(value <0  ? -1 : value&_mask) {}

Pattern::Pattern(const Pattern& pattern):
	_size(pattern._size),
	_mask(pattern._mask),
	_last_mask(pattern._last_mask),
	_value(pattern._value) {}
	
Pattern& Pattern::operator = (const Pattern& pattern) { 
	_size = pattern._size;
	_value = pattern._value;
	_mask = pattern._mask ;
	return *this;
}


inline Pattern Pattern::rshift() const {
	if (_value<0) return Pattern(_size,-1);
	pattern_t value = _value;
	bool l = _last_mask&value;
	value<<=1;
	value&=_mask;
	value+=(pattern_t) l;
	return Pattern(_size,value);
}

inline Pattern Pattern::lshift() const {
	if (_value<0) return Pattern(_size,-1);
	pattern_t value = _value;
	pattern_t l = 1&value;
	value>>=1;
	value+=(l<<(_size-1));
	return Pattern(_size,value);
}

inline Pattern Pattern::min() const {
	if (_value<0) return Pattern(_size,-1);
	Pattern p(*this);
	pattern_t value = p._value;
	
	for (size_t s = 0; s<_size-1; s++){
		p = p.lshift();
		if (p.value() < value)	value = p.value();
	}
	return Pattern(_size,value);
}

inline bool Pattern::is_uniform() const {
	if (_value<0) return false;
	pattern_t value = _value;
	size_t changes = (size_t) (_last_mask&value and 1&value) ;
	for (size_t s = 0; s<_size-1;s++){
		changes+=(size_t) ( (bool) (value&1) != (bool) (value&2)) ;
		if (changes > 2) return false;
		value>>=1;
	}
	return true ;
}

/**********************************************************************
 *                           Converters                               *
 * ********************************************************************/

/**
 *  Converter
 *  Abstract class defining the interface for a LBP converter
 *  used to convert a raw LBP pattern into a specific kind of
 *  pattern (e.g. LBP_ri, LBP_u2, LBP_riu2,...).
 * 
 *  Since they will usually be implemented as lookup tables,
 *  they implement the index operator.
 */ 

class Converter {

	public :
	Converter(){};
	Converter(size_t size){};
	virtual ~Converter(){};
	virtual Pattern operator []  (Pattern::pattern_t pattern) const =0;
	virtual Pattern operator []  (const Pattern& pattern) const =0;
	virtual size_t size() const = 0;    // Size of the patterns
	virtual size_t length() const {return (1<<this->size());};  // Maximum number of patterns
	virtual size_t nvalues() const = 0; // Number of different pattern values
};
/**
 *  DummyConverter
 *  Does nothing, just returns its input value;
 */
class DummyConverter : public Converter {
	private :
	size_t _size;
	public :
	DummyConverter(size_t size):_size(size){}	
	Pattern  operator [] (Pattern::pattern_t pattern) const {return Pattern(_size,pattern);}
	Pattern  operator [] (const Pattern& pattern) const  { return Pattern(pattern);}
	size_t size() const {return _size;}  // Size of the patterns
	size_t nvalues() const {return length();}   // Number of different pattern values
};
/**
 *  LookupTable
 *  Abstract base class for a Lookup table.
 *  Children must implement the '_compute_lut' protected method
 *  which will allocate the needed table and compute the mapped values.
 */ 
class LookupTable : public Converter {
	protected :
	size_t   _size;
	size_t   _nvalues;
	Pattern::pattern_t*  _lut;
	
	virtual void _compute_lut() = 0;
	
	public :
	LookupTable(size_t size) : _size(size), _lut(0){}
	virtual ~LookupTable(){ if (_lut) delete [] _lut; }
	Pattern operator [] (Pattern::pattern_t pattern) const {return pattern < 0 ? Pattern(_size,-1) : Pattern(_size,_lut[pattern]);}
	Pattern operator [] (const Pattern& pattern) const  { return pattern.value() < 0 ? Pattern(_size,-1) : Pattern(_size,_lut[Pattern(pattern)]);}
	size_t size() const { return _size;}     // Size of the patterns
	size_t nvalues() const {return _nvalues;}// Number of different pattern values
};

/**
 * RiLUT
 * Lookup table for the Rotation invariant LBP operator.
 */ 
class RiLUT : public LookupTable {
	protected:
	void _compute_lut();
	public:
	RiLUT(size_t size) : LookupTable(size) {_compute_lut();}
};

void RiLUT::_compute_lut() {
	_nvalues = 0;
	size_t _length = 1<<_size ;
	_lut = new Pattern::pattern_t[_length];
	Pattern::pattern_t values[_length];
	Pattern p;
	for (size_t i = 0;  i<_length ; i++) {
		p = Pattern(_size,i).min();
		if (p.value() == (Pattern::pattern_t)i){
			values[p]=_nvalues;
			_nvalues++;
		}
		_lut[i] = p.value() ;
	}
	
	for (size_t i = 0;  i<_length ; i++) {
		_lut[i] = values[_lut[i]];
	}
}

/**
 * Riu2LUT
 * Lookup table for the uniform rotation invariant LBP.
 */

class Riu2LUT : public LookupTable {
	protected:
	void _compute_lut();
	public:
	Riu2LUT(size_t size) : LookupTable(size) {_compute_lut();}
};

void Riu2LUT::_compute_lut() {
	_nvalues = _size + 2 ;
	_lut = new Pattern::pattern_t[length()];
	Pattern p;
	int pmin;

	_lut[0] = 0;
	for (size_t i = 1; i<=_size ; i++){
		pmin = (1<<i) - 1;
		_lut[pmin] = i;
	}

	for (size_t i = 2;  i< length() ; i++) {
		p = Pattern(_size,i);
		if (p.is_uniform()) {
			pmin = p.min();
			if ((size_t)pmin != i ) _lut[i] = _lut[pmin];
		}
		else _lut[i] = _size+1;
	}
}

/**********************************************************************
 *                     Utility functions                              *
 **********************************************************************/

/**
 *  int nearest_int(double value)
 *  computes the nearest interger value of a float.
 */

int nearest_int(double value){
	double res_value;
	if (value >= 0){
		res_value = floor(value);
		if (value - res_value > res_value + 1 - value ) res_value += 1. ;
	}
	else {
		res_value = ceil(value);	
		if (res_value - value > res_value - 1 - value ) res_value -= 1. ;
	}
	return (int) res_value;
}

/**********************************************************************
 *                     The LBP Operator                               * 
 **********************************************************************/ 

/**
 *  class Operator
 * 
 *  Generic LBP operator.
 *  Computes LBPs over an image. The specific type of LBP to be computed
 *  is specified through the template argument 'LBPConvert' which should
 *  be a class derived from 'Converter'.
 * 
 *  A LBP for the pixel (i,j) is computed at P sample points regularily
 *  spread on a circle of radius R centerd on (i,j). The image value at 
 *  each sample point I_sp is compared to the value of the central 
 *  pixel Ic and the corresponding value Vp of the sample p is set to 1 
 *  if I_sp >= Ic and 0 other wise.
 *  The corresponding LBP is the concatenation of the P sample values Vp.
 * 
 *  The image values used to compute LBPs might be interpolated or not.
 *  The corresponding behaviour is specified through the constructor
 *  argument 'interpolate'. If interpolate is true (default) then 
 *  bilinear interpolation is used. Otherwise, the value of the nearest
 *  pixel is used.
 */ 

template<class LBPConvert, typename T>
class Operator {
	private :
	size_t _P;
	double _R;
	NDArray::ArrayBase<double>* _samples;

	LBPConvert *_lut;
		
	public:
	Operator(size_t P, double R);
	~Operator();
	Pattern compute_raw_pattern(const NDArray::ArrayBase<T>& image, size_t i, size_t j) const ;
	Pattern compute_pattern(const NDArray::ArrayBase<T>& image, size_t i, size_t j) const {return (*_lut)[compute_raw_pattern(image,i,j)];} ;
	
	size_t size() const {return _lut->size();}
	size_t nvalues() const { return _lut->nvalues(); }
	void compute_map          (const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<Pattern::pattern_t>& lbp_map);
	void compute_hist         (const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<double>& hist, double damp = 0 );
	void compute_hist_map     (const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<double>& hist_map, int size, double damp = 0);
	void compute_hist_grid    (const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<double>& hist, double damp = 0);
	void compute_hist_grid_map(const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<double>& hist_map, int size, double damp = 0);
};

template<class LBPConvert, typename T>
Operator<LBPConvert,T>::Operator(size_t P, double R):
	_P(P),_R(R),_samples(new NDArray::CArray<double>(P,2)), _lut(new LBPConvert(P))
{
	
	for (size_t p = 0; p<_P; p++){
		(*_samples)(p,0) = -R * sin(2*M_PIl*(double)p /_P);
		(*_samples)(p,1) =  R * cos(2*M_PIl*(double)p /_P);
	}
}

template<class LBPConvert, typename T>
Operator<LBPConvert,T>::~Operator(){
	delete _samples;
	delete _lut;
}

template<class LBPConvert, typename T>
Pattern Operator<LBPConvert,T>::compute_raw_pattern(const NDArray::ArrayBase<T>& image, size_t i, size_t j) const {
	Pattern::pattern_t pattern = 0;
	double di,dj;
	int ni,nj;
	T center = image(i,j);
	T pval;
	for (size_t p = 0; p<_P ; p++){
		pattern<<=1;
		di = i+(*_samples)(p,0);
		dj = j+(*_samples)(p,1);
		ni = nearest_int(di);
		nj = nearest_int(dj);
		if (ni<0 or nj <0 or ni > (int)image.shape(0)-1 or nj > (int)image.shape(1) -1){
			pattern = -1;
			break;
		}
		else { 
			pval = image(ni,nj);
			if (pval>=center) pattern+=1;
		}
	}
	return Pattern(_P,pattern);
}


/**
 *  Computes the LBP map of the image.
 *  A LBP map is an array with the same same as the image and having for
 *  each pixel the value of the corresponding LBP.
 *  Pixels for which the LBP cannot be computed (pixels on the border)
 *  are set to -1.
 */

template<class LBPConvert, typename T>
void Operator<LBPConvert,T>::compute_map(const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<Pattern::pattern_t>& lbp_map){
	if (image.shape(0) != lbp_map.shape(0) or image.shape(1) != lbp_map.shape(1)) throw LBPError("Shapes of image and LBP map mismatch");

	for (size_t i = 0; i<image.shape(0); i++){
		for (size_t j = 0; j<image.shape(1); j++){
			lbp_map(i,j) = (*_lut)[compute_raw_pattern(image,i,j)];
		}	
	}		
}

/**
 *  Computes the LBP histogram for the given image.
 *  
 */

template<class LBPConvert, typename T>
void Operator<LBPConvert,T>::compute_hist(const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<double>& histogram, double damp){
	
	if (histogram.ndim() != 1 or histogram.shape(0) != _lut->nvalues() ) throw LBPError("Bad shape for histogram");
	Pattern::pattern_t value;
	double* gauss_i = 0;
	double* gauss_j = 0;
	
	size_t height = image.shape(0);
	size_t width  = image.shape(1);
	
	double ic = (height-1.)/2.;
	double jc = (width -1.)/2.;
	double diff;

	if (damp>0.0) {
		double sigma_i = damp*height;
		double sigma_j = damp*width;
		gauss_i = new double[height];
		gauss_j = new double[width];
		for (size_t i = 0; i<height; i++){
			diff = i-ic;
			gauss_i[i] = exp( - diff*diff / (2*sigma_i*sigma_i) );
		}
		for (size_t j = 0; j<width; j++){
			diff = j-jc;
			gauss_j[j] = exp( - diff*diff / (2*sigma_j*sigma_j) );
		}
	
		for (size_t i = 0; i<image.shape(0); i++){
			for (size_t j = 0; j<image.shape(1); j++){
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				if (value != -1) histogram(value)+=gauss_i[i]*gauss_j[j];
			}	
		}
		delete [] gauss_i;
		delete [] gauss_j;
	}
	else {
		for (size_t i = 0; i<image.shape(0); i++){
			for (size_t j = 0; j<image.shape(1); j++){
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				if (value != -1) histogram(value)+=1;
			}	
		}
	}
}

/**
 *  Computes the LBP histogram map with damping for the given image.
 *  
 */

template<class LBPConvert, typename T>
void Operator<LBPConvert,T>::compute_hist_map(const NDArray::ArrayBase<T>& image,
                                          NDArray::ArrayBase<double>& hist_map,
										  int size, double damp){
	if (hist_map.ndim() != 3 or
	    hist_map.shape(0) != image.shape(0) or
		hist_map.shape(1) != image.shape(1) or
		hist_map.shape(2) != _lut->nvalues() ) throw LBPError("Bad shape for histogram");
	Pattern::pattern_t value;
	
	double* gauss = 0;
	if (damp>0.0) {
		double sigma = damp*size;
		gauss = new double[2*size+1];
		for (int i = -size; i<=size; i++) gauss[i+size] = exp( - i*i / (2*sigma*sigma) );

		for (int i = 0; i<(int)image.shape(0); i++){
			for (int j = 0; j<(int)image.shape(1); j++){
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				if (value!=-1) {
					for (int ic = max(i-size,0) ; ic <= min(i+size,(int)image.shape(0)-1) ; ic++){
						for (int jc = max(j-size,0) ; jc <= min(j+size,(int)image.shape(1)-1) ; jc++){
							hist_map(ic,jc,value)+=gauss[i-ic+size]*gauss[j-jc+size];
						}
					}
				}
			}
		}
	}
	else {
		for (int i = 0; i<(int)image.shape(0); i++){
			for (int j = 0; j<(int)image.shape(1); j++){
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				if (value!=-1) {
					for (int ic = max(i-size,0) ; ic <= min(i+size,(int)image.shape(0)-1) ; ic++){
						for (int jc = max(j-size,0) ; jc <= min(j+size,(int)image.shape(1)-1) ; jc++){
							hist_map(ic,jc,value)+=1;
						}
					}
				}
			}
		}		
	}
	if (gauss) delete gauss;
}


/**
 *  Computes the LBP histogram within a grid for the given image.
 *  
 */

template<class LBPConvert, typename T>
void Operator<LBPConvert,T>::compute_hist_grid(const NDArray::ArrayBase<T>& image, NDArray::ArrayBase<double>& histogram, double damp){
	
	if (histogram.ndim() != 3 or  histogram.shape(2) != _lut->nvalues() ) throw LBPError("Bad shape for histogram");
	size_t grid_i = histogram.shape(0);
	size_t grid_j = histogram.shape(1);

	
	Pattern::pattern_t value;
	double* gauss_i = 0;
	double* gauss_j = 0;
	
	size_t height = image.shape(0);
	size_t width  = image.shape(1);

	size_t di = ceil(height*1.0/grid_i);
	size_t dj = ceil(width *1.0/grid_j);



	double ic = (height-1.)/2.;
	double jc = (width -1.)/2.;
	double diff;

	if (damp>0.0) {
		double sigma_i = damp*height;
		double sigma_j = damp*width;
		gauss_i = new double[height];
		gauss_j = new double[width];
		for (size_t i = 0; i<height; i++){
			diff = i-ic;
			gauss_i[i] = exp( - diff*diff / (2*sigma_i*sigma_i) );
		}
		for (size_t j = 0; j<width; j++){
			diff = j-jc;
			gauss_j[j] = exp( - diff*diff / (2*sigma_j*sigma_j) );
		}
	
		for (size_t i = 0; i<height; i++){
			size_t ci = i/di;
			for (size_t j = 0; j<width; j++){
				size_t cj = j/dj;			
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				if (value != -1) histogram(ci,cj,value)+=gauss_i[i]*gauss_j[j];
			}
		}
		delete [] gauss_i;
		delete [] gauss_j;
	}
	else {
		for (size_t i = 0; i<height; i++){
			size_t ci = i/grid_i;
			for (size_t j = 0; j<width; j++){
				size_t cj = j/grid_j;
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				if (value != -1) histogram(ci,cj,value)+=1;
			}	
		}
	}
}

/**
 *  Computes the LBP histogram map over a grid with damping for the given image.
 *  
 */

template<class LBPConvert, typename T>
void Operator<LBPConvert,T>::compute_hist_grid_map(const NDArray::ArrayBase<T>& image,
                                          NDArray::ArrayBase<double>& hist_map,
										  int size, double damp){
	if (hist_map.ndim() != 5 or
	    hist_map.shape(0) != image.shape(0) or
		hist_map.shape(1) != image.shape(1) or
		hist_map.shape(4) != _lut->nvalues() ) throw LBPError("Bad shape for hist_map");
	Pattern::pattern_t value;
	
	size_t grid_i = hist_map.shape(2);
	size_t grid_j = hist_map.shape(3);
	
	size_t height = image.shape(0);
	size_t width  = image.shape(1);

	size_t di = ceil((2.0*size+1.0)/grid_i);
	size_t dj = ceil((2.0*size+1.0)/grid_j);

	size_t indx[5];
	
	double* gauss = 0;
	if (damp>0.0) {
		double sigma = damp*size;
		gauss = new double[2*size+1];
		for (int i = -size; i<=size; i++) gauss[i+size] = exp( - i*i / (2*sigma*sigma) );

		for (int i = 0; i<(int)height; i++){
			for (int j = 0; j<(int)width; j++){
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				indx[4] = value;
				if (value!=-1) {
					for (int ic = max(i-size,0) ; ic <= min(i+size,(int)image.shape(0)-1) ; ic++){
						indx[0] = ic;
						size_t celli = (ic - i + size)/di;
						indx[2] = celli;
						for (int jc = max(j-size,0) ; jc <= min(j+size,(int)image.shape(1)-1) ; jc++){
							indx[1] = jc;
							size_t cellj = (jc - j + size)/dj;
							indx[3] = cellj;
							hist_map(indx)+=gauss[i-ic+size]*gauss[j-jc+size];
						}
					}
				}
			}
		}
	}
	else {
		for (int i = 0; i<(int)image.shape(0); i++){
			for (int j = 0; j<(int)image.shape(1); j++){
				value = (*_lut)[compute_raw_pattern(image,i,j)];
				indx[4] = value;
				if (value!=-1) {
					for (int ic = max(i-size,0) ; ic <= min(i+size,(int)image.shape(0)-1) ; ic++){
						indx[0] = ic;
						size_t celli = (ic - i + size)/di;
						indx[2] = celli;
						for (int jc = max(j-size,0) ; jc <= min(j+size,(int)image.shape(1)-1) ; jc++){
							indx[1] = jc;
							size_t cellj = (jc - j + size)/dj;
							indx[3] = cellj;
							hist_map(indx)+=1;
						}
					}
				}
			}
		}		
	}
	if (gauss) delete gauss;
}



} // end of namespace LBP 
} // end of namespace CVGreyc
#endif
