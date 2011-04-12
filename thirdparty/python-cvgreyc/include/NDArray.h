/**********************************************************************
 *              Library for N-dimensional arrays                      *
 *                                                                    *
 *  This library defines a generic interface for n-dimensional arrays,*
 *  called 'ArrayBase'.                                               *
 *                                                                    *
 *  We define a wrapper around a one-dimensional C-array which can be *
 *  used to wrap arrays given by 'ctypes' in Python.                  *
 *  The way the array is indexed can be tuned separately by defining  *
 *  an 'Indexer' object, which will be a template argument of the     *
 *  CArray object.                                                    *
 *  So far two indexers are given : one for C-style indexing and the  *
 *  other one for Fotran-style indexing.                              *
 *                                                                    *
 *   author : Alexis Mignon (february 2010)                           *
 *   email  : alexis.mignon@info.unicaen.fr                           *
 **********************************************************************/

#ifndef _NDARRAY_H_
#define _NDARRAY_H_

#include "Exceptions.h"
#include <cstddef>

#include <iostream>
using namespace std;

namespace CVGreyc {
namespace NDArray {

class NDArrayError : public Exceptions::Exception {
	public:
	NDArrayError(){};
	NDArrayError(const char* message):
		Exceptions::Exception(message){};
};

class ShapeMismatchError : public NDArrayError {
	public:
	ShapeMismatchError() : NDArrayError("Shapes mismatch !") {}
	ShapeMismatchError(const char* message) : NDArrayError(message) {}
};


/**
 * class IndexGenerator
 * Generate a canonical sequence of indices for arbitrary dimensions
 * and shapes.
 * example :
 * for an array with ndim = 3 and shape = 2,2,3 you will get the sequence :
 *          0,0,0
 *          0,0,1
 *          0,0,2
 *          0,1,0
 *          0,1,1
 *          0,1,2
 *          1,0,0
 *          ...
 *          1,1,2
 */
class IndexGenerator {
	private :
	size_t  _ndim;
	size_t *_shape;
	size_t *_index;
	bool    _first;
	
	public:
	IndexGenerator(size_t ndim, size_t* shape);
	void reset();
	size_t* operator () () ;
	
};

inline IndexGenerator::IndexGenerator(size_t ndim, size_t* shape):_ndim(ndim),_shape(shape),_index(new size_t[ndim]){
	for (size_t i = 0; i<ndim; i++) {
		_shape[i] = shape[i];
		_index[i] = 0;
	}
	_first = true;
}

inline void IndexGenerator::reset(){
	for (size_t i = 0; i<_ndim; i++) {
		_index[i] = 0;
	}
	_first = true;	
}

inline size_t* IndexGenerator::operator () () {
	if (_first) {
		_first = false;	
		return _index;
	}
	long int last = _ndim-1 ;
	while (true){
		if (_index[last]<_shape[last]-1) {
			_index[last]++;
			return _index;
		}
		else {
			_index[last] = 0;
			last--;
		}
		if (last<0) break;

	}
	return 0;
}

/**
 *  
 *  Base class for an n-dimensional array.
 *  This class is just an interface that a Array should implement
 * 
 */

template <typename T>
class ArrayBase {
	public:
	ArrayBase(){};
	virtual ~ArrayBase(){};
	
	virtual T  getElement(size_t* indices) const =0;
	virtual T& getElement(size_t* indices) =0;
	virtual void setElement(size_t* indices, T value) =0;
	
	virtual T operator () (size_t* indices) const = 0;
	virtual T& operator() (size_t* indices) =0;
	
	virtual T operator () (size_t i) const = 0 ;
	virtual T& operator () (size_t i) = 0 ;

	virtual T operator () (size_t i, size_t j) const = 0 ;
	virtual T& operator () (size_t i, size_t j) = 0 ;
	
	virtual T operator () (size_t i, size_t j, size_t k) const = 0 ;
	virtual T& operator () (size_t i, size_t j, size_t k) = 0 ;

	virtual T operator () (size_t i, size_t j, size_t k, size_t l) const  = 0 ;
	virtual T& operator () (size_t i, size_t j, size_t k, size_t l) = 0 ;

	virtual size_t  ndim() const =0;           // number of dimensions
	virtual size_t  shape(size_t i) const = 0; // returns the size along axis i
	virtual size_t *shape() const = 0;         // returns an array containing size for each axis
	virtual size_t  size() const = 0;          // total number of elements
	
	virtual void setValue(T value) = 0;
	
	virtual ArrayBase<T>* reshape(size_t ndim, size_t* shape) const = 0;
	
	// Basic arithmetic operation
	virtual ArrayBase<T>& operator= (const ArrayBase<T>& arr) ;
	virtual ArrayBase<T>& operator= (T value) ;
	virtual ArrayBase<T>& operator+= (const ArrayBase<T>& arr) ;
	virtual ArrayBase<T>& operator+= (T value) ;
	virtual ArrayBase<T>& operator-= (const ArrayBase<T>& arr) ;
	virtual ArrayBase<T>& operator-= (T value) ;
	virtual ArrayBase<T>& operator*= (const ArrayBase<T>& arr) ;
	virtual ArrayBase<T>& operator*= (T value) ;
	virtual ArrayBase<T>& operator/= (const ArrayBase<T>& arr) ;
	virtual ArrayBase<T>& operator/= (T value) ;
};

// Arithmetic operators...

template<typename T>
ArrayBase<T>& ArrayBase<T>::ArrayBase<T>::operator=(const ArrayBase<T>& arr) {
	if (arr.ndim() != this->ndim()) 
		throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator = (const ArrayBase<T>& arr) : Shape mismatch");
	for (size_t d = 0 ; d<this->ndim() ; d++) {
		if (arr.shape(d) != this->shape(d))
			throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator = (const ArrayBase<T>& arr) : Shape mismatch");
	}
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) = arr(gindex);
	return (*this);
}
template<typename T>
ArrayBase<T>& ArrayBase<T>::operator= (T value) {
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) = value;
	return (*this);	
}


template<typename T>
ArrayBase<T>& ArrayBase<T>::operator += (const ArrayBase<T>& arr) {
	if (arr.ndim() != this->ndim()) 
		throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator = (const ArrayBase<T>& arr) : Shape mismatch");
	for (size_t d = 0 ; d<this->ndim() ; d++) {
		if (arr.shape(d) != this->shape(d))
			throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator = (const ArrayBase<T>& arr) : Shape mismatch");
	}
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) += arr(gindex);
	return *this;
}

template<typename T>
ArrayBase<T>& ArrayBase<T>::operator += (T value) {
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) += value;
	return *this;
}

template<typename T>
ArrayBase<T>& ArrayBase<T>::operator -= (const ArrayBase<T>& arr) {
	if (arr.ndim() != this->ndim()) 
		throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator = (const ArrayBase<T>& arr) : Shape mismatch");
	for (size_t d = 0 ; d<this->ndim() ; d++) {
		if (arr.shape(d) != this->shape(d))
			throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator = (const ArrayBase<T>& arr) : Shape mismatch");
	}
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) -= arr(gindex);
	return *this;
}

template<typename T>
ArrayBase<T>& ArrayBase<T>::operator -= (T value) {
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) -= value;
	return *this;
}

template<typename T>
ArrayBase<T>& ArrayBase<T>::operator *= (const ArrayBase<T>& arr) {
	if (arr.ndim() != this->ndim()) 
		throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator *= (const ArrayBase<T>& arr) : Shape mismatch");
	for (size_t d = 0 ; d<this->ndim() ; d++) {
		if (arr.shape(d) != this->shape(d))
			throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator *= (const ArrayBase<T>& arr) : Shape mismatch");
	}
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) *= arr(gindex);
	return *this;
}

template<typename T>
ArrayBase<T>& ArrayBase<T>::operator *= (T value) {
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) *= value;
	return *this;
}

template<typename T>
ArrayBase<T>& ArrayBase<T>::operator /= (const ArrayBase<T>& arr) {
	if (arr.ndim() != this->ndim()) 
		throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator /= (const ArrayBase<T>& arr) : Shape mismatch");
	for (size_t d = 0 ; d<this->ndim() ; d++) {
		if (arr.shape(d) != this->shape(d))
			throw ShapeMismatchError("ArrayBase<T>& ArrayBase<T>::operator /= (const ArrayBase<T>& arr) : Shape mismatch");
	}
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) /= arr(gindex);
	return *this;
}

template<typename T>
ArrayBase<T>& ArrayBase<T>::operator /= (T value) {
	size_t *gindex;
	IndexGenerator gen(this->ndim(),this->shape());
	while ( (gindex = gen() )) (*this)(gindex) /= value;
	return *this;
}

/**
 *  
 *  Base class for an indexer.
 *  Indexer represent the way an array is indexed. Different indexers
 *  can represent C-style/F-style indexing or views of an array.
 *  For instance, some specific Indexers can be implemented for symmetric
 *  matrices.
 * 
 */

class Indexer{
	protected:
	
	size_t  _ndim;    // Number of dimensions
	size_t* _shape;   // Array of sizes along each axis
	
	public:
	Indexer();
	Indexer(size_t ndim, size_t* shape);
	// Constructors with implicit ndim for convenience
	explicit Indexer(size_t size_i);
	Indexer(size_t size_i, size_t size_j);
	Indexer(size_t size_i, size_t size_j, size_t size_k);
	Indexer(size_t size_i, size_t size_j, size_t size_k, size_t size_l);
	
	virtual ~Indexer();
	
	virtual size_t operator()(size_t* indices) const = 0;
	virtual size_t operator()(size_t i) const = 0;
	virtual size_t operator()(size_t i, size_t j) const = 0;
	virtual size_t operator()(size_t i, size_t j, size_t k) const = 0;
	virtual size_t operator()(size_t i, size_t j, size_t k, size_t l) const = 0;
	
	virtual Indexer& operator=(const Indexer& other);
	
	size_t  ndim() const {return _ndim;}
	size_t* shape() const {return _shape;}
	size_t  shape(size_t i) const {return i > (_ndim -1) ? 1 : _shape[i];}
};

inline Indexer::Indexer():_ndim(0),_shape(0){}

inline Indexer::Indexer(size_t nDim, size_t* shape):_ndim(nDim),_shape(new size_t[nDim]){
	for (size_t n=0;n<_ndim;n++){
		_shape[n]=shape[n];	
	}
}

inline Indexer::Indexer(size_t size_i):_ndim(),_shape(new size_t[1]){
	_shape[0]=size_i;
}


inline Indexer::Indexer(size_t size_i, size_t size_j):_ndim(2),_shape(new size_t[2]){
	_shape[0]=size_i;
	_shape[1]=size_j;
}

inline Indexer::Indexer(size_t size_i, size_t size_j, size_t size_k):_ndim(3),_shape(new size_t[3]){
	_shape[0]=size_i;
	_shape[1]=size_j;
	_shape[2]=size_k;
}

inline Indexer::Indexer(size_t size_i, size_t size_j, size_t size_k, size_t size_l):_ndim(4),_shape(new size_t[4]){
	_shape[0]=size_i;
	_shape[1]=size_j;
	_shape[2]=size_k;
	_shape[3]=size_l;
}

inline Indexer::~Indexer(){
	if (_shape) delete [] _shape;	
}


inline Indexer& Indexer::operator=(const Indexer& other) {
	if (_shape) delete [] _shape;
	_ndim = other._ndim;
	_shape = new size_t[_ndim];
	for (size_t n=0;n<_ndim;n++){
		_shape[n]=other._shape[n];
	}
	return *this;
}

/**
 *  C-Style indexer 
 */

class CIndexer : public Indexer {
	public:
	CIndexer(size_t ndim, size_t* shape):Indexer(ndim,shape){};
	explicit CIndexer(size_t size_i):Indexer(size_i){};
	CIndexer(size_t size_i, size_t size_j):Indexer(size_i, size_j){};
	CIndexer(size_t size_i, size_t size_j, size_t size_k):Indexer(size_i,size_j,size_k){};
	CIndexer(size_t size_i, size_t size_j, size_t size_k, size_t size_l):Indexer(size_i,size_j,size_k){};
	
	size_t operator()(size_t* indices) const ;
	size_t operator()(size_t i) const ;
	size_t operator()(size_t i, size_t j) const ;
	size_t operator()(size_t i, size_t j, size_t k) const ;
	size_t operator()(size_t i, size_t j, size_t k, size_t l) const ;
};

inline size_t CIndexer::operator ()(size_t* indices) const {
	size_t _index=indices[0];
	for (size_t n=1;n<_ndim;n++){
		_index*=_shape[n];
		_index+=indices[n];
	}
	return _index;
}

inline size_t CIndexer::operator()(size_t i) const {
	return i;
}

inline size_t CIndexer::operator()(size_t i, size_t j) const {
	return i*_shape[1]+j;
}

inline size_t CIndexer::operator()(size_t i, size_t j, size_t k) const {
	return (i*_shape[1]+j)*_shape[2] + k;
}

inline size_t CIndexer::operator()(size_t i, size_t j, size_t k, size_t l) const {
	return ((i*_shape[1]+j)*_shape[2] + k)*_shape[3]+l;
}

/**
 * XY indexer.
 * 
 * Many image libraries use (x,y) representation instead of (i,j).
 * 
 */

class XYIndexer : public Indexer {
	
	public:
	XYIndexer(size_t ndim, size_t* shape):Indexer(ndim,shape){};
	explicit XYIndexer(size_t size_i):Indexer(size_i){};
	XYIndexer(size_t size_i, size_t size_j):Indexer(size_i, size_j){};
	XYIndexer(size_t size_i, size_t size_j, size_t size_k):Indexer(size_i,size_j,size_k){};
	XYIndexer(size_t size_i, size_t size_j, size_t size_k, size_t size_l):Indexer(size_i,size_j,size_k){};
	
	size_t operator()(size_t* indices) const ;
	size_t operator()(size_t i) const ;
	size_t operator()(size_t i, size_t j) const ;
	size_t operator()(size_t i, size_t j, size_t k) const ;
	size_t operator()(size_t i, size_t j, size_t k, size_t l) const ;
};

inline size_t XYIndexer::operator()(size_t* indices) const {
	if (_ndim==1) return indices[0];
	size_t _index= indices[0]+indices[1]*_shape[0];
	for (size_t n=2;n<_ndim;n++){
		_index*=_shape[n];
		_index+=indices[n];
	}
	return _index;
}

inline size_t XYIndexer::operator()(size_t i) const {
	return i;
}

inline size_t XYIndexer::operator()(size_t i, size_t j) const {
	return j*_shape[0]+i;
}

inline size_t XYIndexer::operator()(size_t i, size_t j, size_t k) const {
	return (j*_shape[0]+i)*_shape[2] + k;
}

inline size_t XYIndexer::operator()(size_t i, size_t j, size_t k, size_t l) const {
	return ((j*_shape[0]+i)*_shape[2] + k)*_shape[3]+l;
}

/**
 * Fortran-style indexer.
 * 
 */

class FIndexer : public Indexer {
	
	public:
	FIndexer(size_t ndim, size_t* shape):Indexer(ndim,shape){};
	explicit FIndexer(size_t size_i):Indexer(size_i){};
	FIndexer(size_t size_i, size_t size_j):Indexer(size_i, size_j){};
	FIndexer(size_t size_i, size_t size_j, size_t size_k):Indexer(size_i,size_j,size_k){};
	FIndexer(size_t size_i, size_t size_j, size_t size_k, size_t size_l):Indexer(size_i,size_j,size_k){};
	
	size_t operator()(size_t* indices) const ;
	size_t operator()(size_t i) const ;
	size_t operator()(size_t i, size_t j) const ;
	size_t operator()(size_t i, size_t j, size_t k) const ;
	size_t operator()(size_t i, size_t j, size_t k, size_t l) const ;
};

inline size_t FIndexer::operator()(size_t* indices) const {
	size_t _index=indices[_ndim-1];
	for (size_t n=1;n<_ndim;n++){
		_index*=_shape[_ndim-1-n];
		_index+=indices[_ndim-1-n];
	}
	return _index;
}

inline size_t FIndexer::operator()(size_t i) const {
	return i;
}

inline size_t FIndexer::operator()(size_t i, size_t j) const {
	return j*_shape[0]+i;
}

inline size_t FIndexer::operator()(size_t i, size_t j, size_t k) const {
	return (k*_shape[1]+j)*_shape[0] + i;
}

inline size_t FIndexer::operator()(size_t i, size_t j, size_t k, size_t l) const {
	return ((l*_shape[2]+k)*_shape[1] + j)*_shape[2]+j;
}

/**
 *  Base class for an ArrayBase wrapping a one dimensional C-array.
 *  
 *  They way the elements of the array are indexed is left to child
 *  classes.
 */

template<typename T>
class CArrayBase : public ArrayBase<T> {
	
	protected:
	size_t   _ndim;     // Number of dimensions
	size_t*  _shape;    // Sizes along each axis
	T*       _data;     // Data array
	size_t   _size;     // Total number of elements
	bool     _owner;    // Does the class owns its data array
	
	
	public:
	// Constructors that create a new data array
	CArrayBase(size_t ndim, size_t* shape);                                 // Standard constructor for any dimensions
	explicit CArrayBase(size_t size_i);                                     // Constructor for one dimension
	CArrayBase(size_t size_i, size_t size_j);                               // Constructor for two dimensions
	CArrayBase(size_t size_i, size_t size_j, size_t size_k);                // Constructor for three dimensions
	CArrayBase(size_t size_i, size_t size_j, size_t size_k, size_t size_l); // Constructor for four dimensions
	
	// Constructors for existing data array
	CArrayBase(T* data, size_t ndim, size_t* shape);                                 // Standard constructor for any dimensions
	CArrayBase(T* data, size_t size_i);                                              // Constructor for one dimension
	CArrayBase(T* data, size_t size_i, size_t size_j);                               // Constructor for two dimensions
	CArrayBase(T* data, size_t size_i, size_t size_j, size_t size_k);                // Constructor for three dimensions
	CArrayBase(T* data, size_t size_i, size_t size_j, size_t size_k, size_t size_l); // Constructor for four dimensions

	virtual ~CArrayBase(){delete [] _shape; if (_owner and _data) delete [] _data;};

	virtual T  getElement(size_t* indices) const = 0;
	virtual T& getElement(size_t* indices) = 0;
	virtual void setElement(size_t* indices, T value) = 0;
	
	virtual T operator () (size_t* indices) const  = 0;
	virtual T& operator() (size_t* indices)  = 0;
	
	virtual T operator () (size_t i) const  = 0;
	virtual T& operator () (size_t i)  = 0;
	
	virtual T operator () (size_t i, size_t j) const  = 0;
	virtual T& operator () (size_t i, size_t j)  = 0;
	
	virtual T operator () (size_t i, size_t j, size_t k) const  = 0;
	virtual T& operator () (size_t i, size_t j, size_t k) = 0;
	
	virtual T operator () (size_t i, size_t j, size_t k, size_t l) const  = 0;
	virtual T& operator () (size_t i, size_t j, size_t k, size_t l)  = 0;
	
	virtual ArrayBase<T>& operator=(const ArrayBase<T>& arr);
	virtual ArrayBase<T>& operator=(T value);
	
	size_t  ndim() const          {return _ndim;}
	size_t* shape() const         {return _shape;}
	size_t  shape(size_t i) const {return i > (_ndim-1) ? 1 : _shape[i];}
	T*      data() const          {return _data;}
	size_t  size() const          {return _size;}
	bool    isOwner() const       {return _owner;}
	
	virtual void setValue(T value) { for (size_t i=0;i<_size;i++) _data[i] = value;}
	
	virtual ArrayBase<T>* reshape(size_t ndim, size_t* shape) const =0;

};

/**
 *  Generic constructor
 *  arguments :
 *  - ndim : number of dimensions
 *  - shape : array containing the sizes along each axis
 */
template<typename T>
CArrayBase<T>::CArrayBase(size_t ndim, size_t* shape):
		_ndim(ndim),_shape(new size_t[ndim]),_data(0),_size(0),
		_owner(true){

	if (ndim == 0) _size = 0;
	else {
		_size = 1.;	
		for (size_t n=0;n<ndim;n++){
			_shape[n] = shape[n];
			_size*=shape[n];	
		}
	}
	_data = new T[_size];

}

/**
 * Constructor for a one dimensional array
 * 
 * arguments :
 * - size_i : size of the array
 * 
 */
template<typename T>
CArrayBase<T>::CArrayBase(size_t size_i):
		_ndim(1),_shape(new size_t[1]),_data(0),_size(0),
		_owner(true){
			
	_size =size_i;
	_shape[0] = size_i;
	_data = new T[_size];
}

/**
 * Constructor for a two dimensional array
 * 
 * arguments :
 * - size_i : size along first axis
 * - size_j : size along second axis
 * 
 */
template<typename T>
CArrayBase<T>::CArrayBase(size_t size_i, size_t size_j):
		_ndim(2),_shape(new size_t[2]),_data(0),_size(0),
		_owner(true){
			
	_size =size_i*size_j;
	_shape[0] = size_i;
	_shape[1] = size_j;
	_data = new T[_size];
}

/**
 * Constructor for a three dimensional array
 * 
 * arguments :
 * - size_i : size along first axis
 * - size_j : size along second axis
 * - size_k : size along third axis
 * 
 */
template<typename T>
CArrayBase<T>::CArrayBase(size_t size_i, size_t size_j, size_t size_k):
		_ndim(3),_shape(new size_t[3]),_data(0),_size(0),
		_owner(true){
			
	_size =size_i*size_j*size_k;
	_shape[0] = size_i;
	_shape[1] = size_j;
	_shape[2] = size_k;
	_data = new T[_size];
}

/**
 * Constructor for a three dimensional array
 * 
 * arguments :
 * - size_i : size along first axis
 * - size_j : size along second axis
 * - size_k : size along third axis
 * - size_l : size along fourth axis
 * 
 */
template<typename T>
CArrayBase<T>::CArrayBase(size_t size_i, size_t size_j, size_t size_k, size_t size_l):
		_ndim(3),_shape(new size_t[3]),_data(0),_size(0),
		_owner(true){
			
	_size =size_i*size_j*size_k*size_l;
	_shape[0] = size_i;
	_shape[1] = size_j;
	_shape[2] = size_k;
	_shape[3] = size_l;
	_data = new T[_size];
}

/**
 *  Generic constructor to wrap an existing C array
 *  arguments :
 *  - ndim : number of dimensions
 *  - shape : array containing the sizes along each axis
 *  - data  : C-array (one dimension)
 */
template<typename T>
CArrayBase<T>::CArrayBase(T* data, size_t ndim, size_t* shape):
		_ndim(ndim),_shape(new size_t[ndim]),_data(data),_size(0),
		_owner(false){

	if (ndim == 0) _size = 0;
	else {
		_size = 1.;	
		for (size_t n=0;n<ndim;n++){
			_shape[n] = shape[n];
			_size*=shape[n];	
		}
	}
}

/**
 *  Constructor for a one dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size if the array
 *  - data   : C-array (one dimension)
 */
template<typename T>
CArrayBase<T>::CArrayBase(T* data, size_t size_i):
		_ndim(1),_shape(new size_t[1]),_data(data),_size(0),
		_owner(false){
			
	_size =size_i;
	_shape[0] = size_i;

}

/**
 *  Constructor for a two dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size along the first axis
 *  - size_j : Size along the second axis
 *  - data   : C-array (one dimension)
 */
template<typename T>
CArrayBase<T>::CArrayBase(T* data, size_t size_i, size_t size_j):
		_ndim(2),_shape(new size_t[2]),_data(data),_size(0),
		_owner(false){
			
	_size =size_i*size_j;
	_shape[0] = size_i;
	_shape[1] = size_j;
}

/**
 *  Constructor for a three dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size along the first axis
 *  - size_j : Size along the second axis
 *  - size_k : Size along the third axis
 *  - data   : C-array (one dimension)
 */
template<typename T>
CArrayBase<T>::CArrayBase(T* data, size_t size_i, size_t size_j, size_t size_k):
		_ndim(3),_shape(new size_t[3]),_data(data),_size(0),
		_owner(false){
			
	_size =size_i*size_j*size_k;
	_shape[0] = size_i;
	_shape[1] = size_j;
	_shape[2] = size_k;
}

/**
 *  Constructor for a four dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size along the first axis
 *  - size_j : Size along the second axis
 *  - size_k : Size along the third axis
 *  - size_l : Size along the fourth axis
 *  - data   : C-array (one dimension)
 */
template<typename T>
CArrayBase<T>::CArrayBase(T* data, size_t size_i, size_t size_j, size_t size_k, size_t size_l):
		_ndim(3),_shape(new size_t[3]),_data(data),_size(0),
		_owner(false){
			
	_size =size_i*size_j*size_k*size_l;
	_shape[0] = size_i;
	_shape[1] = size_j;
	_shape[2] = size_k;
	_shape[3] = size_l;

}

/**
 * Redefinitions of operator=
 */

template<typename T>
ArrayBase<T>& CArrayBase<T>::operator=(const ArrayBase<T>& arr) {
	return ArrayBase<T>::operator=(arr);
}

template<typename T>
ArrayBase<T>& CArrayBase<T>::operator= (T value) {
	for (size_t i = 0; i<_size; i++) _data[i] = value;
	return (*this);
}

/**
 *  Concrete implementation of a CArrayBase.
 *  It is merely a wrapper arround a one dimensional C-array.
 *  The way elements are accessed through indexing is defined by the
 *  'Index' object.
 *  Default indexing is C-Style but can be redifined to anything by 
 *  changing the template argument 'Index'.
 */

template<typename T, class Index=CIndexer>
class CArray : public CArrayBase<T> {
	private :
	Index   _index;     // Indexer
	
	public:
	// Constructors that create a new data array
	CArray(size_t ndim, size_t* shape);                                 // Standard constructor for any dimensions
	explicit CArray(size_t size_i);                                     // Constructor for one dimension
	CArray(size_t size_i, size_t size_j);                               // Constructor for two dimensions
	CArray(size_t size_i, size_t size_j, size_t size_k);                // Constructor for three dimensions
	CArray(size_t size_i, size_t size_j, size_t size_k, size_t size_l); // Constructor for four dimensions
	
	// Constructors for existing data array
	CArray(T* data, size_t ndim, size_t* shape);                                 // Standard constructor for any dimensions
	CArray(T* data, size_t size_i);                                              // Constructor for one dimension
	CArray(T* data, size_t size_i, size_t size_j);                               // Constructor for two dimensions
	CArray(T* data, size_t size_i, size_t size_j, size_t size_k);                // Constructor for three dimensions
	CArray(T* data, size_t size_i, size_t size_j, size_t size_k, size_t size_l); // Constructor for four dimensions

	virtual ~CArray(){};

	T  getElement(size_t* indices) const {return this->_data[_index(indices)];};
	T& getElement(size_t* indices) {return this->_data[_index(indices)];};
	void setElement(size_t* indices, T value) {this->_data[_index(indices)]=value; };
	
	T operator () (size_t* indices) const {return this->_data[_index(indices)];};
	T& operator() (size_t* indices) {return this->_data[_index(indices)];};
	
	T operator () (size_t i) const {return this->_data[_index(i)];}
	T& operator () (size_t i) {return this->_data[_index(i)];}
	
	T operator () (size_t i, size_t j) const {return this->_data[_index(i,j)];}
	T& operator () (size_t i, size_t j) {return this->_data[_index(i,j)];}
	
	T operator () (size_t i, size_t j, size_t k) const {return this->_data[_index(i,j,k)];}
	T& operator () (size_t i, size_t j, size_t k) {return this->_data[_index(i,j,k)];}
	
	T operator () (size_t i, size_t j, size_t k, size_t l) const {return this->_data[_index(i,j,k,l)];}
	T& operator () (size_t i, size_t j, size_t k, size_t l) {return this->_data[_index(i,j,k,l)];}
	
	virtual ArrayBase<T>& operator=(const ArrayBase<T>& arr);
	virtual ArrayBase<T>& operator=(T value);
		
	const Index&  index() const         {return _index;}
		
	
	virtual ArrayBase<T>* reshape(size_t ndim, size_t* shape) const;

};



/**
 *  Generic constructor
 *  arguments :
 *  - ndim : number of dimensions
 *  - shape : array containing the sizes along each axis
 */
template<typename T, class Index>
CArray<T,Index>::CArray(size_t ndim, size_t* shape):
		CArrayBase<T>(ndim,shape),
		_index(Index(ndim,shape)){}
/**
 * Constructor for a one dimensional array
 * 
 * arguments :
 * - size_i : size of the array
 * 
 */
template<typename T, class Index>
CArray<T,Index>::CArray(size_t size_i):
		CArrayBase<T>(size_i),
		_index(Index(size_i)){}

/**
 * Constructor for a two dimensional array
 * 
 * arguments :
 * - size_i : size along first axis
 * - size_j : size along second axis
 * 
 */
template<typename T, class Index>
CArray<T,Index>::CArray(size_t size_i, size_t size_j):
		CArrayBase<T>(size_i,size_j),
		_index(Index(size_i,size_j)){}

/**
 * Constructor for a three dimensional array
 * 
 * arguments :
 * - size_i : size along first axis
 * - size_j : size along second axis
 * - size_k : size along third axis
 * 
 */
template<typename T, class Index>
CArray<T,Index>::CArray(size_t size_i, size_t size_j, size_t size_k):
		CArrayBase<T>(size_i,size_j,size_k),
		_index(Index(size_i,size_j,size_k)){}

/**
 * Constructor for a three dimensional array
 * 
 * arguments :
 * - size_i : size along first axis
 * - size_j : size along second axis
 * - size_k : size along third axis
 * - size_l : size along fourth axis
 * 
 */
template<typename T, class Index>
CArray<T,Index>::CArray(size_t size_i, size_t size_j, size_t size_k, size_t size_l):
		CArrayBase<T>(size_i, size_j, size_k, size_l),
		_index(Index(size_i,size_j,size_k,size_l)){}

/**
 *  Generic constructor to wrap an existing C array
 *  arguments :
 *  - ndim : number of dimensions
 *  - shape : array containing the sizes along each axis
 *  - data  : C-array (one dimension)
 */
template<typename T, class Index>
CArray<T,Index>::CArray(T* data, size_t ndim, size_t* shape):
		CArrayBase<T>(data, ndim, shape),
		_index(Index(ndim,shape)){}

/**
 *  Constructor for a one dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size if the array
 *  - data   : C-array (one dimension)
 */
template<typename T, class Index>
CArray<T,Index>::CArray(T* data, size_t size_i):
		CArrayBase<T>(data, size_i),
		_index(Index(size_i)){}

/**
 *  Constructor for a two dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size along the first axis
 *  - size_j : Size along the second axis
 *  - data   : C-array (one dimension)
 */
template<typename T, class Index>
CArray<T,Index>::CArray(T* data, size_t size_i, size_t size_j):
		CArrayBase<T>(data, size_i, size_j),
		_index(Index(size_i,size_j)){}

/**
 *  Constructor for a three dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size along the first axis
 *  - size_j : Size along the second axis
 *  - size_k : Size along the third axis
 *  - data   : C-array (one dimension)
 */
template<typename T, class Index>
CArray<T,Index>::CArray(T* data, size_t size_i, size_t size_j, size_t size_k):
		CArrayBase<T>(data, size_i, size_j, size_k),
		_index(Index(size_i,size_j,size_k)){}

/**
 *  Constructor for a four dimensional array wrapping an existing C array
 *  arguments :
 *  - size_i : Size along the first axis
 *  - size_j : Size along the second axis
 *  - size_k : Size along the third axis
 *  - size_l : Size along the fourth axis
 *  - data   : C-array (one dimension)
 */
template<typename T, class Index>
CArray<T,Index>::CArray(T* data, size_t size_i, size_t size_j, size_t size_k, size_t size_l):
		CArrayBase<T>(data, size_i, size_j, size_k, size_l),
		_index(Index(size_i,size_j,size_k,size_l)){}

/**
 * Redefinitions of operator=
 */

template<typename T,class Index>
ArrayBase<T>& CArray<T,Index>::operator=(const ArrayBase<T>& arr) {
	return CArrayBase<T>::operator=(arr);
}

template<typename T,class Index>
ArrayBase<T>& CArray<T,Index>::operator= (T value) {
	return CArrayBase<T>::operator=(value);
}

template<typename T, class Index>
ArrayBase<T>* CArray<T,Index>::reshape(size_t ndim, size_t* shape) const {
	return new CArray<T,Index>(this->_data, this->_ndim, this->_shape);
}

} // End of namespace NDArray
} // End of namespace CVGreyc
#endif
