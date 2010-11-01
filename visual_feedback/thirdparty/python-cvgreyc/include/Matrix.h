#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <cstddef>

template <typename T>
class Matrix {
    public :
    Matrix(){};
    virtual ~Matrix(){};
    virtual size_t nrow() const = 0;
    virtual size_t ncol() const = 0;
    virtual size_t size() const = 0;

    virtual T     getElement(size_t i, size_t j) const = 0;
    virtual void  setElement(size_t i, size_t j, T value) = 0;
    
    virtual T  operator()(size_t i, size_t j) const = 0;
    virtual T& operator()(size_t i, size_t j)  = 0;
};

class Indexer {
    public :
    Indexer(){};
    virtual ~Indexer(){};
    virtual size_t operator()(size_t i, size_t j) const =0;
    virtual void setSize(size_t nrow, size_t ncol) = 0;
} ;

class CIndexer : Indexer {
    private :
    size_t _nrow;
    size_t _ncol;
    public :
    CIndexer():_nrow(0),_ncol(0){}
    virtual ~CIndexer(){};
    size_t operator()(size_t i, size_t j) const {return i*_ncol + j;};
    void setSize(size_t nrow, size_t ncol){_nrow = nrow; _ncol = ncol;}; 
};

class FIndexer : Indexer {
    private :
    size_t _nrow;
    size_t _ncol;
    public :
    FIndexer():_nrow(0),_ncol(0){}
    virtual ~FIndexer(){};
    size_t operator()(size_t i, size_t j) const {return i*_nrow + j;};
    void setSize(size_t nrow, size_t ncol){_nrow = nrow; _ncol = ncol;};     
};

template < typename T, class Index = CIndexer >
class MatrixCArray :public Matrix<T> {
private:
    size_t _nrow;
    size_t _ncol;
    Index  index;
    T *_data;
    bool _owner;

public:
    MatrixCArray(size_t nrow, size_t ncol, size_t size):
                _nrow(nrow),_ncol(ncol),
                _data(new T[size]),_owner(true){index.setSize(nrow,ncol);};
                
    MatrixCArray(size_t nrow, size_t ncol):
                _nrow(nrow),_ncol(ncol),
                _data(new T[nrow*ncol]),_owner(true){index.setSize(nrow,ncol);};
                
    MatrixCArray(size_t nrow, size_t ncol, size_t size, T* tab):
                _nrow(nrow),_ncol(ncol),
                _data(tab),_owner(false){index.setSize(nrow,ncol);};
    
    MatrixCArray(size_t nrow, size_t ncol, T* tab):
                _nrow(nrow),_ncol(ncol),
                _data(tab),_owner(false){index.setSize(nrow,ncol);};

    MatrixCArray(const Matrix<T>& matrix):
                _nrow(matrix.nrow()),_ncol(matrix.ncol()),
                _data(new T[matrix.nrow()*matrix.ncol()]),
                _owner(true){

        index.setSize(_nrow,_ncol);
        for (size_t i=0; i<_nrow; i++){
            for (size_t j=0; j<_ncol; j++){
                (*this)(i,j)=matrix(i,j);
            }
        }
    }
    virtual ~MatrixCArray(){if (_owner) delete [] _data;}

    size_t nrow() const { return _nrow; }
    size_t ncol() const { return _ncol; }
    virtual size_t size() const { return _nrow*_ncol; }

    T     getElement(size_t i, size_t j)   const   { return _data[index(i,j)];}
    void  setElement(size_t i, size_t j, T value) { _data[index(i,j)] = value ;}
    T     getElement(size_t i)          { return _data[i];}
    void  setElement(size_t i, T value) { _data[i] = value ;}
    
    T  operator()(size_t i, size_t j) const { return _data[index(i,j)];}
    T& operator()(size_t i, size_t j)       { return _data[index(i,j)];}

    T* operator[](size_t i) {return &_data[index(i,0)];}
};

template <typename T>
class MatrixFArray : public MatrixCArray<T,FIndexer> {
    public:
    MatrixFArray(size_t nrow, size_t ncol, size_t size):MatrixCArray<T,FIndexer>(nrow,ncol,size){}
    MatrixFArray(size_t nrow, size_t ncol):MatrixCArray<T,FIndexer>(nrow,ncol){}                
    MatrixFArray(size_t nrow, size_t ncol, size_t size, T* tab):MatrixCArray<T,FIndexer>(nrow,ncol,size,tab){}    
    MatrixFArray(size_t nrow, size_t ncol, T* tab):MatrixCArray<T,FIndexer>(nrow,ncol,tab){}
    MatrixFArray(const Matrix<T>& matrix):MatrixCArray<T,FIndexer>(matrix){};
    virtual ~MatrixFArray(){};
};

#endif

