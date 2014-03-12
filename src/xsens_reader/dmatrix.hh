/*The MIT License (MIT)

Copyright (c) 2014 HeXiang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/
#ifndef DMATRIX_HXX 
#define DMATRIX_HXX

#include <iostream>
#include <exception>
#include <limits>


class DNotInvertibleMatrixException: public std::exception {};
class DIncompatibleMatrixException: public std::exception {};
class DNotSquareMatrixException: public std::exception {};


template <class X> struct DVector{
public:
  DVector(int n=0);
  ~DVector();
  
  DVector(const DVector&);
  DVector& operator=(const DVector&);
  
  X&  operator[](int i) {
    if ((*shares)>1) detach();
    return elems[i];
  }
  
  const X&  operator[](int i) const { return elems[i]; }
  
  X operator*(const DVector&) const;
  DVector operator+(const DVector&) const;
  DVector operator-(const DVector&) const;
  DVector operator*(const X&) const;
  int dim() const { return size; }
  
  void detach();
  
  static DVector<X> I(int);
  
protected:
  X * elems;
  int size;
  int * shares;
};



template <class X> class DMatrix {
public:
  DMatrix(int n=0,int m=0);
  ~DMatrix();
  
  DMatrix(const DMatrix&);
  DMatrix& operator=(const DMatrix&);
  
  X * operator[](int i) {
    if ((*shares)>1) detach();
    return mrows[i];
  }
  
  const X * operator[](int i) const { return mrows[i]; }
  
  const X det() const;
  DMatrix<X> inv() const;
  DMatrix<X> transpose() const;
  DMatrix<X> operator*(const DMatrix<X>&) const;
  DMatrix<X> operator+(const DMatrix<X>&) const;
  DMatrix<X> operator-(const DMatrix<X>&) const;
  DMatrix<X> operator*(const X&) const;
  
  
  DMatrix<X>& operator+=(const DMatrix<X>&);
  DMatrix<X>& operator-=(const DMatrix<X>&);
  DMatrix<X>& operator*=(const X&);
  DMatrix<X>& operator/=(const X&);
  
  int rows() const { return nrows; }
  int columns() const { return ncols; }
  
  void detach();
  
  static DMatrix<X> I(int);
  static DMatrix<X> diag(const DVector<X>& v);
  
protected:
  X * elems;
  int nrows,ncols;
  X ** mrows;
  
  int * shares;
};


template <class X> DVector<X> operator * (const DMatrix<X> m, const DVector<X> v);

template <class X> DVector<X> operator * (const DVector<X> v, const DMatrix<X> m);




/*************** IMPLEMENTATION ***************/

#include "dmatrix.hxx"


#endif
