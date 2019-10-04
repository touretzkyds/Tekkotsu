#include "least_squares.h"
#include <limits>

/*
Least Squares Solver
by Ethan Tira-Thompson, © 2011

least_squares is free software: you can redistribute it and/or modify
it under the terms of the Lesser GNU General Public License as published
by the Free Software Foundation, either version 2 of the License, or
(at your option) any later version, waiving any requirement to
distribute the license document itself.

least_squares is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
Lesser GNU General Public License for more details.
*/

namespace least_squares {

// can't typedef a template, so use macro :-P
#define V(T) std::valarray< T >
#define M(T) std::valarray< V(T) >

template<class T>
void invert(M(T)& m) {
	const size_t DIM=m.size();
#ifdef DEBUG
	for(size_t i=0; i<DIM; ++i)
		if(m[i].size()!=DIM)
			throw std::invalid_argument("invert() of non-square matrix");
#endif
	
	M(T) x(V(T)(DIM*2),DIM);
	for(size_t c=0; c<DIM; ++c) // copy m in top half
		x[c][std::slice(0,DIM,1)] = m[c];
	for(unsigned int c=0; c<DIM; ++c) // identity (to be inverse) in bottom half
		x[c][DIM+c] = 1;
	
	for(unsigned int c=0; c<DIM; ++c) {
		// find pivot
		T mx=std::abs(x[c][c]);
		size_t mxi=c;
		for(unsigned int r=c+1; r<DIM; ++r) {
			const T v=std::abs(x[r][c]);
			if(v>mx) {
				mx=v;
				mxi=r;
			}
		}
		if(mx<std::numeric_limits<float>::epsilon())
			throw std::underflow_error("low rank matrix, non-invertable");
		
		// swap to put pivot in position
		std::swap(x[c],x[mxi]);
		
		{
			const T p=x[c][c];
			for(size_t c2=c+1; c2<DIM*2; ++c2)
				x[c][c2] /= p;
			x[c][c] = 1;
		}
		
		for(size_t r=c+1; r<DIM; ++r) {
			const T p = x[r][c];
			for(size_t c2=c+1; c2<DIM*2; ++c2)
				x[r][c2] -= x[c][c2]*p;
			x[r][c] = 0;
		}
	}
	for(size_t c=0; c<DIM; ++c) {
		for(size_t r=0; r<c; ++r) {
			const T p = x[r][c];
			for(size_t c2=c+1; c2<DIM*2; ++c2)
				x[r][c2] -= x[c][c2]*p;
			x[r][c] = 0;
		}
	}
	
	for(size_t c=0; c<DIM; ++c)
		m[c] = x[c][std::slice(DIM,DIM,1)];
}
template void invert(M(float)& m);
template void invert(M(double)& m);


template<class T>
V(T) linearFit(const M(T)& x, const V(T)& y) {
#ifdef DEBUG
	if(x.size()==0) {
		throw std::invalid_argument("linearFit(): empty x");
	}
	for(size_t i=1; i<x.size(); ++i) {
		if(x[0].size()!=x[i].size()) {
			throw std::invalid_argument("linearFit(): x has uneven columns");
		}
	}
	if(x[0].size()!=y.size()) {
		throw std::invalid_argument("linearFit(): x and y length mismatch");
	}
#endif
	const size_t N=x.size();
	
	// first compute x' * x:
	const V(T) vn(N);
	M(T) p(vn,N);
	for(size_t i=0; i<N; ++i) {
		for(size_t j=0; j<=i; ++j) {
			p[j][i] = p[i][j] = (x[i]*x[j]).sum(); //std::inner_product(x[i].begin(),x[i].end(),x[j].begin(),0);
		}
	}
	
	invert(p); // invert that
	
	// could multiply by x' and then y in turn...
	/*
	std::vector<V(T) > a(N,x[0]);
	for(size_t i=0; i<N; ++i) {
		a[i] *= p[i][0];
		for(size_t j=1; j<N; ++j) {
			a[i] += x[j]*p[i][j];
		}
	}
	V(T) ans(N);
	for(size_t i=0; i<N; ++i) {
		ans[i] = (a[i]*y).sum(); //std::inner_product(a[i].begin(),a[i].end(),y.begin(),0);
	}
	return ans;
	*/
	
	// ... but faster to find x' * y first, then apply that to p
	V(T) r(N);
	for(size_t i=0; i<N; ++i)
		r[i] = (x[i]*y).sum();
	
	V(T) acc = p[0]*r[0];
	for(size_t i=1; i<N; ++i)
		acc += p[i]*r[i];
	return acc;
}
template V(float) linearFit(const M(float)& x, const V(float)& y);
template V(double) linearFit(const M(double)& x, const V(double)& y);

}

#ifdef LEAST_SQUARES_TEST_MAIN
#include <iostream>
#include <vector>
using namespace std;

template<class T> ostream& operator<<(ostream& os, V(T) a) {
	for(size_t i=0; i<a.size(); ++i)
		os << a[i] << ' ';
	return os;
}

void runtest() {
	size_t S=100;
	
	{
		std::vector<float> x(S), y(S);
		float s=0;
		for(size_t i=0; i<S; ++i) {
			float t = float(i)/S*2-1;
			x[i] = t;
			y[i] = random() / float(1<<31);
			s+=y[i];
		}
		{
			cout << "noise average: ";
			V(float) a = least_squares::polyFit(0,x,y);
			cout << a << "vs " << s/S << endl;
		}
		{
			cout << "noise fit: ";
			V(float) a = least_squares::polyFit(1,x,y);
			cout << a << endl;
		}
	}
	
	valarray<valarray<float> > x(valarray<float>(S),3);
	valarray<float> y(S);
	
	float m=3.214;
	float b=1.2;
	
	for(size_t i=0; i<S; ++i) {
		float t = float(i)/S*2-1;
		x[0][i] = 1;
		x[1][i] = t;
		x[2][i] = t*t;
		//y[i] = m*t + b;
	}
	y = m*x[1] + b*x[0];
	
	{
		cout << "linear fit: ";
		V(float) a = least_squares::linearFit(x,y);
		cout << a << endl;
	}
	
	{
		cout << "poly fit:   ";
		V(float) a = least_squares::polyFit(2, x[1], y);
		cout << a << endl;
		
		cout << "poly abs error sum: ";
		cout << abs(least_squares::polyApply(a,x[1]) - y).sum() << endl;
	}
	
	float arrf[] = { 1, 2, 3 };
	std::valarray<float> vaf(arrf,3);
	std::vector<float> vef(arrf,arrf+3);
	V(float) t1 = least_squares::polyApply(vaf,vef);
	cout << t1 << endl;
	V(float) t2 = least_squares::polyApply(vef,vaf);
	cout << t2 << endl;
	
	/*cout << "[ ";
	for(unsigned int i=0; i<4; ++i) {
		for(unsigned int j=0; j<4; ++j) {
			m[j][i] = random() / float(1<<31);
			f(i,j) = m[j][i];
			cout << m[j][i] << (j==3?";":", "); 
		}
		cout << (i==3?"]\n":"\n");
	}*/
	
}

int main(int argc, char** argv) {
	try {
		runtest();
	} catch(const std::exception& ex) {
		cerr << ex.what() << endl;
		return 1;
	}
	return 0;
}
#endif
