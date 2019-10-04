#ifndef INCLUDED_least_squares_h
#define INCLUDED_least_squares_h

#include <valarray>
#include <stdexcept>

/*!

Least Squares Solver
by Ethan Tira-Thompson, © 2011

Solves linear systems using Ordinary Least Squares.

Provides the namespace 'least_squares'.

There are two solver functions:

linearFit(x,y)
    Returns the parameters 'a' such that y = x·a minimizes squared error.
    x should be a valarray< valarray<T> >, where each x[i] is a column
    of data with length matching that of the valarray y.

polyFit(degree,x,y)
    Performs a polynomial fit of degree n:
      y = [ 1 x .. xⁿ ] · a
    given x and y (as vector or valarray)  Results will be in order of
    increasing power (i.e. index 0 is constant term, 1 is linear term, etc.)

Additionally, there are two helper functions:

polyApply(poly,x)
    Computes the polynomial of 'x' given polynomial terms 'poly'.
    These arguments can be either vector or valarray.

invert(x)
    Inverts a square valarray matrix 'x', overwriting results in-place.
    Throws std::underflow_error if the matrix is deficient.

TODO: does not handle degenerate/low-rank conditions very well.
Ideally, the fit functions should handle internally instead of allowing
invert()'s std::underflow_error to pass through

LICENSE:
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

// can't template a typedef, so use macro :-P
// (we'll undef these again at the end so we don't pollute global namespace)
#define V(T) std::valarray< T >
#define M(T) std::valarray< V(T) >


//! Returns parameters 'a' via ordinary least squares such that y = x·a
/*! Each element of @a x should be a valarray with length matching that of @a y.
 *  Will then compute invert(x' * x) * x' * y to find the fit.
 *
 *  TODO: does not handle degenerate/low-rank conditions.  Ideally, this
 *  should be handled internally instead of allowing invert()'s
 *  std::underflow_error to pass through. */
template<class T>
V(T) linearFit(const M(T)& x, const V(T)& y);

//! Returns fit 'a' of @a degree powers of @a x: y = [ 1 x .. xⁿ ] · a
/*! Accepts vector or valarray... results will be in order of
 *  increasing power (i.e. 0 is constant term, 1 is linear term, etc.)
 *
 *  TODO: does not handle degenerate/low-rank conditions.  Ideally, this
 *  should be handled internally instead of allowing invert()'s
 *  std::underflow_error to pass through. */
template<class Vx, class Vy>
V(typename Vx::value_type) polyFit(size_t degree, const Vx& x, const Vy& y);

//! Computes the polynomial of x given terms from @a poly.
/*! Accepts either vector and valarray arguments... @a poly terms should
 *  be in order of increasing degree à la polyFit() */
template<class Vp, class Vx>
V(typename Vx::value_type) polyApply(const Vp& poly, const Vx& x);

//! Invert a matrix, in-place, using Gauss-Jordan elimination
/*! Throws std::underflow_error if the matrix is deficient. */
template<class T>
void invert(M(T)& m);


/* Extern specializations allow us to move this template code to the .cpp file.
 * The downside is template will be undefined other than float and double...
 * Add more specializations here (e.g. complex) or else move template code
 * back into header. */
extern template void invert(M(float)& m);
extern template void invert(M(double)& m);
extern template V(float) linearFit(const M(float)& x, const V(float)& y);
extern template V(double) linearFit(const M(double)& x, const V(double)& y);


template<class Vx, class Vy>
V(typename Vx::value_type) polyFit(size_t degree, const Vx& x, const Vy& y) {
#ifdef DEBUG
	if(degree>0) {
		if(x.size()!=y.size()) {
			throw std::invalid_argument("polyFit(): x and y length mismatch");
		}
	}
	if(y.size()<degree) {
		throw std::invalid_argument("polyFit(): insufficient values for degree");
	}
#endif
	typedef typename Vx::value_type T;
	const size_t N=y.size();
	M(T) xn(V(T)(1,N),degree+1);
	if(degree>0) {
		memcpy(&xn[1][0], &x[0], sizeof(T)*N);
		for(size_t i=2; i<=degree; ++i) {
			xn[i] = xn[i-1]*xn[1];
		}
	}
	return linearFit(xn,V(T)(&y[0],N));
}


template<class Vp, class Vx>
V(typename Vx::value_type) polyApply(const Vp& poly, const Vx& x) {
	typedef typename Vx::value_type T;
	const size_t N=x.size();
	if(poly.size()==0)
		return V(T)(N);
	if(poly.size()==1)
		return V(T)(poly[0],N);
	V(T) v(N); // need to copy x into valarray in case it's a vector
	memcpy(&v[0],&x[0],sizeof(T)*N);
	V(T) acc(v*poly[1]); // linear term
	acc+=poly[0]; // constant term
	if(poly.size()>2) {
		V(T) p(v); // going to have to compute powers
		for(size_t i=2; i<poly.size(); ++i) {
			p*=v;
			acc += p*poly[i];
		}
	}
	return acc;
}

#undef V
#undef M

}

#endif
