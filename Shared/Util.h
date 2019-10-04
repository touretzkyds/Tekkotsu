//-*-c++-*-
/*=======================================================================
    Util.h
  -----------------------------------------------------------------------
    Numerical utility functions
  -----------------------------------------------------------------------
    Copyright 1999, 2000, 2001 James R. Bruce
    School of Computer Science, Carnegie Mellon University
  -----------------------------------------------------------------------
    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to: Free Software Foundation,
    Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
  =======================================================================*/

#ifndef __UTIL_H__
#define __UTIL_H__

#include <cmath>
#include <string.h>

const double TODEG = (180.0 / M_PI);  // 57.295779513082;

#if (!defined(__SGI_STL_ALGOBASE_H) && !defined(ALGOBASE_H))
/*
template <class num>
inline num max(num a,num b)
{
  return((a > b)? a : b);
}

template <class num>
inline num min(num a,num b)
{
  return((a < b)? a : b);
}


template <class data>
inline void swap(data &a,data &b)
{
  data t;
  t = a;
  a = b;
  b = t;
}
*/

#endif

template <class num1,class num2>
inline num1 bound(num1 x,num2 low,num2 high)
{
  if(x < low ) x = low;
  if(x > high) x = high;
  return(x);
}

template <class num>
inline num max3(num a,num b,num c)
{
  if(a > b){
    return((a > c)? a : c);
  }else{
    return((b > c)? b : c);
  }
}

template <class num>
inline num min3(num a,num b,num c)
{
  if(a < b){
    return((a < c)? a : c);
  }else{
    return((b < c)? b : c);
  }
}

template <class num>
inline void sort(num &a,num &b,num &c)
{
  if(a > b) swap(a,b);
  if(b > c) swap(b,c);
  if(a > b) swap(a,b);
}

template <class real>
real fmodt(real x,real m)
// Does a real modulus the *right* way, using
// truncation instead of round to zero.
{
  return(x - floor(x / m)*m);
}

template <class num1,class num2>
inline num1 setbits(num1 val,num2 bits)
{
  return(val |= bits);
}

template <class num1,class num2>
inline num1 clearbits(num1 val,num2 bits)
{
  return(val &= ~bits);
}

template <class real>
real saw(real t)
{
  t -= floor(t);
  return(2 * ((t < 0.5)? t : (1.0-t)));
}

template <class data>
inline int mcopy(data *dest,data *src,int num)
{
  int i;

  for(i=0; i<num; i++) dest[i] = src[i];

  return(num);
}

template <class data>
inline data mset(data *dest,data val,int num)
{
  int i;

  for(i=0; i<num; i++) dest[i] = val;

  return(val);
}

template <class data>
inline void mzero(data &d)
{
  memset(&d,0,sizeof(d));
}


template <class node>
int list_length(node *list)
{
  node *p = list;
  int num = 0;

  while(p){
    num++;
    p = p->next;
  }

  return(num);
}

// normalize the supplied angle to lie in (-M_PI, M_PI]
// NOTE: this one passes value and returns the adjusted angle
inline double norm_angle(double angle)
{
  angle=fmod(angle,2*M_PI);

  if(angle <   0.0) angle += 2*M_PI;
  if(angle >= M_PI) angle -= 2*M_PI;

  return(angle);
}

// NOTE: This one passed by reference
inline void angle_wrap(double &angle) {
  angle=fmod(angle,2*M_PI);
  if (angle < 0.0) angle+=2*M_PI;
  if (angle >= M_PI) angle-=2*M_PI;
}

// return the normalized angle halfway between these two
// assumes the arguments are already normalized to (-M_PI, M_PI].
inline double avg_angle(double left, double right) {
  if (left < right) {
    left += 2*M_PI;
  }
  double result = (left+right)/2;
  if (result > M_PI) {
    result -= 2*M_PI;
  }
  
  return result;
}

// wrapper for handling edge cases in atan2
// TODO: is this really neccessary?
inline double atan2a(double y, double x) {
  if (x == 0.0) {
    if (y == 0.0) {
      return 0.0;
    } else if (y > 0) return M_PI/2.0; 
    else return -M_PI/2.0;
  }
  else return atan2(y, x);
}

inline double atan2b(double y, double x)
{
  if(fabs(x) < 1.0E-4){
    if(y == 0.0) return(0.0);
    return((y > 0)? M_PI/2.0 : -M_PI/2.0);
  }
  else return atan2(y, x);
}

template <class num>
inline int sign(num n)
{
  if(n > 0) return( 1);
  if(n < 0) return(-1);
  return(0);
}

// value less than minimum is clipped to 0.0 to avoid underflow
template <class num>
inline num gaussian_with_min(num x,num min_value)
{
  num val;

  val = std::exp(-(x*x)/2);

  if(val < min_value)
    val = 0;

  return(val);
}

const double gaussian_constant = sqrt(2*M_PI);

inline double gaussian_prob(double x, double mean, double sigma) {
  double offset = x-mean;
  double top = exp(-offset*offset/(2*sigma*sigma));
  double bottom = gaussian_constant*sigma;
  
  if (bottom < 1e-10) {
    if (offset < 1e-10) {
      return 1/1e-10;
    } else {
      return 0;
    }
  }
  
  return top/bottom;
}

/*
template <class node>
node *random_element(node *list)
{
  node *p = list;
  int num = 0;

  while(p){
    num++;
    p = p->next;
  }

  return(num);
}
*/

/*! @file
 * @brief Numerical Utilities
 * @author James R. Bruce (Creator)
 *
 * @verbatim
  =======================================================================
    Util.h
  -----------------------------------------------------------------------
    Numerical utility functions
  -----------------------------------------------------------------------
    Copyright 1999, 2000, 2001 James R. Bruce
    School of Computer Science, Carnegie Mellon University
  -----------------------------------------------------------------------
    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to: Free Software Foundation,
    Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
		=======================================================================
 * @endverbatim
 */

#endif
