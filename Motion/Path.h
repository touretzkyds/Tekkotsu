//-*-c++-*-
/*========================================================================
    path.h : Template for building spline paths
  ------------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
		========================================================================*/

#ifndef __SPLINE_PATH_H__
#define __SPLINE_PATH_H__

#include "Shared/Util.h"
#include "Spline.h"

#define SPATH SplinePath<point,fnum>
#define SPATH_TEM template <class point,class fnum>

SPATH_TEM
class SplinePath{
  NonUniformHermiteSplineSegment<point,fnum> s;
  double t0,t1;
  point x0,x1,dx0,dx1;
public:
	SplinePath() : s(), t0(), t1(), x0(),x1(),dx0(),dx1() {}
  void init(point x,point dx);
  void add(point x,point dx,double length,double t);

  point eval(double t);
  point eval_deriv(double t);
};


//==== Spline Path Queue Implementation ====//

SPATH_TEM
void SPATH::init(point x,point dx)
{
  x0  = x1  = x;
  dx0 = dx1 = dx;
  t0  = t1  = 0;
}

SPATH_TEM
void SPATH::add(point x,point dx,double length,double t)
{
  x0  = eval(t);
  dx0 = eval_deriv(t);

  x1  = x;
  dx1 = dx;

  t0  = t;
  t1  = t + length;

  s.create(x0,x1,dx0,dx1,length);
}

SPATH_TEM
point SPATH::eval(double t)
{
  if(t < t0){
    return(x0 + dx0*(t-t0));
  }else if(t >= t1){
    return(x1 + dx1*(t-t1));
  }else{
    return(s.eval(t - t0));
  }
}

SPATH_TEM
point SPATH::eval_deriv(double t)
{
  if(t < t0){
    return(dx0);
  }else if(t >= t1){
    return(dx1);
  }else{
    return(s.eval_deriv(t - t0));
  }
}

/*! @file
 * @brief Performs calculations regarding splines for path execution
 * @author James R. Bruce (Creator)
 * 
 * @verbatim
  ========================================================================
    path.h : Template for building spline paths
  ------------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
		========================================================================
 * @endverbatim
 */

#endif
// __SPLINE_PATH_H__
