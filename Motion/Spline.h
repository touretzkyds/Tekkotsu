//-*-c++-*-
/*========================================================================
    spline.h : Implementation  of Uniform and Non-Uniform Hermite Splines 
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

#ifndef __SPLINE_H__
#define __SPLINE_H__

#define HSPLINE HermiteSplineSegment<point,fnum>
#define NUHSPLINE NonUniformHermiteSplineSegment<point,fnum>

#define HSPLINE_TEM    template <class point,class fnum>
#define NUHSPLINE_TEM  template <class point,class fnum>

// #define TSPLINE  HermiteSplineSegment<fnum,fnum>
// #define CRSPLINE CatmullRomSpline<point,fnum>
// #define CRSPLINE_TEM template <class point,class fnum>

#define EPS 1E-6

// #define DEBUG

HSPLINE_TEM
class HermiteSplineSegment{
public:
	HermiteSplineSegment() : a(),b(),c(),d() {}
  point a,b,c,d;
public:
  void create(point x1,point x2,point dx1,point dx2);
  void create(point *pts,int num,int i);
  point eval(fnum u);
  point eval_deriv(fnum u);
};

NUHSPLINE_TEM
class NonUniformHermiteSplineSegment{
public:
	NonUniformHermiteSplineSegment() : a(),b(),c(),d(),t() {}
  point a,b,c,d;
  fnum t;
public:
  void create(point x0,point x1,point dx0,point dx1,fnum t1);
  void create(point *pts,double t1,int num,int i);
  point eval(fnum u);
  point eval_deriv(fnum u);
};

/*
template <class point,class fnum>
class CatmullRomSpline{
  typedef HermiteSplineSegment<point,fnum> segment;
  segment *seg;
  int num_segs,max;
  double 
public:
  CatmullRomSpline()
    {seg=NULL; num_segs=max=0;}

  bool allocate(int num);
  int find_segment(fnum t);

  //  void create(point *pts,num *time,point dx1,point dx2,int num);
  bool create(point *pts,fnum *time,int num);
  bool create_loop(point *pts,fnum *time,int num);
  point eval(fnum t);
  point eval_deriv(fnum t);
};
*/


//==== Hermite Spline Segment Implementation ====//

HSPLINE_TEM
void HSPLINE::create(point x1,point x2,point dx1,point dx2)
{
  // a = (x1 *  2) + (x2 * -2) + (dx1 *  1) + (dx2 *  1);
  // b = (x1 * -3) + (x2 *  3) + (dx1 * -2) + (dx2 * -1);
  a = (x1 - x2)*2 + dx1 + dx2;
  b = (x2 - x1)*3 - dx1*2 - dx2;
  c = dx1;
  d = x1;
}

HSPLINE_TEM
void HSPLINE::create(point *pts,int num,int i)
{
  point x1,x2,dx1,dx2;

  x1 = pts[(i + 1) % num];
  x2 = pts[(i + 2) % num];
  dx1 = x2 - pts[(i + 0) % num];
  dx2 = pts[(i + 3) % num] - x1;

  create(x1,x2,dx1,dx2);
}

HSPLINE_TEM
point HSPLINE::eval(fnum u)
{
  fnum u2,u3;
  point p;

  u2 = u*u;
  u3 = u*u*u;

  p = a*u3 + b*u2 + c*u + d;

  return(p);
}

HSPLINE_TEM
point HSPLINE::eval_deriv(fnum u)
{
  fnum u2;
  point p;

  u2 = u*u;

  p = a*(3*u2) + b*(2*u) + c;

  return(p);
}


//==== Non-Uniform Hermite Spline Segment Implementation ====//

NUHSPLINE_TEM
void NUHSPLINE::create(point x0,point x1,point dx0,point dx1,fnum t1)
{
  t = t1;
  d = x0;
  c = dx0;
  // b = ((x1-c*t1-d)*3 - dx1*t1) / (t1*t1);
  // a = (dx1 - b*t1*2 - c) / (t1*t1*3);
  b = ((x1 - x0)*3 - dx0*(t*2) - dx1*t) / (t*t);
  a = (dx1 - dx0 - b*(t*2)) / (3*t*t);

#ifdef DEBUG
	/*  point tx0,tx1,tdx0,tdx1;

  tx0 = eval(0.0) - x0;
  tx1 = eval(t1) - x1;
  tdx0 = eval_deriv(0.0) - dx0;
  tdx1 = eval_deriv(t1) - dx1;

  / *
  printf("Error: %f\n",
         tx0.sqlength() + tx1.sqlength() +
         tdx0.sqlength() + tdx1.sqlength());
  * /

  printf("Error: x0(%f,%f,%f) x1(%f,%f,%f) dx0(%f,%f,%f) dx1(%f,%f,%f)\n",
         tx0.x, tx0.y, tx0.z,
         tx1.x, tx1.y, tx1.z,
         tdx0.x, tdx0.y, tdx0.z,
         tdx1.x, tdx1.y, tdx1.z);*/
#endif
}

NUHSPLINE_TEM
void NUHSPLINE::create(point *pts,double t1,int num,int i)
// Not 100% sure this is a logical thing to calculate
{
  point x0,x1,dx0,dx1;

  x0 = pts[(i + 1) % num];
  x1 = pts[(i + 2) % num];
  dx0 = (x1 - pts[(i + 0) % num]) / (2*t1);
  dx1 = (pts[(i + 3) % num] - x0) / (2*t1);

  create(x0,x1,dx0,dx1,t1);
}

NUHSPLINE_TEM
point NUHSPLINE::eval(fnum u)
{
  fnum u2,u3;
  point p;

  u2 = u*u;
  u3 = u*u*u;

  p = a*u3 + b*u2 + c*u + d;

  return(p);
}

NUHSPLINE_TEM
point NUHSPLINE::eval_deriv(fnum u)
{
  fnum u2;
  point p;

  u2 = u*u;

  p = a*(3*u2) + b*(2*u) + c;

  return(p);
}

//==== Catmull-Rom Spline Implementation ====//

/*
HSPLINE_TEM
void makeSpline(HSPLINE *seg,const point *pts,int num)
{
  int i;

  for(i=0; i<num-3; i++){
    seg[i].create(pts[i+1],pts[i+2],pts[i+2] - pts[i+0],pts[i+3] - pts[i+1]);
  }
}

HSPLINE_TEM
point evalSpline(HSPLINE *seg,int num,double t)
{
  int s;

  if(t >= num) t = num - EPS;
  if(t <= 0.0) t = 0.0;
  s = (int)t;

  return(seg[s].eval(t - s));
}

HSPLINE_TEM
void makeSpline(HSPLINE *seg,TSPLINE *tseg,const point *pts,fnum time,int num)
{
  int i;

  for(i=0; i<num-3; i++){
    tseg[i].create(time[i+1],time[i+2],time[i+2] - time[i+0],time[i+3] - time[i+1]);
    seg[i].create(pts[i+1],pts[i+2],pts[i+2] - pts[i+0],pts[i+3] - pts[i+1]);
  }
}

HSPLINE_TEM
point evalSpline(HSPLINE *seg,TSPLINE *tseg,int num,double t)
{
  int s;

  // find time to evaluate point spline
  if(t >= num) t = num - EPS;
  if(t <= 0.0) t = 0.0;
  s = (int)t;
  t = tseg[s].eval(t - s);

  // evaluate it
  s = (int)t;
  return(seg[s].eval(t - s));
}
*/

/*
CRSPLINE_TEM
bool CRSPLINE::allocate(int num)
{
  current = 0;

  if(num <= max){
    num_segs = num;
    return(true);
  }

  if(seg) delete(seg);
  seg = new segment[num];

  if(seg){
    max = num_segs = num;
    return(true);
  }else{
    max = num_segs = 0;
    return(false);
  }
}

CRSPLINE_TEM
bool CRSPLINE::create(point *pts,fnum *ntime,int num)
{
  if(!allocate(num)) return(false);

  // TODO

  return(false);
}

CRSPLINE_TEM
bool CRSPLINE::create_loop(point *pts,fnum *time,int num)
{
  point ldx,dx;
  fnum sum,t;
  int i;

  if(!allocate(num)) return(false);

  ldx = (pts[1] - pts[num-1]) / time[0];
  sum = 0.0;

  for(i=0; i<num_segs; i++){
    dx = (pts[(i+2)%num_segs] - pts[i]) / time[(i+1)%num_segs];
    t = time[i];

    // printf("i = %d\n",i); fflush(stdout);
    seg[i].create(pts[i],pts[(i+1)%num_segs],ldx*t,dx*t,sum,t);

    sum += t;
    ldx = dx;
  }

  return(true);
}

CRSPLINE_TEM
int CRSPLINE::find_segment(fnum t)
{
  fnum d;
  int i;

  d = t - seg[current].start;
  if(d>=0 && d<=seg[current].time) return(current);

  for(i=0; i<num_segs; i++){
    d = t - seg[i].start;
    if(d>=0 && d<=seg[i].time) return(current = i);
  }

  return(current = num_segs - 1);
}

CRSPLINE_TEM
point CRSPLINE::eval(fnum t)
{
  int i;
  i = find_segment(t);
  return(seg[i].eval(t));
}

CRSPLINE_TEM
point CRSPLINE::eval_deriv(fnum t)
{
  int i;
  i = find_segment(t);
  return(seg[i].eval_deriv(t));
}
*/

/*! @file
 * @brief Performs calculations regarding splines for path execution
 * @author James R. Bruce (Creator)
 * 
 * @verbatim
  ========================================================================
    spline.h : Implementation  of Uniform and Non-Uniform Hermite Splines 
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
// __SPLINE_H__

