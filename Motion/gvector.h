//-*-c++-*-
/*========================================================================
    Vector.h : Simple vector class for 2D and 3D vectors
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

#ifndef __GVECTOR_H__
#define __GVECTOR_H__

#include <math.h>
#include "Shared/Util.h"

#define V3COMP(p) p.x,p.y,p.z
#define V2COMP(p) p.x,p.y

namespace GVector {

//#define point3d vector3d
//#define point2d vector2d

/*
inline int sqrt(int n){
  return((int)sqrt((double)n));
}
*/

//=====================================================================//
//  Vector3D Class
//=====================================================================//

template <class num>
class vector3d{
public:
  num x,y,z;

  vector3d() : x(),y(),z()
    {}
  vector3d(num nx,num ny,num nz) : x(nx),y(ny),z(nz)
    {}

  void set(num nx,num ny,num nz)
    {x=nx; y=ny; z=nz;}
  void set(vector3d<num> p)
    {x=p.x; y=p.y; z=p.z;}

  vector3d<num> &operator=(const vector3d<num> p)
    {set(p); return(*this);}

  num length() const;
  num sqlength() const;
  vector3d<num> norm() const;
  void normalize();

  num dot(const vector3d<num> p) const;
  vector3d<num> cross(const vector3d<num> p) const;

  vector3d<num>& operator+=(const vector3d<num> p);
  vector3d<num>& operator-=(const vector3d<num> p);
  vector3d<num>& operator*=(const vector3d<num> p);
  vector3d<num>& operator/=(const vector3d<num> p);

  vector3d<num> operator+(const vector3d<num> p) const;
  vector3d<num> operator-(const vector3d<num> p) const;
  vector3d<num> operator*(const vector3d<num> p) const;
  vector3d<num> operator/(const vector3d<num> p) const;

  vector3d<num> operator*(num f) const;
  vector3d<num> operator/(num f) const;
  vector3d<num>& operator*=(num f);
  vector3d<num>& operator/=(num f);

  vector3d<num> operator-() const;

  bool operator==(const vector3d<num> p) const;
  bool operator!=(const vector3d<num> p) const;
  bool operator< (const vector3d<num> p) const;
  bool operator> (const vector3d<num> p) const;
  bool operator<=(const vector3d<num> p) const;
  bool operator>=(const vector3d<num> p) const;

  vector3d<num> rotate_x(const double a) const;
  vector3d<num> rotate_y(const double a) const;
  vector3d<num> rotate_z(const double a) const;
};

template <class num>
num vector3d<num>::length() const
{
  return(sqrt(x*x + y*y + z*z));
}

template <class num>
num vector3d<num>::sqlength() const
{
  return(x*x + y*y + z*z);
}

template <class num>
vector3d<num> vector3d<num>::norm() const
{
  vector3d<num> p;
  num l;

  l = sqrt(x*x + y*y + z*z);
  p.x = x / l;
  p.y = y / l;
  p.z = z / l;

  return(p);
}

template <class num>
void vector3d<num>::normalize()
{
  num l;

  l = sqrt(x*x + y*y + z*z);
  x /= l;
  y /= l;
  z /= l;
}

template <class num>
num vector3d<num>::dot(const vector3d<num> p) const
{
  return(x*p.x + y*p.y + z*p.z);
}

template <class num>
num dot(const vector3d<num> a,const vector3d<num> b)
{
  return(a.x*b.x + a.y*b.y + a.z*b.z);
}

template <class num>
vector3d<num> vector3d<num>::cross(const vector3d<num> p) const
{
  vector3d<num> r;

  // right handed
  r.x = y*p.z - z*p.y;
  r.y = z*p.x - x*p.z;
  r.z = x*p.y - y*p.x;

  return(r);
}

template <class num>
vector3d<num> cross(const vector3d<num> a,const vector3d<num> b)
{
  vector3d<num> r;

  // right handed
  r.x = a.y*b.z - a.z*b.y;
  r.y = a.z*b.x - a.x*b.z;
  r.z = a.x*b.y - a.y*b.x;

  return(r);
}

#define VECTOR3D_EQUAL_BINARY_OPERATOR(opr) \
  template <class num> \
  vector3d<num>& vector3d<num>::operator opr (const vector3d<num> p) \
  {                  \
    x opr p.x;   \
    y opr p.y;   \
    z opr p.z;   \
    return(*this);   \
  }

VECTOR3D_EQUAL_BINARY_OPERATOR(+=)
VECTOR3D_EQUAL_BINARY_OPERATOR(-=)
VECTOR3D_EQUAL_BINARY_OPERATOR(*=)
VECTOR3D_EQUAL_BINARY_OPERATOR(/=)

#define VECTOR3D_BINARY_OPERATOR(opr) \
  template <class num> \
  vector3d<num> vector3d<num>::operator opr (const vector3d<num> p) const \
  {                  \
    vector3d<num> r; \
    r.x = x opr p.x; \
    r.y = y opr p.y; \
    r.z = z opr p.z; \
    return(r);       \
  }

VECTOR3D_BINARY_OPERATOR(+)
VECTOR3D_BINARY_OPERATOR(-)
VECTOR3D_BINARY_OPERATOR(*)
VECTOR3D_BINARY_OPERATOR(/)

#define VECTOR3D_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector3d<num> vector3d<num>::operator opr (const num f) const \
  {                  \
    vector3d<num> r; \
    r.x = x opr f;   \
    r.y = y opr f;   \
    r.z = z opr f;   \
    return(r);       \
  }

VECTOR3D_SCALAR_OPERATOR(*)
VECTOR3D_SCALAR_OPERATOR(/)

#define VECTOR3D_EQUAL_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector3d<num>& vector3d<num>::operator opr (num f) \
  {                \
    x opr f;   \
    y opr f;   \
    z opr f;   \
    return(*this); \
  }

VECTOR3D_EQUAL_SCALAR_OPERATOR(*=)
VECTOR3D_EQUAL_SCALAR_OPERATOR(/=)

#define VECTOR3D_LOGIC_OPERATOR(opr,combine) \
  template <class num> \
  bool vector3d<num>::operator opr (const vector3d<num> p) const \
  {                            \
    return((x opr p.x) combine \
           (y opr p.y) combine \
           (z opr p.z));       \
  }

VECTOR3D_LOGIC_OPERATOR(==,&&)
VECTOR3D_LOGIC_OPERATOR(!=,||)

VECTOR3D_LOGIC_OPERATOR(< ,&&)
VECTOR3D_LOGIC_OPERATOR(> ,&&)
VECTOR3D_LOGIC_OPERATOR(<=,&&)
VECTOR3D_LOGIC_OPERATOR(>=,&&)

template <class num>
vector3d<num> vector3d<num>::operator-() const
{
  vector3d<num> r;
  r.x = -x;
  r.y = -y;
  r.z = -z;
  return(r);
}

// returns point rotated around X axis by <a> radians (right handed)
template <class num>
vector3d<num> vector3d<num>::rotate_x(const double a) const
{
  vector3d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = x;
  q.y = c*y + -s*z;
  q.z = s*y + c*z;

  return(q);
}

// returns point rotated around Y axis by <a> radians (right handed)
template <class num>
vector3d<num> vector3d<num>::rotate_y(const double a) const
{
  vector3d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = c*x + s*z;
  q.y = y;
  q.z = -s*x + c*z;

  return(q);
}

// returns point rotated around Z axis by <a> radians (right handed)
template <class num>
vector3d<num> vector3d<num>::rotate_z(const double a) const
{
  vector3d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = c*x + -s*y;
  q.y = s*x + c*y;
  q.z = z;

  return(q);
}

// returns distance between two points
template <class num>
num distance(const vector3d<num> a,const vector3d<num> b)
{
  num dx,dy,dz;

  dx = a.x - b.x;
  dy = a.y - b.y;
  dz = a.z - b.z;

  return(sqrt(dx*dx + dy*dy + dz*dz));
}

// returns square of distance between two points
template <class num>
num sdistance(const vector3d<num> a,const vector3d<num> b)
{
  num dx,dy,dz;

  dx = a.x - b.x;
  dy = a.y - b.y;
  dz = a.z - b.z;

  return(dx*dx + dy*dy + dz*dz);
}

// returns distance from point p to line x0-x1
template <class num>
num distance_to_line(const vector3d<num> x0,const vector3d<num> x1,const vector3d<num> p)
{
  vector3d<num> x;
  num t;

  t = ((p.x - x0.x) + (p.y - x0.y) + (p.z - x0.z)) / (x1.x + x1.y + x1.z);
  x = x0 + (x1 - x0) * t;

  return(distance(x,p));
}


//=====================================================================//
//  Vector2D Class
//=====================================================================//

template <class num>
class vector2d{
public:
  num x,y;

  vector2d() : x(), y()
    {}
  vector2d(num nx,num ny) : x(nx), y(ny)
    {}

  void set(num nx,num ny)
    {x=nx; y=ny;}
  void set(vector2d<num> p)
    {x=p.x; y=p.y;}
  vector2d<num> &operator=(vector2d<num> p)
    {set(p); return(*this);}

  num length() const;
  num sqlength() const;
  num angle() const
    {return(atan2a(y,x));}

  vector2d<num> norm() const;
  void normalize();

  num dot(const vector2d<num> p) const;
  vector2d<num> cross(const vector2d<num> p) const;

  vector2d<num> operator+=(const vector2d<num> p);
  vector2d<num> operator-=(const vector2d<num> p);
  vector2d<num> operator*=(const vector2d<num> p);
  vector2d<num> operator/=(const vector2d<num> p);

  vector2d<num> operator+(const vector2d<num> p) const;
  vector2d<num> operator-(const vector2d<num> p) const;
  vector2d<num> operator*(const vector2d<num> p) const;
  vector2d<num> operator/(const vector2d<num> p) const;

  vector2d<num> operator*(const num f) const;
  vector2d<num> operator/(const num f) const;
  vector2d<num> operator*=(num f);
  vector2d<num> operator/=(num f);

  vector2d<num> operator-() const;

  bool operator==(const vector2d<num> p) const;
  bool operator!=(const vector2d<num> p) const;
  bool operator< (const vector2d<num> p) const;
  bool operator> (const vector2d<num> p) const;
  bool operator<=(const vector2d<num> p) const;
  bool operator>=(const vector2d<num> p) const;

  vector2d<num> rotate(const num a) const;
};

template <class num>
num vector2d<num>::length() const
{
  return(sqrt(x*x + y*y));
}

template <class num>
num vector2d<num>::sqlength() const
{
  return(x*x + y*y);
}

template <class num>
vector2d<num> vector2d<num>::norm() const
{
  vector2d<num> p;
  num l;

  l = sqrt(x*x + y*y);
  p.x = x / l;
  p.y = y / l;

  return(p);
}

template <class num>
void vector2d<num>::normalize()
{
  num l;

  l = sqrt(x*x + y*y);
  x /= l;
  y /= l;
}

template <class num>
num vector2d<num>::dot(const vector2d<num> p) const
{
  return(x*p.x + y*p.y);
}

template <class num>
num dot(const vector2d<num> a,const vector2d<num> b)
{
  return(a.x*b.x + a.y*b.y);
}

/*
template <class num>
vector2d<num> vector2d<num>::cross(const vector2d<num> p) const
{
  vector2d<num> r;

  // right handed
  r.x = y*p.z - z*p.y;
  r.y = z*p.x - x*p.z;

  return(r);
}
*/

#define VECTOR2D_EQUAL_BINARY_OPERATOR(opr) \
  template <class num> \
  vector2d<num> vector2d<num>::operator opr (const vector2d<num> p) \
  {                  \
    x = x opr p.x;   \
    y = y opr p.y;   \
    return(*this);   \
  }

VECTOR2D_EQUAL_BINARY_OPERATOR(+=)
VECTOR2D_EQUAL_BINARY_OPERATOR(-=)
VECTOR2D_EQUAL_BINARY_OPERATOR(*=)
VECTOR2D_EQUAL_BINARY_OPERATOR(/=)

#define VECTOR2D_BINARY_OPERATOR(opr) \
  template <class num> \
  vector2d<num> vector2d<num>::operator opr (const vector2d<num> p) const \
  {                  \
    vector2d<num> r; \
    r.x = x opr p.x; \
    r.y = y opr p.y; \
    return(r);       \
  }

VECTOR2D_BINARY_OPERATOR(+)
VECTOR2D_BINARY_OPERATOR(-)
VECTOR2D_BINARY_OPERATOR(*)
VECTOR2D_BINARY_OPERATOR(/)

#define VECTOR2D_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector2d<num> vector2d<num>::operator opr (const num f) const \
  {                  \
    vector2d<num> r;  \
    r.x = x opr f;   \
    r.y = y opr f;   \
    return(r);       \
  }

VECTOR2D_SCALAR_OPERATOR(*)
VECTOR2D_SCALAR_OPERATOR(/)

#define VECTOR2D_EQUAL_SCALAR_OPERATOR(opr) \
  template <class num> \
  vector2d<num> vector2d<num>::operator opr (num f) \
  {                \
    x = x opr f;   \
    y = y opr f;   \
    return(*this); \
  }

VECTOR2D_EQUAL_SCALAR_OPERATOR(*=)
VECTOR2D_EQUAL_SCALAR_OPERATOR(/=)

#define VECTOR2D_LOGIC_OPERATOR(opr,combine) \
  template <class num> \
  bool vector2d<num>::operator opr (const vector2d<num> p) const \
  {                            \
    return((x opr p.x) combine \
           (y opr p.y));       \
  }

VECTOR2D_LOGIC_OPERATOR(==,&&)
VECTOR2D_LOGIC_OPERATOR(!=,||)

VECTOR2D_LOGIC_OPERATOR(< ,&&)
VECTOR2D_LOGIC_OPERATOR(> ,&&)
VECTOR2D_LOGIC_OPERATOR(<=,&&)
VECTOR2D_LOGIC_OPERATOR(>=,&&)


template <class num>
vector2d<num> vector2d<num>::operator-() const
{
  vector2d<num> r;
  r.x = -x;
  r.y = -y;
  return(r);
}

template <class num>
vector2d<num> vector2d<num>::rotate(const num a) const {
  vector2d<num> q;
  double s,c;

  s = sin(a);
  c = cos(a);

  q.x = c*x + -s*y;
  q.y = s*x + c*y;

  return(q);
}

template <class num>
num distance(const vector2d<num> a,const vector2d<num> b)
{
  num dx,dy;

  dx = a.x - b.x;
  dy = a.y - b.y;

  return(sqrt(dx*dx + dy*dy));
}

// returns square of distance between two points
template <class num>
num sdistance(const vector2d<num> a,const vector2d<num> b)
{
  num dx,dy;

  dx = a.x - b.x;
  dy = a.y - b.y;

  return(dx*dx + dy*dy);
}

// returns distance from point p to line x0-x1
template <class num>
num distance_to_line(const vector2d<num> x0,const vector2d<num> x1,const vector2d<num> p)
{
  vector2d<num> x;
  num t;

  t = ((p.x - x0.x) + (p.y - x0.y)) / (x1.x + x1.y);
  x = x0 + (x1 - x0) * t;

  // printf("dist:(%f,%f)-(%f,%f)\n",x.x,x.y,p.x,p.y);

  return(distance(x,p));
}

// returns perpendicular offset from line x0-x1 to point p
template <class num>
num offset_to_line(const vector2d<num> x0,const vector2d<num> x1,const vector2d<num> p)
{
  vector2d<num> n,v;

  // get normal to line
  n = x1 - x0;
  n.set(n.y,-n.x);

  v = p - x0;

  return(n.dot(v));
}

//==== Generic functions =============================================//
// (work on 2d or 3d vectors)

// returns nearest point on line segment x0-x1 to point p
template <class vector>
vector point_on_segment(const vector x0,const vector x1,const vector p)
{
  vector q,dx;
  double f,l;

  dx = x1 - x0;
  f = dot(dx,p-x0);
  l = dx.sqlength();

  if(f <= 0.0) return(x0); // this handles x0=x1 case also
  if(f > l) return(x1);

  q = x0 + dx * (f / sqrt(l));

  return(q);
}

// ray: r0=origin of ray, rd=direction vector of ray
// plane: p0=point on plane, pn=plane normal
// result = point on plane where ray intersects
//   if ray intersects plane (going forward or backward) set to point of intersection otherwise not touched
// returns true if ray intersects plane (going forward along direction)
template <class vector>
bool intersect_ray_plane(const vector r0, const vector rd, const vector p0, const vector pn, vector &result) {
  double t;
  double d;

  if(rd.dot(pn)==0.0)
    return false;

  d = -p0.dot(pn);

  t = -(r0.dot(pn) + d) / (rd.dot(pn));

  result = r0 + rd * t;

  return true;
}

} // namespace GVector

/*! @file
 * @brief vector class to aid in mathematical vector calculations
 * @author James R. Bruce (Creator)
 * 
 * @verbatim
  ========================================================================
    Vector.h : Simple vector class for 2D and 3D vectors
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
// __GVECTOR_H__
