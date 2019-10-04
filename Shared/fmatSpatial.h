//-*-c++-*-
#ifndef INCLUDED_fmatSpatial_h_
#define INCLUDED_fmatSpatial_h_

#include "fmatCore.h"

namespace fmat {
	
	template<typename R> class TransformT;
	
	// ************************* rotation matrices ******************** //
	
	//! returns 2x2 rotation matrix (i.e. implied rotation about Z), for angle @a rad in radians
	template<typename R>
	inline Matrix<2,2,R> rotation2DT(R rad) {
		R s = std::sin(rad), c = std::cos(rad);
		R dat[4] = { c, s, -s, c };
		return Matrix<2,2,R>(dat);
	}
	//! returns 2x2 rotation matrix for angle @a rad (in radians) about Z axis
	inline Matrix<2,2,fmatReal> rotation2D(fmatReal rad) { return rotation2DT<fmatReal>(rad); }
	
	//! returns NxN rotation matrix for angle @a rad (in radians) about X axis (only uses upper 3x3)
	template<size_t N, typename R>
	inline Matrix<N,N,R> rotationXN(R rad) {
		(void)sizeof(fmat_internal::CompileTimeAssert<Rotation_requires_larger_dimensions,N>=3>);
		Matrix<N,N,R> ans = Matrix<N,N,R>::identity();
		ans(1,1) = ans(2,2) = std::cos(rad);
		ans(1,2) = -(ans(2,1) = std::sin(rad));
		return ans;
	}
	//! returns 3x3 rotation matrix for angle @a rad (in radians) about X axis
	inline Matrix<3,3,fmatReal> rotationX(fmatReal rad) { return rotationXN<3,fmatReal>(rad); }

	//! returns NxN rotation matrix for angle @a rad (in radians) about Y axis (only uses upper 3x3)
	template<size_t N, typename R>
	inline Matrix<N,N,R> rotationYN(R rad) {
		(void)sizeof(fmat_internal::CompileTimeAssert<Rotation_requires_larger_dimensions,N>=3>);
		Matrix<N,N,R> ans = Matrix<N,N,R>::identity();
		ans(0,0) = ans(2,2) = std::cos(rad);
		ans(2,0) = -(ans(0,2) = std::sin(rad));
		return ans;
	}
	//! returns 3x3 rotation matrix for angle @a rad (in radians) about Z axis
	inline Matrix<3,3,fmatReal> rotationY(fmatReal rad) { return rotationYN<3,fmatReal>(rad); }
	
	//! returns NxN rotation matrix for angle @a rad (in radians) about Z axis (only uses upper 2x2)
	template<size_t N, typename R>
	inline Matrix<N,N,R> rotationZN(R rad) {
		(void)sizeof(fmat_internal::CompileTimeAssert<Rotation_requires_larger_dimensions,N>=2>);
		if(N==2) {
			R s = std::sin(rad), c = std::cos(rad);
			R dat[4] = { c, s, -s, c };
			return Matrix<N,N,R>(dat);
		} else {
			Matrix<N,N,R> ans = Matrix<N,N,R>::identity();
			ans(0,0) = ans(1,1) = std::cos(rad);
			ans(0,1) = -(ans(1,0) = std::sin(rad));
			return ans;
		}
	}
	//! returns 3x3 rotation matrix for angle @a rad (in radians) about Z axis
	inline Matrix<3,3,fmatReal> rotationZ(fmatReal rad) { return rotationZN<3,fmatReal>(rad); }
	
	
	
	// ************************* Quaternion ******************** //
	
	//! Quaternions can be more efficient and more stable for a series of rotations than a corresponding 3x3 matrix, also more compact storage
	template<class R=fmatReal>
	class QuaternionT {
	public:
		//! Default constructor, initialize to identity (0 rotation)
		QuaternionT() : w(1), x(0), y(0), z(0) {}
		
		//! Explicit construction from elements (careful, does not check normalization!)
		QuaternionT(R w_, R x_, R y_, R z_) : w(w_), x(x_), y(y_), z(z_) {}
		
		//! copies from another representation, assuming operator[] is supported and w is index 0
		template<typename T> static QuaternionT from(const T& q) { return QuaternionT(q[0],q[1],q[2],q[3]); }
		//! copies from another representation, assuming operator[] is supported and w is index 0
		template<typename T> QuaternionT& importFrom(const T& q) { w=q[0]; x=q[1]; y=q[2]; z=q[3]; return *this; }
		//! copies into another representation, assuming operator[] is supported and w is index 0
		template<typename T> T exportTo() const { T q; q[0]=w; q[1]=x; q[2]=y; q[3]=z; return q; }
		//! copies into another representation, assuming operator[] is supported and w is index 0
		template<typename T> T& exportTo(T& q) const { q[0]=w; q[1]=x; q[2]=y; q[3]=z; return q; }
		//! unpacks into specified storage
		template<typename T> void exportTo(T& w_, T& x_, T& y_, T& z_) const { w_=w; x_=x; y_=y; z_=z; }
		//! unpacks into specified storage, skipping #w which can be reconstitued from √(1-x^2-y^2-z^2), see fromAxis()
		template<typename T> void exportTo(T& x_, T& y_, T& z_) const { if(w<0) { x_=-x; y_=-y; z_=-z; } else { x_=x; y_=y; z_=z; } }
		
		R getW() const { return w; } //!< returns #w
		R getX() const { return x; } //!< returns #x
		R getY() const { return y; } //!< returns #y
		R getZ() const { return z; } //!< returns #z
		
		bool operator==(const QuaternionT<R> &other) const { return w==other.w && x==other.x && y==other.y && z==other.z; }

		//! returns a no-op quaternion (0 rotation), the same as the default constructor, but can re-use this instance instead of creating new ones all the time
		static const QuaternionT& identity() { return IDENTITY; }
		static const QuaternionT IDENTITY; //!< identity instance
		
		//! generate quaternion representing rotation of @a rad radians about X axis
		static QuaternionT<R> aboutX(R rad) { return fmat::QuaternionT<R>(std::cos(rad/2),std::sin(rad/2),0,0); }
		//! generate quaternion representing rotation of @a rad radians about Y axis
		static QuaternionT<R> aboutY(R rad) { return fmat::QuaternionT<R>(std::cos(rad/2),0,std::sin(rad/2),0); }
		//! generate quaternion representing rotation of @a rad radians about Z axis
		static QuaternionT<R> aboutZ(R rad) { return fmat::QuaternionT<R>(std::cos(rad/2),0,0,std::sin(rad/2)); }
		
		//! Generate quaternion from axis-angle representation (@a angle in radians, @a axis will be re-normalized if non-zero @a angle)
		/*! If axis is 0 length, initializes to identity (0 rotation) */
		template<class T>
		static QuaternionT fromAxisAngle(const T& axis, R angle) {
			if(std::abs(angle) <= std::numeric_limits<R>::epsilon())
				return QuaternionT(); // 0 rotation, axis does not matter
			R n = axis.sumSq();
			if(n <= std::numeric_limits<R>::epsilon())
				return QuaternionT(); // invalid axis, just use identity
			R s = std::sin(angle/2)/std::sqrt(n);
			return QuaternionT(std::cos(angle/2), axis[0]*s, axis[1]*s, axis[2]*s);
		}
		
		//! Generate quaternion from just the axis component of another quaternion (i.e. @a axis magnitude should be less than 1; 0 magnitude means no rotation)
		/*! This method requires only basic operations on the input type, so you can pass a raw array or plist::Point... */
		template<class T>
		static QuaternionT fromAxis(const T& axis) {
			R n2 = static_cast<R>(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
			if(n2>1) {
				if(n2>1+std::numeric_limits<R>::epsilon()*10) {
					R n = std::sqrt(n2);
					std::cerr << "Warning: fmat::Quaternion axis constructor passed non-normalized |" << axis[0] << ',' << axis[1] << ',' << axis[2] << "| = " << n << " (err " << (n-1) << ')' << std::endl;
					R s = 1/n;
					return QuaternionT(0, axis[0]*s, axis[1]*s, axis[2]*s);
				} else {
					// minor rounding error
					R s = 1/n2; // not bothering with sqrt, ensures it goes below 1
					return QuaternionT(0, axis[0]*s, axis[1]*s, axis[2]*s);
				}
			} else {
				return QuaternionT(std::sqrt(1-n2),axis[0],axis[1],axis[2]);
			}
		}
		
		//! Generate quaternion from rotation matrix, this assumes the rotation matrix is well-formed
		/*! This implementation is based on cross pollination between "Angel"'s code on this page:
		 *    http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
		 *  and the Ogre3D Quaternion implementation, which itself references:
		 *    Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
		 *    article "Quaternion Calculus and Fast Animation". */
		template<class T>
		static QuaternionT fromMatrix(const T& rot) {
			QuaternionT quat = fmat_internal::NoInit();
			R tracem1 = rot(0,0) + rot(1,1) + rot(2,2);
			if( tracem1 >= 0 ) {
				const R r = std::sqrt(tracem1 + 1);
				quat.w = (R)0.5 * r;
				const R s = (R)0.5 / r;
				quat.x = ( rot(2,1) - rot(1,2) ) * s;
				quat.y = ( rot(0,2) - rot(2,0) ) * s;
				quat.z = ( rot(1,0) - rot(0,1) ) * s;
			} else {
				if ( rot(0,0) > rot(1,1) && rot(0,0) > rot(2,2) ) {
					const R r = std::sqrt( 1.0f + rot(0,0) - rot(1,1) - rot(2,2));
					quat.x = (R)0.5 * r;
					const R s = (R)0.5 / r;
					quat.w = (rot(2,1) - rot(1,2) ) * s;
					quat.y = (rot(0,1) + rot(1,0) ) * s;
					quat.z = (rot(0,2) + rot(2,0) ) * s;
				} else if (rot(1,1) > rot(2,2)) {
					const R r = std::sqrt( 1.0f + rot(1,1) - rot(0,0) - rot(2,2));
					quat.y = (R)0.5 * r;
					const R s = (R)0.5 / r;
					quat.w = (rot(0,2) - rot(2,0) ) * s;
					quat.x = (rot(0,1) + rot(1,0) ) * s;
					quat.z = (rot(1,2) + rot(2,1) ) * s;
				} else {
					const R r = std::sqrt( 1.0f + rot(2,2) - rot(0,0) - rot(1,1) );
					quat.z = (R)0.5 * r;
					const R s = (R)0.5 / r;
					quat.w = (rot(1,0) - rot(0,1) ) * s;
					quat.x = (rot(0,2) + rot(2,0) ) * s;
					quat.y = (rot(1,2) + rot(2,1) ) * s;
				}
			}
			return quat;
		}
		
		//! returns a 3x3 rotation matrix representation
		/*! implemented by simplified version of operator*(*this, Matrix<3,3>::identity()) */
		Matrix<3,3,R> toMatrix() const {
			const R t2 =   w*x;
			const R t3 =   w*y;
			const R t4 =   w*z;
			const R t5 =  -x*x;
			const R t6 =   x*y;
			const R t7 =   x*z;
			const R t8 =  -y*y;
			const R t9 =   y*z;
			const R t10 = -z*z;
			const R values[9] = {
				2*(t8 + t10) + 1,
				2*(t4 +  t6),
				2*(t7 -  t3),
				2*(t6 -  t4),
				2*(t5 + t10) + 1,
				2*(t2 +  t9),
				2*(t3 + t7),
				2*(t9 - t2),
				2*(t5 + t8) + 1
			};
			return Matrix<3,3,R>(values);
		}
		//! allows conversion/assignment to Matrix<3,3>
		operator Matrix<3,3,R>() const { return toMatrix(); }
		
		//! return axis of rotation represented by quaternion @a q (i.e. the axis component of axis-angle representation)
		Column<3,R> axis() const {
			R n2 = (x*x + y*y + z*z);
			if(n2 < std::numeric_limits<R>::epsilon())
				return fmat::packT<R>(0,0,1); // no rotation, unknown axis
			R sc = 1/std::sqrt(n2);
			return fmat::packT<R>(x*sc, y*sc, z*sc);
		}
		
		//! Return angle of rotation (range ±π radians) about the quaternion's native axis (i.e. the angle component for axis-angle representation)
		/*! Handles some slight denormalization gracefully. */
		R angle() const {
			R n = std::sqrt(x*x + y*y + z*z);
			R ang = 2*std::atan2(n,w); // the atan2 result is always positive because 'n' is always positive
			if(ang>static_cast<R>(M_PI)) // so ang is always positive, don't need a '< M_PI' case
				ang-=2*static_cast<R>(M_PI);
			return ang;
			// alternative... faster/better?
			/*if(w>1) return 2*std::acos(2-w);
			 else if(w<-1) return 2*std::acos(2+w);
			 else return 2*std::acos(w);*/
		}
		
		//! Return angle of rotation represented by the quaternion about an arbitrary axis (assumed to already be normalized)
		template<class T>
		R axisComponent(const T& v) const {
			/*R ang_2 = std::acos(w);
			 R sc = std::sin(ang_2);*/
			R n = std::sqrt(x*x + y*y + z*z);
			if(n < std::numeric_limits<R>::epsilon())
				return 0;
			R ang_2 = std::atan2(n,w);
			return 2*ang_2*(x*v[0] + y*v[1] + z*v[2])/n;
		}
		
		//! Returns yaw-pitch-roll aka heading-elevation-bank conversion, where roll-pitch-yaw correspond to compounding rotations about the global x, y, and z axis respectively (in that order)
		/*! From the "driver's seat" positive heading is turning to the left (z is up, using right hand rule, not compass heading),
		 *  positive pitch is looking down, and positive roll is spinning clockwise.  Within this frame-oriented view, we apply
		 *  rotation axes in the 'reverse' order: first z, then y, then x.
		 *
		 *  You can reconstruct a Quaternion from these values by: q · v = aboutZ(yaw) * aboutY(pitch) * aboutX(roll) · v
		 *  Note we right-multiply v to apply the rotation, so x is applied to an incoming vector first, then y, then z.
		 *
		 *  With thanks to http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
		 *  (note that page uses different axis mapping however!) */
		fmat::Column<3,R> ypr() const {
			const R sqw=w*w, sqx=x*x, sqy=y*y, sqz=z*z;
			const R unit = sqw + sqx + sqy + sqz; // if normalised is one, otherwise is correction factor
			const R test = y*w - x*z;
			if (test > 0.4999*unit) { // singularity at north pole
				return packT<R>(-2 * std::atan2(x,w), (R)M_PI_2, 0);
			}
			if (test < -0.4999*unit) { // singularity at south pole
				return packT<R>(2 * std::atan2(x,w), -(R)M_PI_2, 0);
			}
			return packT<R>(
				std::atan2(2*z*w+2*x*y , sqx - sqz - sqy + sqw),
				std::asin(2*test/unit),
				std::atan2(2*x*w+2*z*y , -sqx + sqz - sqy + sqw)
			);
		}
		
		//! Negates just the quaternion axis
		QuaternionT inverse() const ATTR_must_check { return QuaternionT(w,-x,-y,-z); }
		
		// ! Negates the entire quaternion (not the same as the inverse)
		/* ! Although I wonder if it should be the same as inverse(), this is what other quaternion implementations do */
		// Since it's not clear what this should do, it's left undefined
		//QuaternionT operator-() const ATTR_must_check { return QuaternionT(-w,-x,-y,-z); }
		
		//! multiply quaternions
		template<typename Rb>
		QuaternionT<typename fmat_internal::promotion_trait<R,Rb>::type>
		operator*(const QuaternionT<Rb>& q) const {
			return QuaternionT<typename fmat_internal::promotion_trait<R,Rb>::type>(
				w*q.w - x*q.x - y*q.y - z*q.z,
				w*q.x + x*q.w + y*q.z - z*q.y,
				w*q.y - x*q.z + y*q.w + z*q.x,
				w*q.z + x*q.y - y*q.x + z*q.w
			);
		}
		
		//! multiply quaternions
		QuaternionT operator*=(const QuaternionT& q) { return this->operator=(operator*(q)); }
		
		//! multiply 3-row matrix by quaternion
		template<template<size_t H, size_t W, typename Rt> class T, size_t W>
		Matrix<3,W,R> operator*(const T<3,W,R>& m) const {
			const R t2 =   w*x;
			const R t3 =   w*y;
			const R t4 =   w*z;
			const R t5 =  -x*x;
			const R t6 =   x*y;
			const R t7 =   x*z;
			const R t8 =  -y*y;
			const R t9 =   y*z;
			const R t10 = -z*z;
			Matrix<3,W,R> o=fmat_internal::NoInit();
			for(size_t c = 0; c<W; ++c) {
				o(0,c) = 2*( (t8 + t10)*m(0,c) + (t6 -  t4)*m(1,c) + (t3 + t7)*m(2,c) ) + m(0,c);
				o(1,c) = 2*( (t4 +  t6)*m(0,c) + (t5 + t10)*m(1,c) + (t9 - t2)*m(2,c) ) + m(1,c);
				o(2,c) = 2*( (t7 -  t3)*m(0,c) + (t2 +  t9)*m(1,c) + (t5 + t8)*m(2,c) ) + m(2,c);
			}
			return o;
		}
		
		//! multiply Transform by quaternion
		TransformT<R> operator*(const TransformT<R>& m) const {
			const R t2 =   w*x;
			const R t3 =   w*y;
			const R t4 =   w*z;
			const R t5 =  -x*x;
			const R t6 =   x*y;
			const R t7 =   x*z;
			const R t8 =  -y*y;
			const R t9 =   y*z;
			const R t10 = -z*z;
			TransformT<R> o=fmat_internal::NoInit();
			for(size_t c = 0; c<4; ++c) {
				o(0,c) = 2*( (t8 + t10)*m(0,c) + (t6 -  t4)*m(1,c) + (t3 + t7)*m(2,c) ) + m(0,c);
				o(1,c) = 2*( (t4 +  t6)*m(0,c) + (t5 + t10)*m(1,c) + (t9 - t2)*m(2,c) ) + m(1,c);
				o(2,c) = 2*( (t7 -  t3)*m(0,c) + (t2 +  t9)*m(1,c) + (t5 + t8)*m(2,c) ) + m(2,c);
			}
			return o;
		}
		
		//! multiply 3-element vector by quaternion
		Column<3,R> operator*(const Column<3,R> v) const {
			const R t2 =   w*x;
			const R t3 =   w*y;
			const R t4 =   w*z;
			const R t5 =  -x*x;
			const R t6 =   x*y;
			const R t7 =   x*z;
			const R t8 =  -y*y;
			const R t9 =   y*z;
			const R t10 = -z*z;
			Column<3,R> o=fmat_internal::NoInit();
			o[0] = 2*( (t8 + t10)*v[0] + (t6 -  t4)*v[1] + (t3 + t7)*v[2] ) + v[0];
			o[1] = 2*( (t4 +  t6)*v[0] + (t5 + t10)*v[1] + (t9 - t2)*v[2] ) + v[1];
			o[2] = 2*( (t7 -  t3)*v[0] + (t2 +  t9)*v[1] + (t5 + t8)*v[2] ) + v[2];
			return o;
		}
		
		//! returns sum of squares of components; should be "close" to 1, otherwise call normalize()
		R sumSq() const { return w*w + x*x + y*y + z*z; }
		
		//! returns magnitude of quaternion; should be "close" to 1, otherwise call normalize()
		R norm() const { return std::sqrt(w*w + x*x + y*y + z*z); }
		
		//! Re-normalize the quaternion magnitude to 1 (or 0 if norm is already invalid) and positive W, returns the previous magnitude
		R normalize() {
			R n = sumSq();
			if(n < std::numeric_limits<R>::epsilon()) {
				w=x=y=z=0; // can't normalize a zero vector, just clean it up
				return 0;
			}
			if(n<=1 && 1-n < std::numeric_limits<R>::epsilon()) {
				// already normalized, just fix w if needed
				if(w<0) {
					w=-w;
					x=-x;
					y=-y;
					z=-z;
				}
				return 1;
			}
			R sc = (w<0?-1:1) / std::sqrt(n);
			w*=sc;
			x*=sc;
			y*=sc;
			z*=sc;
			return n;
		}
		
		friend inline std::ostream& operator<<(std::ostream& os, const QuaternionT<R>& q) { return os << fmat::SubVector<4,const R>((const R*)&q.w); }
		
	protected:
		R w, x, y, z;
		
		//! no-op constructor for functions which fill in results with additional computation
		QuaternionT(const fmat_internal::NoInit&) : w(), x(), y(), z() {}
	};
	typedef QuaternionT<> Quaternion;
	
	//! returns the rotation needed to transform @a p into @a q (this is the core of a 'slerp' implementation)
	template<class Ra, class Rb>
	QuaternionT<typename fmat_internal::promotion_trait<Ra,Rb>::type>
	crossProduct(const QuaternionT<Ra>& p, const QuaternionT<Rb>& q) { return q * p.inverse(); }
	
	// forward to the specialized Quaternion invert
	template<typename R> QuaternionT<R> invert(const QuaternionT<R>& q) { return q.inverse(); }
	
	
	
	// ************************* Transform ******************** //
	
	namespace fmat_internal {
		//! Expanded calculations to facilitate optimized compilation of these operations for TransformT
		template<typename R1, typename R2>
		struct transformOps {
			//! The output type for these operations, e.g. float * double -> double
			typedef typename fmat_internal::promotion_trait<R1,R2>::type out_t;
			//! Transform a length-3 column
			template<typename V>
			inline static Column<3,out_t> multiply3(const TransformT<R1>& tr, const V& x) {
				Column<3,out_t> ans=fmat_internal::NoInit();
				const R1 *a = &tr(0,0);
				const R2 *b = &x[0];
				ans[0] = a[0+3*0]*b[0] + a[0+3*1]*b[1] + a[0+3*2]*b[2] + a[0+3*3];
				ans[1] = a[1+3*0]*b[0] + a[1+3*1]*b[1] + a[1+3*2]*b[2] + a[1+3*3];
				ans[2] = a[2+3*0]*b[0] + a[2+3*1]*b[1] + a[2+3*2]*b[2] + a[2+3*3];
				return ans;
			}
			//! Transform a length-4 column (homogenous coordinate)
			template<typename V>
			inline static Column<4,out_t> multiply4(const TransformT<R1>& tr, const V& x) {
				Column<4,out_t> ans=fmat_internal::NoInit();
				const R1 *a = &tr(0,0);
				const R2 *b = &x[0];
				ans[0] = a[0+3*0]*b[0] + a[0+3*1]*b[1] + a[0+3*2]*b[2] + a[0+3*3]*b[3];
				ans[1] = a[1+3*0]*b[0] + a[1+3*1]*b[1] + a[1+3*2]*b[2] + a[1+3*3]*b[3];
				ans[2] = a[2+3*0]*b[0] + a[2+3*1]*b[1] + a[2+3*2]*b[2] + a[2+3*3]*b[3];
				ans[3] = b[3];
				return ans;
			}
			//! Concatenate transforms
			inline static void multiplyT(const R1* a, const R2* b, out_t* ans) {
				ans[0+3*0] = a[0+3*0]*b[0+3*0] + a[0+3*1]*b[1+3*0] + a[0+3*2]*b[2+3*0];
				ans[1+3*0] = a[1+3*0]*b[0+3*0] + a[1+3*1]*b[1+3*0] + a[1+3*2]*b[2+3*0];
				ans[2+3*0] = a[2+3*0]*b[0+3*0] + a[2+3*1]*b[1+3*0] + a[2+3*2]*b[2+3*0];
				ans[0+3*1] = a[0+3*0]*b[0+3*1] + a[0+3*1]*b[1+3*1] + a[0+3*2]*b[2+3*1];
				ans[1+3*1] = a[1+3*0]*b[0+3*1] + a[1+3*1]*b[1+3*1] + a[1+3*2]*b[2+3*1];
				ans[2+3*1] = a[2+3*0]*b[0+3*1] + a[2+3*1]*b[1+3*1] + a[2+3*2]*b[2+3*1];
				ans[0+3*2] = a[0+3*0]*b[0+3*2] + a[0+3*1]*b[1+3*2] + a[0+3*2]*b[2+3*2];
				ans[1+3*2] = a[1+3*0]*b[0+3*2] + a[1+3*1]*b[1+3*2] + a[1+3*2]*b[2+3*2];
				ans[2+3*2] = a[2+3*0]*b[0+3*2] + a[2+3*1]*b[1+3*2] + a[2+3*2]*b[2+3*2];
				ans[0+3*3] = a[0+3*0]*b[0+3*3] + a[0+3*1]*b[1+3*3] + a[0+3*2]*b[2+3*3] + a[0+3*3];
				ans[1+3*3] = a[1+3*0]*b[0+3*3] + a[1+3*1]*b[1+3*3] + a[1+3*2]*b[2+3*3] + a[1+3*3];
				ans[2+3*3] = a[2+3*0]*b[0+3*3] + a[2+3*1]*b[1+3*3] + a[2+3*2]*b[2+3*3] + a[2+3*3];
			}
		};
	}
	
	
	//! Efficient computation of affine transform operations
	template<typename R=fmatReal>
	class TransformT : public Matrix<3,4,R> {
	public:
		enum {
			HEIGHT=3, H=3,
			WIDTH=4, W=4,
			CAP=12
		};
		typedef R storage_t;
		typedef typename fmat_internal::unconst<R>::type out_t;
		
		TransformT(const fmat_internal::NoInit& noinit) : Matrix<H,W,R>(noinit) {}
		
		TransformT() : Matrix<H,W,R>(Matrix<H,W,R>::identity()) { }
		explicit TransformT(const R* x, size_t colStride=H) : Matrix<H,W,R>(x,colStride) {}
		TransformT(const SubMatrix<H,W,R>& x) : Matrix<H,W,R>(x) {}
		TransformT(const Matrix<H,W,R>& x) : Matrix<H,W,R>(x) {}
		template<typename Rot, typename Pt> TransformT(const Rot& r, const Pt& p) : Matrix<H,W,R>(fmat_internal::NoInit()) { rotation()=r; translation()=p; }
		template<size_t SH, size_t SW> explicit TransformT(const SubMatrix<SH,SW,R>& x) : Matrix<H,W,R>(x) {}
		template<size_t SH, size_t SW> explicit TransformT(const SubMatrix<SH,SW,R>& x, size_t rowOff, size_t colOff) : Matrix<H,W,R>(x,rowOff,colOff) {}
		template<size_t SH, size_t SW> explicit TransformT(const Matrix<SH,SW,R>& x) : Matrix<H,W,R>(x) {}
		template<size_t SH, size_t SW> explicit TransformT(const Matrix<SH,SW,R>& x, size_t rowOff, size_t colOff) : Matrix<H,W,R>(x,rowOff,colOff) {}
		
		SubMatrix<3,3,R> rotation() { return SubMatrix<3,3,R>(data); }
		SubMatrix<3,3,const R> rotation() const { return SubMatrix<3,3,const R>(data); }
		SubVector<3,R> translation() { return Matrix<H,W,R>::column(3); }
		SubVector<3,const R> translation() const { return Matrix<H,W,R>::column(3); }
		
		//! generate transform representing rotation of @a rad radians about X axis
		static TransformT<R> aboutX(R rad) {
			TransformT<R> ans;
			ans(1,1) = ans(2,2) = std::cos(rad);
			ans(1,2) = -(ans(2,1) = std::sin(rad));
			return ans;
		}
		//! generate transform representing rotation of @a rad radians about Y axis
		static TransformT<R> aboutY(R rad) {
			TransformT<R> ans;
			ans(0,0) = ans(2,2) = std::cos(rad);
			ans(2,0) = -(ans(0,2) = std::sin(rad));
			return ans;
		}
		//! generate transform representing rotation of @a rad radians about Z axis
		static TransformT<R> aboutZ(R rad) {
			TransformT<R> ans;
			ans(0,0) = ans(1,1) = std::cos(rad);
			ans(0,1) = -(ans(1,0) = std::sin(rad));
			return ans;
		}
		//! generate transform representing a translation of @a x
		template<class V>
		static TransformT<R> offset(const V& x) { TransformT<R> ans; ans.translation() = x; return ans; }
				
		static const TransformT& identity() { return IDENTITY; }
		static const TransformT IDENTITY; //!< identity instance
		
		//! returns the inverse transform, allowing for affine transformations (however, cannot invert 0 or inf scaling!)
		inline TransformT inverse() const ATTR_must_check {
			TransformT ans=fmat_internal::NoInit();
			ans.rotation() = fmat::invert(rotation());
			ans.translation() = -ans.rotation() * translation();
			return ans;
		}

		//! returns the inverse transform, assuming a rigid transform (zero scale and skew) for faster computation
		inline TransformT rigidInverse() const ATTR_must_check {
			TransformT ans=fmat_internal::NoInit();
			R * t2dat = &ans(0,0);
			t2dat[0] = data[0];
			t2dat[1] = data[3];
			t2dat[2] = data[6];
			t2dat[3] = data[1];
			t2dat[4] = data[4];
			t2dat[5] = data[7];
			t2dat[6] = data[2];
			t2dat[7] = data[5];
			t2dat[8] = data[8];
			t2dat[9]  = -(data[0]*data[9] + data[1]*data[10] + data[2]*data[11]);
			t2dat[10] = -(data[3]*data[9] + data[4]*data[10] + data[5]*data[11]);
			t2dat[11] = -(data[6]*data[9] + data[7]*data[10] + data[8]*data[11]);
			return ans;
		}
		
		template<typename R2>
		inline Column<3,typename fmat_internal::promotion_trait<R,R2>::type>
		operator*(const Column<3,R2>& x) const {
			return fmat_internal::transformOps<R,R2>::multiply3(*this,x);
		}
		
		template<typename R2>
		inline Column<3,typename fmat_internal::promotion_trait<R,R2>::type>
		operator*(const SubVector<3,R2>& x) const {
			return fmat_internal::transformOps<R,R2>::multiply3(*this,x);
		}
		
		template<size_t N, typename R2>
		inline Matrix<3,N,typename fmat_internal::promotion_trait<R,R2>::type>
		operator*(const Matrix<3,N,R2>& x) const {
			Matrix<3,N,typename fmat_internal::promotion_trait<R,R2>::type> ans = fmat_internal::NoInit();
			for(size_t i=0; i<N; ++i)
				ans.column(i) = fmat_internal::transformOps<R,R2>::multiply3(*this,x.column(i));
			return ans;
		}
		
		template<typename R2>
		inline Column<4,typename fmat_internal::promotion_trait<R,R2>::type>
		operator*(const Column<4,R2>& x) const {
			return fmat_internal::transformOps<R,R2>::multiply4(*this,x);
		}
		
		template<typename R2>
		inline Column<4,typename fmat_internal::promotion_trait<R,R2>::type>
		operator*(const SubVector<4,R2>& x) const {
			return fmat_internal::transformOps<R,R2>::multiply4(*this,x);
		}
		
		template<size_t N, typename R2>
		inline Matrix<4,N,typename fmat_internal::promotion_trait<R,R2>::type>
		operator*(const Matrix<4,N,R2>& x) const {
			Matrix<4,N,typename fmat_internal::promotion_trait<R,R2>::type> ans = fmat_internal::NoInit();
			for(size_t i=0; i<N; ++i)
				ans.column(i) = fmat_internal::transformOps<R,R2>::multiply4(*this,x.column(i));
			return ans;
		}
		
		template<typename R2>
		inline TransformT<typename fmat_internal::promotion_trait<R,R2>::type>
		operator*(const TransformT<R2>& tb) const {
			TransformT<typename fmat_internal::promotion_trait<R,R2>::type> mans=fmat_internal::NoInit();
			fmat_internal::transformOps<R,R2>::multiplyT(data,&tb(0,0),&mans(0,0));
			return mans;
		}
		
		template<typename R2>
		TransformT&
		operator*=(const TransformT<R2>& t) {
			fmat_internal::transformOps<R,R2>::multiplyT(&TransformT(*this)(0,0),&t(0,0),data);
			return *this;
		}
		using Matrix<H,W,R>::operator=;
		using Matrix<H,W,R>::operator*=;
		using Matrix<H,W,R>::operator/=;
		
	protected:
		using Matrix<H,W,R>::data;
	};
	typedef TransformT<> Transform;
	
	//! invert the matrix, taking advantage of known structure:
	template<typename R>
	inline Matrix<4,4,R> invertTransform(const Matrix<4,4,R>& t) {
		fmat::Column<4> p = -t.column(3);
		const R * tdat = &t(0,0);
		/*R t2dat[16] = {
		 tdat[0], tdat[4], tdat[8], 0,
		 tdat[1], tdat[5], tdat[9], 0,
		 tdat[2], tdat[6], tdat[10], 0,
		 fmat::dotProduct(t.column(0),p), fmat::dotProduct(t.column(1),p), fmat::dotProduct(t.column(2),p), tdat[15]
		 };
		 return Matrix<4,4,R>(t2dat);*/
		Matrix<4,4,R> t2(t);
		R * t2dat = &t2(0,0);
		t2dat[1] = tdat[4];
		t2dat[2] = tdat[8];
		t2dat[4] = tdat[1];
		t2dat[6] = tdat[9];
		t2dat[8] = tdat[2];
		t2dat[9] = tdat[6];
		t2dat[12] = fmat::dotProduct(t.column(0),p);
		t2dat[13] = fmat::dotProduct(t.column(1),p);
		t2dat[14] = fmat::dotProduct(t.column(2),p);
		return t2;
	}
	
	// forward to the specialized Transform invert
	template<typename R>
	TransformT<R>
	invert(const TransformT<R>& m) { return m.inverse(); }
	
	//! Returns yaw-pitch-roll aka heading-elevation-bank conversion, where roll-pitch-yaw correspond to compounding rotations about the global x, y, and z axis respectively (in that order)
	/*! From the "driver's seat" positive heading is turning to the left (z is up, using right hand rule, not compass heading),
	 *  positive pitch is looking down, and positive roll is spinning clockwise.  Within this frame-oriented view, we apply
	 *  rotation axes in the 'reverse' order: first z, then y, then x.
	 *
	 *  You can reconstruct a Quaternion from these values by: q · v = aboutZ(yaw) * aboutY(pitch) * aboutX(roll) · v
	 *  Note we right-multiply v to apply the rotation, so x is applied to an incoming vector first, then y, then z.
	 *
	 *  With thanks to http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
	 *  (note that page uses different axis mapping however!) */
	template<typename R>
	Column<3,R>
	ypr(const QuaternionT<R>& q) { return q.ypr(); }
	
	//! Returns yaw-pitch-roll aka heading-elevation-bank conversion, where roll-pitch-yaw correspond to compounding rotations about the global x, y, and z axis respectively (in that order)
	/*! From the "driver's seat" positive heading is turning to the left (z is up, using right hand rule, not compass heading),
	 *  positive pitch is looking down, and positive roll is spinning clockwise.  Within this frame-oriented view, we apply
	 *  rotation axes in the 'reverse' order: first z, then y, then x.
	 *
	 *  You can reconstruct a rotation from these values by: q · v = rotationZ(yaw) * rotationY(pitch) * rotationX(roll) · v
	 *  Note we right-multiply v to apply the rotation, so x is applied to an incoming vector first, then y, then z. */
	template<typename R>
	Column<3,R>
	ypr(const TransformT<R>& m) { return ypr(m.rotation()); }
	
	//! Returns yaw-pitch-roll aka heading-elevation-bank conversion, where roll-pitch-yaw correspond to compounding rotations about the global x, y, and z axis respectively (in that order)
	/*! From the "driver's seat" positive heading is turning to the left (z is up, using right hand rule, not compass heading),
	 *  positive pitch is looking down, and positive roll is spinning clockwise.  Within this frame-oriented view, we apply
	 *  rotation axes in the 'reverse' order: first z, then y, then x.
	 *
	 *  You can reconstruct a rotation from these values by: q · v = rotationZ(yaw) * rotationY(pitch) * rotationX(roll) · v
	 *  Note we right-multiply v to apply the rotation, so x is applied to an incoming vector first, then y, then z. */
	template<template<size_t,size_t,typename R> class M, typename R>
	Column<3,typename fmat_internal::unconst<R>::type>
	ypr(const M<3,3,R>& m) {
		typedef typename fmat_internal::unconst<R>::type out_t;
		if (m(2,0) > 0.9998) { // singularity at north pole
			return packT<out_t>(std::atan2(-m(0,1),m(1,1)), -M_PI_2, 0);
		}
		if (m(2,0) < -0.9998) { // singularity at south pole
			return packT<out_t>(std::atan2(-m(0,1),m(1,1)), M_PI_2, 0);
		}
		return packT<out_t>(
			std::atan2(m(1,0),m(0,0)),
			std::asin(-m(2,0)),
			std::atan2(m(2,1),m(2,2))
		);
	}
	
	//! Returns the scaling factors for d1 and d2 through p1 and p2 respectively to reach common intersection
	/*! May return infinity if d1 and d2 are parallel (including collinear).  */
	template<typename R>
	Column<2,R>
	segmentIntersection(const Column<2,R>& p1, const Column<2,R>& d1, const Column<2,R>& p2, const Column<2,R>& d2) {
		fmat::Matrix<2,2,R> dm = fmat_internal::NoInit();
		dm.column(0) = d1;
		dm.column(1) = -d2;
		try {
			dm = invert(dm);
		} catch(...) {
			// no solution, parallel lines, return infinity
			return Column<2,R>(std::numeric_limits<fmat::fmatReal>::infinity());
		}
		Column<2,R> o = p2-p1;
		return dm * o;
	}
	
	//! Returns the scaling factor of d1 from p1 to reach intersection of d2 through p2
	/*! May return infinity if d1 and d2 are parallel (including collinear).  */
	template<typename R>
	R
	rayIntersection(const Column<2,R>& p1, const Column<2,R>& d1, const Column<2,R>& p2, const Column<2,R>& d2) {
		return segmentIntersection(p1, d1, p2, d2)[0];
	}
	
	//! Returns the point of intersection of d1 through p1 and d2 through p2
	/*! May return point at infinity if d1 and d2 are parallel (including collinear),
	  *  such a point will be in the correct quadrant. */
	template<typename R>
	Column<2,R>
	lineIntersection(const Column<2,R>& p1, const Column<2,R>& d1, const Column<2,R>& p2, const Column<2,R>& d2) {
		return d1 * rayIntersection(p1,d1,p2,d2) + p1;
	}
	
	//! Returns the orthogonal left vector (rotate by 90°)
	template<template<size_t,typename R> class V, typename R>
	Column<2,typename V<2,R>::out_t>
	normalLeft(const V<2,R>& v) { return fmat::pack(-v[1],v[0]); }

	//! Returns the orthogonal right vector (rotate by -90°)
	template<template<size_t,typename R> class V, typename R>
	Column<2,typename V<2,R>::out_t>
	normalRight(const V<2,R>& v) { return fmat::pack(v[1],-v[0]); }
}

/*! @file
 * @brief Provides data structures and algorithms for spatial operations
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
