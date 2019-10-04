#ifndef INCLUDED_BoundingBox_H
#define INCLUDED_BoundingBox_H

#include "fmatCore.h"
#include <limits>
#include <cmath>
#include <cstdlib>

//! Bounding box of a shape; used for coordinate calculations
template<size_t N, typename T=fmat::fmatReal>
class BoundingBox {
public:
	static const BoundingBox ALL_INCLUSIVE; //!< a bounding box initialized to contain everything: (-∞,∞)
	static const BoundingBox ALL_EXCLUSIVE; //!< a bounding box initialized to exclude everything: (∞,-∞) — this is also default construction
	
	fmat::Column<N,T> min; //!< minimum corner
	fmat::Column<N,T> max; //!< maximum corner

	//! Constructor, defaults to empty bounding box (aka ALL_EXCLUSIVE)
	/*! Using inverted infinities, so no point should collide,
	    and any expansion will collapse to the expansion. */
	BoundingBox() :
		min( std::numeric_limits<T>::infinity()),
		max(-std::numeric_limits<T>::infinity()) {}

	//! Constructor for 2D boxes
	BoundingBox(T xmin, T ymin, T xmax, T ymax) :
		min(), max() { min[0]=xmin; min[1]=ymin; max[0]=xmax; max[1]=ymax; }
	
	//! Conversion between types (same dimensionality)
	template<typename R> BoundingBox(const BoundingBox<N,R>& bb) : min(bb.min), max(bb.max) {}
	
	//! Conversion between type/dimensions.
	/*! If @a inclusive is set and this is an 'upcast' (D < N), extra dimensions will be set to all-inclusive (-∞,∞).
	 *  Otherwise, extra dimensions will be set to all-exclusive (∞,-∞) and you would need to provide
	 *  additional expansion in order to form a positive volume. */
	template<size_t D, typename R> explicit BoundingBox(const BoundingBox<D,R>& bb, bool inclusive=true) :
		min(inclusive?-std::numeric_limits<T>::infinity():std::numeric_limits<T>::infinity()),
		max(inclusive?std::numeric_limits<T>::infinity():-std::numeric_limits<T>::infinity())
	{
		const size_t MIN=(D<N)?D:N;
		(fmat::SubVector<MIN,T>)min = fmat::SubVector<MIN,const R>(bb.min);
		(fmat::SubVector<MIN,T>)max = fmat::SubVector<MIN,const R>(bb.max);
	}
	
	//! Constructor for single-point box (note collision requires non-zero area)
	template<class P> explicit BoundingBox(const P& p) : min(p), max(p) {}

	//! Constructor, specify min/max directly (you are responsible to ensure min << max)
	template<class P> explicit BoundingBox(const P& _min, const P& _max) : min(_min), max(_max) {}
	
	//! Returns true if box has zero or negative area
	bool empty() const { return !(min << max); }
	
	//! Sets min and max to default values (negative infinite area)
	void clear() { max = -(min = std::numeric_limits<T>::infinity()); }
	
	//! Return the width of dimenion @a x (Cowabunga Dude!)
	T getDimension(size_t x) const { return max[x]-min[x]; }
	
	//! Return the width of each dimension
	fmat::Column<N,T> getDimensions() const { return max-min; }
	
	//! Returns the center of the bounding box
	fmat::Column<N,T> getCenter() const { return (min+max)/2; }
	
	//! Returns a random point within the bounding box (including edges)
	fmat::Column<N,T> getRandom() const {
		fmat::Column<N,T> ans = fmat::fmat_internal::NoInit();
		for(size_t i=0; i<N; ++i) {
			// Aperios doesn't provide a random(), only rand()...
			// For testing consistency, Aibo target models always use rand(), regardless of local/aperios platform.
#if defined(TGT_ERS210) || defined(TGT_ERS220) || defined(TGT_ERS2xx) || defined(TGT_ERS7) || defined(PLATFORM_APERIOS)
			ans[i] = rand() / (T)RAND_MAX * (max[i]-min[i]) + min[i];
#else
			ans[i] = random() / (T)((1U<<31)-1) * (max[i]-min[i]) + min[i];
#endif
		}
		return ans;
	}
	
	//! Expand the bounding box to include @a p
	/*! This picks up columns of the correct size */
	template<class R> void expand(const fmat::Column<N,R>& p) { min.minimize(p); max.maximize(p); }
	
	//! Expand the bounding box to include @a p
	/*! This picks up subvectors of the correct size */
	template<class R> void expand(const fmat::SubVector<N,R>& p) { min.minimize(p); max.maximize(p); }
	
	//! Expand the bounding box to include @a p
	/*! This version picks up extended-dimenion points, arrays, and column subclasses, converting to a SubVector intermediary.
	 *  Allows implicit truncation of 3D to 2D, since this is pretty commonly used in DualCoding. */
	template<class P> void expand(const P& p) {
		fmat::SubVector<N,const T> pp(p);
		min.minimize(pp); max.maximize(pp);
	}

	//! Expand the bounding box to include @a bb
	void expand(const BoundingBox& bb) {
		min.minimize(bb.min);
		max.maximize(bb.max);
	}
	
	//! Applies the specified rotation about the bounding box center, then resets to the axis-aligned boundaries of the rotated bounding box
	/*! This does not handle bounding boxes with infinite bounds. */
	void rotate(fmat::Matrix<N,N> r) { // pass-by-copy because we modify it during execution
		fmat::Column<N,T> c = getCenter();
#if defined(DEBUG) && !defined(PLATFORM_APERIOS)
		for(size_t i=0; i<N; ++i) {
			if(!std::isfinite(c[i])) {
				std::cerr << "BoundingBox::rotate() applied to infinite boundaries" << std::endl;
				break;
			}
		}
#endif
		r.abs();
		fmat::Column<N,T> x = r*(max - c);
		max = c + x;
		min = c - x;
	}
	
	//! Translates the bounding box, moving the min and max bounds
	void translate(const fmat::Column<N>& x) {
		min+=x;
		max+=x;
	}
	
	void transform(const fmat::Matrix<N,N>& r, const fmat::Column<N>& x) {
		rotate(r);
		recenter(r*getCenter() + x);
	}
	
	//! Translates the bounding box, moving the min and max bounds such that the center is placed at @a c
	void recenter(const fmat::Column<N>& c) {
		fmat::Column<N,T> d = getDimensions()/2;
		min = c - d;
		max = c + d;
	}
	
	//! Return true if this box includes p (note edge coincidence does not count)
	template<class P> bool collides(const P& p) const {
		return ( min << p ) && ( p << max );
	}
	
	//! Return true if this box intersects p (note edge coincidence does not count)
	bool collides(const BoundingBox& bb) const {
		for(size_t i=N; i>0; ) {
			--i;
			// if we find an axis which does not overlap, then bb do not overlap
			if( (bb.max[i]<=min[i]) || (max[i]<=bb.min[i]) )
				return false;
		}
		// if all axes' components overlap, then bb overlap
		return true;
	}
	
	//! Returns a corner of the box as determined by which bits are set
	/*! The ith bit corresponds to the ith axis,
		0 returns lower bound, and 1 returns upper bound */
	fmat::Column<N,T> getCorner(size_t x) const {
		fmat::Column<N,T> ans=min;
		for(size_t i=N; i>0; ) {
			--i;
			if( (x>>i)&1 )
				ans[i]=max[i];
		}
		return ans;
	}

	//! For display/serialization
	friend std::ostream& operator<<(std::ostream& out, const BoundingBox& bb) {
		out << "BoundingBox(";
		size_t i=0;
		do {
			out << bb.min[i] << ":" << bb.max[i];
		} while(++i < N && (out << ", "));
		out << ")";
		return out;
	}
	
	// equality test
	bool operator==(const BoundingBox& bb) const { return min==bb.min && max==bb.max; }
	
	// inequality test 
	bool operator!=(const BoundingBox& bb) const { return min!=bb.min || max!=bb.max; }
};

template<size_t N, typename T> const BoundingBox<N,T>
BoundingBox<N,T>::ALL_INCLUSIVE(-std::numeric_limits<T>::infinity(),std::numeric_limits<T>::infinity());

template<size_t N, typename T> const BoundingBox<N,T>
BoundingBox<N,T>::ALL_EXCLUSIVE(std::numeric_limits<T>::infinity(),-std::numeric_limits<T>::infinity());

extern template class BoundingBox<2>;
extern template class BoundingBox<3>;

typedef BoundingBox<2> BoundingBox2D;
typedef BoundingBox<3> BoundingBox3D;

#endif
