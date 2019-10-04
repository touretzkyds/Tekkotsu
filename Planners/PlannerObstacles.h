//-*-c++-*-
#ifndef INCLUDED_PlannerObstacles_H
#define INCLUDED_PlannerObstacles_H

#include <ostream>

#include "Shared/fmat.h"
#include "Shared/plist.h"
#include "Shared/FamilyFactory.h"
#include "Shared/BoundingBox.h"
#include <vector>

template<size_t N>
class PlannerObstacle;
typedef PlannerObstacle<2> PlannerObstacle2D;
typedef PlannerObstacle<3> PlannerObstacle3D;

class RectangularObstacle;
class CircularObstacle;
class EllipticalObstacle;
class ConvexPolyObstacle;
class HierarchicalObstacle;
class BoxObstacle;
class CylindricalObstacle;
class SphericalObstacle;
class EllipsoidObstacle;

namespace plist {
	//! supports use of plist::ArrayOf<PlannerObstacle> for polymorphic load/save
	template<> PlannerObstacle2D* loadXML(xmlNode* node);
	template<> PlannerObstacle3D* loadXML(xmlNode* node);
	//! not supported
	template<> inline PlannerObstacle2D* allocate() {
		throw std::runtime_error("cannot plist::allocate generic instances (PlannerObstacle2D)");
	}
	template<> inline PlannerObstacle3D* allocate() {
		throw std::runtime_error("cannot plist::allocate generic instances (PlannerObstacle3D)");
	}
	template<> void inline assign<PlannerObstacle2D>(PlannerObstacle2D&, PlannerObstacle2D const&) { throw std::runtime_error("cannot assign"); }
	template<> void inline assign<PlannerObstacle3D>(PlannerObstacle3D&, PlannerObstacle3D const&) { throw std::runtime_error("cannot assign"); }
}

namespace DualCoding {
	class ShapeRoot;
}

//! Base class for all obstacles
/*! Subclasses should remember to override operator= to avoid using the plist::Dictionary version
 *  (plist::Dictionary::operator= will only copy dictionary entries, skips other state */
template <size_t N>
class PlannerObstacle : public plist::Dictionary {
public:
	//! each collision shape is assigned a prime number, these provide fast dispatch in collides()
	enum ObstacleGeometry {
		RECTANGULAR_OBS=2,
		CIRCULAR_OBS=3,
		ELLIPTICAL_OBS=5,
		CONVEX_POLY_OBS=7,
		HIERARCHICAL_OBS=11,
		BOX_OBS=13,
		CYLINDRICAL_OBS=17,
		SPHERICAL_OBS=19,
		ELLIPSOID_OBS=23
	};
	
	//! Default constructor
	explicit PlannerObstacle(ObstacleGeometry geom, const std::string& t) : 
		plist::Dictionary(), name(), type(t), geometry(geom), shapeId(-1), bodyObstacle(false) { init(); }
	
	//! Destructor
	virtual ~PlannerObstacle() {}
	
	//! sgn(x): returns sign of input
	template <typename T>
	static int sgn(T x) { return x < 0 ? -1 : 1; }
	
	//! Sub-components of an obstacle
	plist::Primitive<std::string> name;
	
	static void convertShapeToPlannerObstacle(const DualCoding::ShapeRoot& shape, 
						  float inflation,
						  std::vector<PlannerObstacle*> &obstacles);
	
	//! Increases the size of the obstacle in all directions by at least @a amount
	//virtual void bloat(float amount) = 0;
	
	//! Decreases the size of the obstacle in all directions by at least @a amount
	//virtual void contract(float amount) = 0;
	
	//! moves the obstacle
	virtual void updatePosition(const fmat::SubVector<N,const fmat::fmatReal>& newPos) = 0;
	
	//! rotate the obstacle about a specified origin and a 2d rotation matrix
	virtual void rotate(const fmat::SubVector<N,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<N,N,const fmat::fmatReal>& rot) = 0;
	
	//! rotate the obstacle about its center by a 2d rotation matrix
	void rotate(const fmat::SubMatrix<N,N,const fmat::fmatReal>& rot) { rotate(getCenter(), rot); }
	
	//! get center point of obstacle
	virtual fmat::Column<N> getCenter() const = 0;
	
	//! get boundaries of the current obstacle
	virtual BoundingBox<N> getBoundingBox() const = 0;
	
	//! checks for collision with other PlannerObstacles
	bool collides(const PlannerObstacle<N>& other) const;
	
	//! All collision tests that need to be handled by subclasses for specific points
	virtual bool collides(const fmat::SubVector<N,const fmat::fmatReal>& point) const = 0;
	
	//! Support function for GJK collision detection algorithm.
	/*! Gets the point on the convex polygon farthest along the specified direction. */
	virtual fmat::Column<N> getSupport(const fmat::SubVector<N,const fmat::fmatReal>& direction) const = 0;
	
	//! Calculate the vector to closest point on the obstacle boundary from @a pt
	virtual fmat::Column<N> gradient(const fmat::SubVector<N,const fmat::fmatReal>& pt) const = 0;
	
	virtual std::string toString() const;
	friend std::ostream& operator<<(std::ostream& os, const PlannerObstacle& po) { return os << po.toString(); }
	
	ObstacleGeometry getObstacleGeometry() const { return geometry; }
	
	int getShapeId() const { return shapeId; }
	void setShapeId(int id) { shapeId = id; }
	
	bool isBodyObstacle() { return bodyObstacle; }
	void setBodyObstacle() { bodyObstacle = true; }
	
	const std::string& getTypeName() const { return type; }
	
	typedef FamilyFactory<PlannerObstacle,std::string> registry_t;
	static registry_t& getRegistry() { static registry_t registry; return registry; }
	
	PLIST_CLONE_FWD(PlannerObstacle);
	
protected:
	//! Copy constructor, be careful not to cross-copy different types
	PlannerObstacle(const PlannerObstacle& o) : 
		plist::Dictionary(), name(o.name), 
		type(o.type), geometry(o.geometry), shapeId(o.shapeId), bodyObstacle(false) { init(); }
	//! Don't use this assignment...
	PlannerObstacle& operator=(const PlannerObstacle& o) {
		name = o.name;
		type = o.type;
		geometry = o.geometry;
		init();
		return *this;
	}
	
	void init() {
		addEntry("Name",name,"Human readable name for debugging, display");
		addEntry(".type",type,"Allows polymorphic load/save, must match known class name");
		setLoadSavePolicy(FIXED,SYNC);
	}
	
	plist::Primitive<std::string> type;
	ObstacleGeometry geometry;
	int shapeId; //!< For obstacles defined by DualCoding shapes
	bool bodyObstacle; //!< For 3D path planning, to avoid arm-body collisions
};

template<size_t N>
std::string PlannerObstacle<N>::toString() const {
	std::ostringstream os;
	os << "PlannerObstacle[type=" << type << ",name=" << name << "]";
	return os.str();
}

//! Rectangular defined obstacle designated by 4 corners and its rotation matrix from axis alignment
class RectangularObstacle: public PlannerObstacle2D {
protected:
	friend class ConvexPolyObstacle;
	
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	//! Center of rectangle
	fmat::Column<2> center;
	
	//! minimum extents (along rectangle axes)
	fmat::Column<2> minEx;
	//! maximum extents (along rectangle axes)
	fmat::Column<2> maxEx;
	
	//! axis aligned bounding box
	BoundingBox2D bBox;
	
	//! The rotation matrix to rotate from 'normal' coordinates to the obstacle orientation
	fmat::Matrix<2,2> unrot;
	//! corner points for fast collision checking
	/*! points are stored in clockwise order from upper right:
	 *  - top right
	 *  - bottom right
	 *  - bottom left
	 *  - top left */
	fmat::Matrix<4,2> points;
	
public:
	//! Default constructor
	RectangularObstacle(): PlannerObstacle2D(RECTANGULAR_OBS,autoRegisterName),
												 center(), minEx(), maxEx(), bBox(), 
												 unrot(fmat::Matrix<2,2>::identity()), points() {}
	
	//! Pass the center, the extents from center (half-width and half-height), and an angular (radian) rotation
	RectangularObstacle(const fmat::SubVector<2,const fmat::fmatReal>& centerPoint,
						const fmat::SubVector<2,const fmat::fmatReal>& extents,
						fmat::fmatReal orient)
	: PlannerObstacle2D(RECTANGULAR_OBS,autoRegisterName), center(), minEx(), maxEx(), bBox(), unrot(), points()
	{
		reset(centerPoint,extents,orient); 
	}
	
	//! Pass a bounding box and a transform... the transformation is applied relative to origin, e.g. the @a bb center will rotate about the origin
	RectangularObstacle(const BoundingBox2D& bb, const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot, const fmat::SubVector<2,const fmat::fmatReal>& off)
	: PlannerObstacle2D(RECTANGULAR_OBS,autoRegisterName), center(), minEx(), maxEx(), bBox(), unrot(), points()
	{
		fmat::Column<2> c(bb.getCenter()), d(bb.getDimensions());
		reset(rot*c+off, d/2, rot);
	}
	
	//! Assignment, should not use plist::Dictionary version
	RectangularObstacle& operator=(const RectangularObstacle& o) {
		PlannerObstacle2D::operator=(o);
		center=o.center;
		minEx=o.minEx;
		maxEx=o.maxEx;
		bBox=o.bBox;
		unrot=o.unrot;
		points=o.points;
		return *this;
	}
	
	using PlannerObstacle2D::collides;
	
	//! Test if the rectangle includes a specific point
	virtual bool collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const;
	
	bool collides(const RectangularObstacle& other) const;
	bool collides(const CircularObstacle& other) const;
	bool collides(const EllipticalObstacle& other) const;
	
	virtual fmat::Column<2> getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const;
	
	//! do a complete reset of parameters
	virtual void reset(const fmat::Column<2>& centerPoint, const fmat::Column<2>& extents, fmat::fmatReal orient) {
		reset(centerPoint,extents,fmat::rotation2D(orient));
	}
	
	//! do a complete reset of parameters (so if you already have the rotation matrix, avoids regeneration from the angular value)
	virtual void reset(fmat::Column<2> centerPoint,
					   const fmat::SubVector<2,const fmat::fmatReal>& extents,
					   const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot);
	
	virtual void updatePosition(const fmat::SubVector<2, const fmat::fmatReal>& newPos);
	
	using PlannerObstacle2D::rotate;
	virtual void rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot);
	
	//! Minimum and maximum X and Y extents (axis aligned bounding box)
	virtual BoundingBox2D getBoundingBox() const { return bBox; }
	
	virtual fmat::Column<2> gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	//! Gets the center (X,Y)
	virtual fmat::Column<2> getCenter() const { return center; }
	
	//! returns the width along 'local' axes
	float getWidth() const { return maxEx[0] - minEx[0]; }
	//! returns the height along 'local' axes
	float getHeight() const { return maxEx[1] - minEx[1]; }
	//! returns the half-width and half-height, suitable for easy use with reset()
	fmat::Column<2> getExtents() const { return (maxEx-minEx)/2; }
	//! returns the orienation of the rectangle in radians
	float getOrientation() const;
	
	enum CornerOrder {
		TOP_RIGHT,
		TOP_LEFT,
		BOTTOM_LEFT,
		BOTTOM_RIGHT,
		NUM_CORNERS // for looping
	};
	
	//! returns the specified corner point
	fmat::Column<2> getCorner(size_t i) const { return fmat::pack(points(i,0),points(i,1)); }
	
	//! Increases the size of the obstacle in all directions by at least @a amount
	virtual void bloat(float amount);
	
	//! Decreases the size of the obstacle in all directions by at least @a amount
	virtual void contract(float amount);
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for RectangularObstacle
	PLIST_CLONE_DEF(RectangularObstacle,new RectangularObstacle(*this));
};

//! Circle/Sphere defined obstacle designated by a center point and a radius
class CircularObstacle : public PlannerObstacle2D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	//! Center of the obstacle
	fmat::Column<2> center;
	
	//! Radius of the obstacle
	fmat::fmatReal radius;
	
public:
	//! Default constructor
	CircularObstacle() : PlannerObstacle2D(CIRCULAR_OBS,autoRegisterName),
	center(), radius(0) {}
	
	//! point and radius
	CircularObstacle(fmat::fmatReal x, fmat::fmatReal y, fmat::fmatReal r) : PlannerObstacle2D(CIRCULAR_OBS,autoRegisterName),
	center(fmat::pack(x,y)), radius(r) {}
	
	//! point and radius
	CircularObstacle(const fmat::Column<2>& c, fmat::fmatReal r) : PlannerObstacle2D(CIRCULAR_OBS,autoRegisterName),
	center(c), radius(r) {}
	
	//! Assignment, should not use plist::Dictionary version
	CircularObstacle& operator=(const CircularObstacle& o) {
		PlannerObstacle2D::operator=(o);
		center=o.center;
		radius=o.radius;
		return *this;
	}
	
	using PlannerObstacle2D::collides;
	
	//! Test if the circle includes a specific point
	virtual bool collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const {
		return (point - center).sumSq() < radius*radius;
	}
	
	bool collides(const CircularObstacle& other) const;
	
	virtual fmat::Column<2> getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const;
	
	virtual fmat::Column<2> getCenter() const { return center; }
	virtual fmat::fmatReal getRadius() const { return radius; }
	virtual void setRadius(fmat::fmatReal r) { radius = r; }
	
	virtual void updatePosition(const fmat::SubVector<2, const fmat::fmatReal>& newPos) { center = newPos; }
	
	using PlannerObstacle2D::rotate;
	virtual void rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot) {
		updatePosition(rot * (getCenter()-origin) + origin);
	}
	
	virtual BoundingBox2D getBoundingBox() const {
		return BoundingBox2D(center - radius, center + radius);
	}
	
	virtual fmat::Column<2> gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	//! Increases the size of the obstacle in all directions by at least @a amount
	virtual void bloat(float amount) { radius += amount; }
	//! Decreases the size of the obstacle in all directions by at least @a amount
	virtual void contract(float amount) { radius += amount; }
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for CircularObstacle
	PLIST_CLONE_DEF(CircularObstacle,new CircularObstacle(*this));
};

//! Elliptically defined obstacle
class EllipticalObstacle : public PlannerObstacle2D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
public:
	//! Focus of the ellipse
	fmat::Column<2> focus1;
	
	//! Focus of the ellipse	
	fmat::Column<2> focus2;
	
	//! Center of the ellipse
	fmat::Column<2> center;
	
	//! Half-length of the major axis
	fmat::fmatReal semimajor;
	
	//! Half-length of the minor axis
	fmat::fmatReal semiminor;
	
	//! Default constructor
	EllipticalObstacle() : PlannerObstacle2D(ELLIPTICAL_OBS,autoRegisterName),
	focus1(), focus2(), center(), semimajor(), semiminor() {}
	
	//! Location, semimajor, semiminor, orientation
	/*! Swaps the major/minor and adds 90° to the orientation if major<minor */
	EllipticalObstacle(const fmat::Column<2>& c, fmat::fmatReal _semimajor, fmat::fmatReal _semiminor, 
			   fmat::fmatReal orientation) : PlannerObstacle2D(ELLIPTICAL_OBS,autoRegisterName),
	focus1(), focus2(), center(c), semimajor(_semimajor), semiminor(_semiminor)
	  { reset(c, _semimajor, _semiminor, orientation); }
	
	//! Circle with point and radius
	EllipticalObstacle(const fmat::Column<2>& c, fmat::fmatReal r) : PlannerObstacle2D(ELLIPTICAL_OBS,autoRegisterName),
	focus1(c), focus2(c), center(c), semimajor(r), semiminor(r) {}
	
	//! Assignment, should not use plist::Dictionary version
	EllipticalObstacle& operator=(const EllipticalObstacle& o) {
		PlannerObstacle2D::operator=(o);
		focus1=o.focus1;
		focus2=o.focus2;
		center=o.center;
		semimajor=o.semimajor;
		semiminor=o.semiminor;
		return *this;
	}
	
	//! Initialize from two foci and semimajor @a s
	/*! Will throw an exception if @a s is too short (must be at least half the distance between the foci) */
	void reset(const fmat::Column<2>& f1, const fmat::Column<2>& f2, fmat::fmatReal s);
	
	//! Initialize from center, extents, and orientation
	/*! Swaps the major/minor and adds 90° to the orientation if major<minor */
	void reset(const fmat::Column<2>& c, fmat::fmatReal smajor, fmat::fmatReal sminor, fmat::fmatReal ori);
	
	//! Initialize from circle: center and radius
	void reset(const fmat::Column<2>& c, fmat::fmatReal radius) {
		focus1 = focus2 = center = c;
		semimajor = semiminor = std::abs(radius);
	}
	
	using PlannerObstacle2D::collides;
	
	//! Test if the circle includes a specific point
	virtual bool collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const {
		return (point - focus1).norm() + (point - focus2).norm() < 2*semimajor;
	}
	
	virtual fmat::Column<2> getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const;
	
	virtual fmat::Column<2> getCenter() const { return center; }
	
	virtual void updatePosition(const fmat::SubVector<2, const fmat::fmatReal>& newPos);
	
	using PlannerObstacle2D::rotate;
	virtual void rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot);
	
	//! Returns the angle of the major axis
	float getAngle() const { return std::atan2(focus1[1] - focus2[1], focus1[0] - focus2[0]); }
	
	//! If rotating into the ellipse's frame, construct rotation matrix directly
	fmat::Matrix<2,2> getOrientation() const;
	
	//! Returns the point on the edge of the ellipse at the specified angle
	/*! @a theta should be relative to the reference frame of the space that the ellipse resides in
	 *  (and returns results in the external frame as well) */
	fmat::Column<2> getPointOnEdge(fmat::fmatReal theta) const {
		return getPointOnEdge(fmat::pack(std::cos(theta),std::sin(theta)));
	}
	
	//! Returns the point on the edge of the ellipse which intersects vector @a v through the center.
	/*! @a v does not need to be normalized */
	fmat::Column<2> getPointOnEdge(const fmat::Column<2>& direction) const;
	
	virtual BoundingBox2D getBoundingBox() const;
	
	//! Approximates the closest point on the ellipse to @a pt
	/*! Basically scales the vector from the center to @a pt */
	virtual fmat::Column<2> gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	//! Increases the size of the obstacle in all directions by at least @a amount
	virtual void bloat(float amount) { reset(center, semimajor+amount, semiminor+amount, getAngle()); }
	//! Decreases the size of the obstacle in all directions by at least @a amount
	virtual void contract(float amount) { reset(center, semimajor-amount, semiminor-amount, getAngle()); }
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode* node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for EllipticalObstacle
	PLIST_CLONE_DEF(EllipticalObstacle,new EllipticalObstacle(*this));
};

//! Manages collision detection with convex polygons
/*! Basically the same algorithm used for rectangles, just generalized for more sides */
class ConvexPolyObstacle : public PlannerObstacle2D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	std::vector<fmat::Column<2> > points; //! the points of the polygon, in counter-clockwise order
	std::vector<fmat::Column<2> > normals; //! the 'outside' normal of segment following corresponding entry in #points
	
public:
	//! default constructor
	ConvexPolyObstacle() : PlannerObstacle2D(CONVEX_POLY_OBS,autoRegisterName), points(), normals() {}
	
	//! Assignment, should not use plist::Dictionary version
	ConvexPolyObstacle& operator=(const ConvexPolyObstacle& o) {
		PlannerObstacle2D::operator=(o);
		points = o.points;
		normals = o.normals;
		return *this;
	}
	
	//! returns the points, in counter-clockwise order
	const std::vector<fmat::Column<2> >& getPoints() const { return points; }
	
	//! returns the normals, in counter-clockwise order
	const std::vector<fmat::Column<2> >& getNormals() const { return normals; }
	
	//! Selects points which form a convex hull, clears current points
	/*! Passing a set because the hulling algorithm (Andrew's Monotone Chain)
	 *  begins with sorting the list of points anyway.  If you have a different STL container 'c'
	 *  just pass std::set<fmat::Column<2>(c.begin(),c.end()) to convert to a set.
	 *  This implementation adapted from:
	 *  http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain */
	void hull(const std::set<fmat::Column<2> >& p);
	
	//! remove all points and normals
	void clear() { points.clear(); normals.clear(); }
	
	//! Add a point to the polygon, must maintain convexity and in counter-clockwise order, call close() when done
	void addPoint(const fmat::Column<2>& p);
	
	//! Calculates values for final segment of polygon, call after a series of addPoint()
	void close();
	
	using PlannerObstacle2D::collides;
	//! Test if the polygon includes a specific point
	virtual bool collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const;
	//! For polygon on rectangle collision
	bool collides(const RectangularObstacle& other) const;
	//! For polygon on circle collision
	bool collides(const CircularObstacle& other) const;
	//! For polygon on polygon collision
	bool collides(const ConvexPolyObstacle& other) const;
	
	virtual fmat::Column<2> getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const;
	
	virtual fmat::Column<2> getCenter() const;
	virtual void updatePosition(const fmat::SubVector<2, const fmat::fmatReal>& newPos) { offset(newPos-getCenter()); }
	
	using PlannerObstacle2D::rotate;
	virtual void rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot);
	virtual BoundingBox2D getBoundingBox() const;
	
	virtual void offset(const fmat::Column<2>& off);
	virtual fmat::Column<2> gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for ConvexPolyObstacle
	PLIST_CLONE_DEF(ConvexPolyObstacle,new ConvexPolyObstacle(*this));
};

//! Hierarchically defined obstacle containing multiple obstacles
class HierarchicalObstacle : public PlannerObstacle2D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	//! recalculate rectangular bounding box based on contained obstacles
	void recalculateBoundingBox();
	
	//! grow the bounding box to include that of @a other
	void expandBoundingBox(PlannerObstacle2D& other);
	
	std::vector<PlannerObstacle2D*> components; //!< the obstacles contained in the hierarchy
	
	BoundingBox2D aabb; //!< axis-aligned bounding box of all obstacles
	
	fmat::Column<2> center; //!< origin of the obstacle, all sub-obstacles will be relative to this point
	fmat::Matrix<2,2> rotation; //!< rotation about the origin
	
public:
	//! default constructor
	HierarchicalObstacle()
	: PlannerObstacle2D(HIERARCHICAL_OBS,autoRegisterName), components(), aabb(), center(fmat::ZERO2), rotation(fmat::Matrix<2,2>::identity()) {}
	
 	//! copy constructor (deep copy #components)
 	HierarchicalObstacle(const HierarchicalObstacle& ho)
	: PlannerObstacle2D(ho), components(), aabb(ho.aabb), center(ho.center), rotation(ho.rotation)
 	{
 		components.reserve(ho.components.size());
 		for(std::vector<PlannerObstacle2D*>::const_iterator it=ho.components.begin(); it!=ho.components.end(); ++it) {
 			components.push_back(dynamic_cast<PlannerObstacle2D*>((*it)->clone())); // need explicit cast for old compilers, should be no-op
 		}
 	}
 	
 	//! Assignment, should not use plist::Dictionary version
 	HierarchicalObstacle& operator=(const HierarchicalObstacle& o) {
		PlannerObstacle2D::operator=(o);
 		clear();
 		components.reserve(o.components.size());
 		for(std::vector<PlannerObstacle2D*>::const_iterator it=o.components.begin(); it!=o.components.end(); ++it) {
 			components.push_back(dynamic_cast<PlannerObstacle2D*>((*it)->clone())); // need explicit cast for old compilers, should be no-op
 		}
 		aabb=o.aabb;
 		center=o.center;
 		rotation=o.rotation;
 		return *this;
 	}
 	
	//! destructor
	virtual ~HierarchicalObstacle() { clear(); }
	
	//! returns components of specified type
	template <class T>
	void getComponents(std::vector<T> &obs) const;
	
	const std::vector<PlannerObstacle2D*>& getComponents() const { return components; }
	
	//! For obstacle on point
	virtual bool collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const;
	
	using PlannerObstacle2D::collides;
	virtual bool collides(const PlannerObstacle2D& other) const;
	
	//! Never called: since HO's aren't convex, must test collision with each obstacle
	virtual fmat::Column<2> getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const { return fmat::Column<2>(); }
	
	virtual BoundingBox2D getBoundingBox() const { return aabb; }
	
	virtual fmat::Column<2> getCenter() const { return center; }
	virtual fmat::Matrix<2,2> getOrientation() const { return rotation; }
	
	virtual void updatePosition(const fmat::SubVector<2, const fmat::fmatReal>& newPos);
	
	//! update to specified orientation about the center
	virtual void updateRotation(const fmat::Matrix<2,2>& rot) { rotation = rot; }
	
	using PlannerObstacle2D::rotate;
	virtual void rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot);
	
	virtual fmat::Column<2> gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	//! Print the string representation for all components contained in the Hierarchy.
	virtual std::string componentsToString() const;
	
	//! clear all components
	void clear() {
		for (unsigned int i = 0; i < components.size(); i++)
			delete components[i];
		components.clear(); recalculateBoundingBox();
	}
	
	//! add component to end of component list; components should be transformed in terms of Hierarchical Obstacle
	/*! Memory allocation of @a o will be claimed by the hierarchical obstacle. */
	void add(PlannerObstacle2D* o) { components.push_back(o); expandBoundingBox(*o); }
	
	//! clone definition for HierarchicalObstacle
	PLIST_CLONE_DEF(HierarchicalObstacle,new HierarchicalObstacle(*this));
};

template <class T>
void HierarchicalObstacle::getComponents(std::vector<T> &comps) const {
	for (unsigned int i = 0; i < components.size(); i++) {
		T t = dynamic_cast<T>(components[i]);
		if (t)
			comps.push_back(t);
	}
}

class BoxObstacle : public PlannerObstacle3D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	//! Center of box
	fmat::Column<3> center;
	
	//! minimum extents (along box axes)
	fmat::Column<3> minEx;
	//! maximum extents (along box axes)
	fmat::Column<3> maxEx;
	
	//! axis aligned bounding box
	BoundingBox3D bBox;
	
	//! The rotation matrix to rotate from 'normal' coordinates to the obstacle orientation
	fmat::Matrix<3,3> unrot;
	
	//! corner points for fast collision checking
	/*! points are stored first top then bottom,
	 both in clockwise order from upper right:
	 *  - top upper right
	 *  - top lower right
	 *  - top lower left
	 *  - top upper left
	 *  - bottom upper right
	 *  - bottom lower right
	 *  - bottom lower left
	 *  - bottom upper left */
	fmat::Matrix<8,3> points;
	
public:
	//! Default constructor
	BoxObstacle() : PlannerObstacle3D(BOX_OBS,autoRegisterName),
									center(), minEx(), maxEx(), bBox(), unrot(fmat::Matrix<3,3>::identity()), points() {}
	
	//! Pass the center, the extents from center (half-length, half-width and half-height), and a rotation
	BoxObstacle(const fmat::SubVector<3,const fmat::fmatReal>& centerPoint,
							const fmat::SubVector<3,const fmat::fmatReal>& extents,
							const fmat::SubMatrix<3,3,const fmat::fmatReal>& orient)
		: PlannerObstacle3D(BOX_OBS,autoRegisterName), center(), minEx(), maxEx(), bBox(), unrot(), points()
	{
		reset(centerPoint,extents,orient); 
	}
	
	//! Pass a bounding box and a transform... the transformation is applied relative to origin, e.g. the @a bb center will rotate about the origin
	BoxObstacle(const BoundingBox3D& bb,
							const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot,
							const fmat::SubVector<3,const fmat::fmatReal>& off)
		: PlannerObstacle3D(BOX_OBS,autoRegisterName), center(), minEx(), maxEx(), bBox(), unrot(), points()
	{
		fmat::Column<3> c(bb.getCenter()), d(bb.getDimensions());
		reset(rot*c+off, d/2, rot);
	}
	
	//! Assignment, should not use plist::Dictionary version
	BoxObstacle& operator=(const BoxObstacle& o) {
		PlannerObstacle3D::operator=(o);
		center=o.center;
		minEx=o.minEx;
		maxEx=o.maxEx;
		bBox=o.bBox;
		unrot=o.unrot;
		points=o.points;
		return *this;
	}
	
	//! do a complete reset of parameters
	virtual void reset(fmat::Column<3> centerPoint,
					   const fmat::SubVector<3,const fmat::fmatReal>& extents,
					   const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot);
	
	virtual void updatePosition(const fmat::SubVector<3, const fmat::fmatReal>& newPos);
	
	using PlannerObstacle3D::rotate;
	virtual void rotate(const fmat::SubVector<3,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot);
	
	virtual std::string toString() const;
	virtual fmat::Column<3> getCenter() const { return center; }
	virtual BoundingBox3D getBoundingBox() const { return bBox; }
	virtual fmat::Matrix<3,3> getOrientation() const { return unrot.transpose(); }
	
	using PlannerObstacle3D::collides;
	virtual bool collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const;
	
	bool collides(const BoxObstacle& other) const;
	
	virtual fmat::Column<3> getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const;
	
	virtual fmat::Column<3> gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const;
	
	//! returns the length along 'local' axes
	float getLength() const { return maxEx[0] - minEx[0]; }
	//! returns the width along 'local' axes
	float getWidth() const { return maxEx[1] - minEx[1]; }
	//! returns the height along 'local' axes
	float getHeight() const { return maxEx[2] - minEx[2]; }
	//! returns the half-width and half-height, suitable for easy use with reset()
	fmat::Column<3> getExtents() const { return (maxEx-minEx)/2; }
	
	enum CornerOrder {
		TOP_UPPER_RIGHT,
		TOP_LOWER_RIGHT,
		TOP_LOWER_LEFT,
		TOP_UPPER_LEFT,
		BOTTOM_UPPER_RIGHT,
		BOTTOM_LOWER_RIGHT,
		BOTTOM_LOWER_LEFT,
		BOTTOM_UPPER_LEFT,
		NUM_CORNERS // for looping
	};
	
	//! returns the specified corner point
	fmat::Column<3> getCorner(size_t i) const { return fmat::pack(points(i,0),points(i,1),points(i,2)); }
	
	//! Increases the size of the obstacle in all directions by at least @a amount
	virtual void bloat(float amount);
	
	//! Decreases the size of the obstacle in all directions by at least @a amount
	virtual void contract(float amount) { bloat(-amount); }
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode * node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for BoxObstacle
	PLIST_CLONE_DEF(BoxObstacle,new BoxObstacle(*this));
};

//! Cylinder with central axis in alignment with z-axis by default.
class CylindricalObstacle : public PlannerObstacle3D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	//! center of the cylinder
	fmat::Column<3> center;
	
	//! 3D orientation of cylinder
	fmat::Matrix<3,3> orientation;
	
	//! radius of cylinder
	fmat::fmatReal radius;
	
	//! half-height
	fmat::fmatReal halfHeight;
	
public:
	//! Default constructor
	CylindricalObstacle() : PlannerObstacle3D(CYLINDRICAL_OBS,autoRegisterName),
	center(), orientation(), radius(0), halfHeight(0) {}
	
	CylindricalObstacle(const fmat::SubVector<3,const fmat::fmatReal>& c,
						const fmat::SubMatrix<3,3,const fmat::fmatReal>& o,
						fmat::fmatReal r,
						fmat::fmatReal hh) :
	PlannerObstacle3D(CYLINDRICAL_OBS,autoRegisterName), center(c), orientation(o), radius(r), halfHeight(hh) {}
	
	//! Assignment, should not use plist::Dictionary version
	CylindricalObstacle& operator=(const CylindricalObstacle& o) {
		PlannerObstacle3D::operator=(o);
		center=o.center;
		orientation=o.orientation;
		radius=o.radius;
		halfHeight=o.halfHeight;
		return *this;
	}
	
	virtual void updatePosition(const fmat::SubVector<3, const fmat::fmatReal>& newPos) { center = newPos; }
	virtual void rotate(const fmat::SubVector<3,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot) {
		center = rot * (center - origin) + origin;
		orientation = rot * orientation;
	}
	virtual fmat::Column<3> getCenter() const { return center; }
	virtual float getRadius() const { return radius; }
	virtual BoundingBox3D getBoundingBox() const;
	
	using PlannerObstacle3D::collides;
	virtual bool collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const;
	
	virtual fmat::Column<3> getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const;
	
	virtual fmat::Column<3> gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode* node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for CylindricalObstacle
	PLIST_CLONE_DEF(CylindricalObstacle,new CylindricalObstacle(*this));
};

//! Spherically defined obstacle
class SphericalObstacle : public PlannerObstacle3D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	//! center of the sphere
	fmat::Column<3> center;
	
	//! radius of the sphere
	fmat::fmatReal radius;
	
public:
	//! Default constructor
	SphericalObstacle() : PlannerObstacle3D(SPHERICAL_OBS,autoRegisterName),
	center(), radius(0) {}
	
	//! point and radius
	SphericalObstacle(fmat::fmatReal x, fmat::fmatReal y, fmat::fmatReal z, fmat::fmatReal r) : PlannerObstacle3D(CIRCULAR_OBS,autoRegisterName),
	center(fmat::pack(x,y,z)), radius(r) {}
	
	//! point and radius
	SphericalObstacle(const fmat::Column<3>& c, fmat::fmatReal r) : PlannerObstacle3D(CIRCULAR_OBS,autoRegisterName),
	center(c), radius(r) {}
	
	//! Assignment, should not use plist::Dictionary version
	SphericalObstacle& operator=(const SphericalObstacle& o) {
		PlannerObstacle3D::operator=(o);
		center=o.center;
		radius=o.radius;
		return *this;
	}
	
	virtual void updatePosition(const fmat::SubVector<3, const fmat::fmatReal>& newPos) { center = newPos; }
	
	virtual void rotate(const fmat::SubVector<3,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot) { center = rot * (center - origin) + origin; }
	
	virtual fmat::Column<3> getCenter() const { return center; }
	virtual fmat::fmatReal getRadius() const { return radius; }
	virtual BoundingBox3D getBoundingBox() const {
		return BoundingBox3D(center - radius, center + radius);
	}
	
	using PlannerObstacle3D::collides;
	//! For obstacle on point
	virtual bool collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const;
	
	bool collides(const SphericalObstacle& other) const;
	
	virtual fmat::Column<3> getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const;
	
	virtual fmat::Column<3> gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode* node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for SphericalObstacle
	PLIST_CLONE_DEF(SphericalObstacle,new SphericalObstacle(*this));
};

class EllipsoidObstacle : public PlannerObstacle3D {
protected:
	//! stores the class name used for polymorphic load/save
	static const std::string autoRegisterName;
	
	//! center of the ellipsoid
	fmat::Column<3> center;
	
	//! 3D orientation of the ellipse
	fmat::Matrix<3,3> orientation;
	
	//! positive half-width extents along each axis
	fmat::Column<3> extents;
	
public:
	//! Default constructor
	EllipsoidObstacle() :
	PlannerObstacle3D(ELLIPSOID_OBS,autoRegisterName), center(), orientation(), extents() {}
	
	EllipsoidObstacle(const fmat::SubVector<3, const fmat::fmatReal>& c,
					  const fmat::SubMatrix<3,3, const fmat::fmatReal>& o,
					  const fmat::SubVector<3, const fmat::fmatReal>& e) :
	PlannerObstacle3D(ELLIPSOID_OBS,autoRegisterName), center(c), orientation(o), extents(e) {}
	
	//! Assignment, should not use plist::Dictionary version
	EllipsoidObstacle& operator=(const EllipsoidObstacle& o) {
		PlannerObstacle3D::operator=(o);
		center=o.center;
		orientation=o.orientation;
		extents=o.extents;
		return *this;
	}
	
	virtual void updatePosition(const fmat::SubVector<3, const fmat::fmatReal>& newPos) { center = newPos; }
	virtual void rotate(const fmat::SubVector<3,const fmat::fmatReal>& origin,
						const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot) {
		center = rot * (center - origin) + origin;
		orientation = rot * orientation;
	}
	virtual fmat::Column<3> getCenter() const { return center; }
	virtual BoundingBox3D getBoundingBox() const;
	
	using PlannerObstacle3D::collides;
	virtual bool collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const;
	
	virtual fmat::Column<3> getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const;
	
	//! helper function, in the case of any of the components being zero.
	virtual fmat::Column<2> get2Support(const fmat::SubVector<2,const fmat::fmatReal>& direction) const;
	
	virtual fmat::Column<3> gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const;
	
	virtual std::string toString() const;
	
	virtual void loadXML(xmlNode* node);
	virtual void saveXML(xmlNode* node) const;
	using plist::DictionaryBase::saveXML;
	
	//! clone definition for EllipsoidObstacle
	PLIST_CLONE_DEF(EllipsoidObstacle,new EllipsoidObstacle(*this));
};

#endif
