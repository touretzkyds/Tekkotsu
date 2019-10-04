#include "PlannerObstacles.h"
#include "GJK.h"
#include "Shared/debuget.h"
#include "Shared/plistSpecialty.h"
#include "Shared/mathutils.h" // for isnan fix

const std::string RectangularObstacle::autoRegisterName = PlannerObstacle2D::getRegistry().registerType<RectangularObstacle>("Rectangle");
const std::string CircularObstacle::autoRegisterName = PlannerObstacle2D::getRegistry().registerType<CircularObstacle>("Circle");
const std::string EllipticalObstacle::autoRegisterName = PlannerObstacle2D::getRegistry().registerType<EllipticalObstacle>("Ellipse");
const std::string ConvexPolyObstacle::autoRegisterName = PlannerObstacle2D::getRegistry().registerType<ConvexPolyObstacle>("ConvexPoly");
const std::string HierarchicalObstacle::autoRegisterName = PlannerObstacle2D::getRegistry().registerType<HierarchicalObstacle>("Hierarchy");
const std::string BoxObstacle::autoRegisterName = PlannerObstacle3D::getRegistry().registerType<BoxObstacle>("Box");
const std::string CylindricalObstacle::autoRegisterName = PlannerObstacle3D::getRegistry().registerType<CylindricalObstacle>("Cylinder");
const std::string SphericalObstacle::autoRegisterName = PlannerObstacle3D::getRegistry().registerType<SphericalObstacle>("Sphere");
const std::string EllipsoidObstacle::autoRegisterName = PlannerObstacle3D::getRegistry().registerType<EllipsoidObstacle>("Ellipsoid");

PLIST_CLONE_IMP(RectangularObstacle,new RectangularObstacle(*this));
PLIST_CLONE_IMP(CircularObstacle,new CircularObstacle(*this));
PLIST_CLONE_IMP(EllipticalObstacle,new EllipticalObstacle(*this));
PLIST_CLONE_IMP(ConvexPolyObstacle,new ConvexPolyObstacle(*this));
PLIST_CLONE_IMP(HierarchicalObstacle,new HierarchicalObstacle(*this));
PLIST_CLONE_IMP(BoxObstacle,new BoxObstacle(*this));
PLIST_CLONE_IMP(CylindricalObstacle,new CylindricalObstacle(*this));
PLIST_CLONE_IMP(SphericalObstacle,new SphericalObstacle(*this));
PLIST_CLONE_IMP(EllipsoidObstacle,new EllipsoidObstacle(*this));

namespace plist {
	template<> PlannerObstacle2D* loadXML(xmlNode* node) {
		plist::Primitive<std::string> type;
		Dictionary td(false);
		td.setUnusedWarning(false);
		td.addEntry(".type",type);
		td.loadXML(node);
		PlannerObstacle2D* po = PlannerObstacle2D::getRegistry().create(type);
		if(po==NULL)
			throw XMLLoadSave::bad_format(node,"Unknown PlannerObstacle2D type "+type);
		try {
			po->loadXML(node);
		} catch(...) {
			delete po;
			throw;
		}
		return po;
	}
	template<> PlannerObstacle3D* loadXML(xmlNode* node) {
		plist::Primitive<std::string> type;
		Dictionary td(false);
		td.setUnusedWarning(false);
		td.addEntry(".type",type);
		td.loadXML(node);
		PlannerObstacle3D* po = PlannerObstacle3D::getRegistry().create(type);
		if(po==NULL)
			throw XMLLoadSave::bad_format(node,"Unknown PlannerObstacle3D type "+type);
		try {
			po->loadXML(node);
		} catch(...) {
			delete po;
			throw;
		}
		return po;
	}
}

// Collision dispatching
template <>
bool PlannerObstacle2D::collides(const PlannerObstacle2D& other) const {
	/* Although we have the generalized GJK algorithm for collision detection,
	 we use analytic solutions that are fully tested and are faster than GJK. */
	switch(geometry * geometry * other.geometry) {
		// handled by rect: rect-rect, rect-circ
		case RECTANGULAR_OBS * RECTANGULAR_OBS * RECTANGULAR_OBS:
			return static_cast<const RectangularObstacle*>(this)->collides(static_cast<const RectangularObstacle&>(other));
		case RECTANGULAR_OBS * RECTANGULAR_OBS * CIRCULAR_OBS:
			return static_cast<const RectangularObstacle*>(this)->collides(static_cast<const CircularObstacle&>(other));
		case RECTANGULAR_OBS * CIRCULAR_OBS * CIRCULAR_OBS:
			return static_cast<const RectangularObstacle*>(&other)->collides(static_cast<const CircularObstacle&>(*this));
			
		// handled by circ: circ-circ
		case CIRCULAR_OBS * CIRCULAR_OBS * CIRCULAR_OBS:
			return static_cast<const CircularObstacle*>(this)->collides(static_cast<const CircularObstacle&>(other));
			
		// handled by convex: convex-rect, convex-circ, convex-convex
		case CONVEX_POLY_OBS * CONVEX_POLY_OBS * RECTANGULAR_OBS:
			return static_cast<const ConvexPolyObstacle*>(this)->collides(static_cast<const RectangularObstacle&>(other));
		case CONVEX_POLY_OBS * RECTANGULAR_OBS * RECTANGULAR_OBS:
			return static_cast<const ConvexPolyObstacle*>(&other)->collides(static_cast<const RectangularObstacle&>(*this));
		case CONVEX_POLY_OBS * CONVEX_POLY_OBS * CIRCULAR_OBS:
			return static_cast<const ConvexPolyObstacle*>(this)->collides(static_cast<const CircularObstacle&>(other));
		case CONVEX_POLY_OBS * CIRCULAR_OBS * CIRCULAR_OBS:
			return static_cast<const ConvexPolyObstacle*>(&other)->collides(static_cast<const CircularObstacle&>(*this));
		case CONVEX_POLY_OBS * CONVEX_POLY_OBS * CONVEX_POLY_OBS:
			return static_cast<const ConvexPolyObstacle*>(this)->collides(static_cast<const ConvexPolyObstacle&>(other));
			
		// handled by hierarchical: hierarchical-rect, hierarchical-circ, hierarchical-ellipse, hierarchical-convex
		case HIERARCHICAL_OBS * HIERARCHICAL_OBS * RECTANGULAR_OBS:
			return static_cast<const HierarchicalObstacle*>(this)->collides(static_cast<const RectangularObstacle&>(other));
		case HIERARCHICAL_OBS * RECTANGULAR_OBS * RECTANGULAR_OBS:
			return static_cast<const HierarchicalObstacle*>(&other)->collides(static_cast<const RectangularObstacle&>(*this));
		case HIERARCHICAL_OBS * HIERARCHICAL_OBS * CIRCULAR_OBS:
			return static_cast<const HierarchicalObstacle*>(this)->collides(static_cast<const CircularObstacle&>(other));
		case HIERARCHICAL_OBS * CIRCULAR_OBS * CIRCULAR_OBS:
			return static_cast<const HierarchicalObstacle*>(&other)->collides(static_cast<const CircularObstacle&>(*this));
		case HIERARCHICAL_OBS * HIERARCHICAL_OBS * ELLIPTICAL_OBS:
			return static_cast<const HierarchicalObstacle*>(this)->collides(static_cast<const EllipticalObstacle&>(other));
		case HIERARCHICAL_OBS * ELLIPTICAL_OBS * ELLIPTICAL_OBS:
			return static_cast<const HierarchicalObstacle*>(&other)->collides(static_cast<const EllipticalObstacle&>(*this));
		case HIERARCHICAL_OBS * HIERARCHICAL_OBS * CONVEX_POLY_OBS:
			return static_cast<const HierarchicalObstacle*>(this)->collides(static_cast<const ConvexPolyObstacle&>(other));
		case HIERARCHICAL_OBS * CONVEX_POLY_OBS * CONVEX_POLY_OBS:
			return static_cast<const HierarchicalObstacle*>(&other)->collides(static_cast<const ConvexPolyObstacle&>(*this));
		case HIERARCHICAL_OBS * HIERARCHICAL_OBS * HIERARCHICAL_OBS:
			return static_cast<const HierarchicalObstacle*>(this)->collides(static_cast<const HierarchicalObstacle&>(other));
	}
	
	/* For the shapes that don't have efficient or accurate analytic solutions
	 we default to GJK.*/
	return GJK::collides(this, &other);
}

template <>
bool PlannerObstacle3D::collides(const PlannerObstacle3D& other) const {
	/* Although we have the generalized GJK algorithm for collision detection,
	 we use analytic solutions that are fully tested and are faster than GJK. */
	switch(geometry * geometry * other.geometry) {
		// handled by box: box-box
		case BOX_OBS * BOX_OBS * BOX_OBS:
			return static_cast<const BoxObstacle*>(this)->collides(static_cast<const BoxObstacle&>(other));
			
		// handled by sphere: sphere-sphere
		case SPHERICAL_OBS * SPHERICAL_OBS * SPHERICAL_OBS:
			return static_cast<const SphericalObstacle*>(this)->collides(static_cast<const SphericalObstacle&>(other));
	}
	
	/* For the shapes that don't have efficient or accurate analytic solutions
	 we default to GJK.*/
	return GJK::collides(this, &other);
}

void RectangularObstacle::reset(fmat::Column<2> centerPoint,
								const fmat::SubVector<2,const fmat::fmatReal>& extents,
								const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot) {
	center = centerPoint;
	
	fmat::Column<2> off = rot * extents;
	points.row(TOP_RIGHT) = &(center + off)[0]; // 0 top right
	points.row(BOTTOM_LEFT) = &(center - off)[0]; // 2 bottom left
	off = rot * fmat::pack(-extents[0],extents[1]);
	points.row(TOP_LEFT) = &(center + off)[0]; // 1 top left
	points.row(BOTTOM_RIGHT) = &(center - off)[0]; // 3 bottom right
	
	bBox.min = &points.minR()[0];
	bBox.max = &points.maxR()[0];
	
	unrot = rot.transpose(); // now switch to inverse
	centerPoint = unrot * centerPoint;
	maxEx = centerPoint + extents;
	minEx = centerPoint - extents;
}

void RectangularObstacle::updatePosition(const fmat::SubVector<2,const fmat::fmatReal>& newPos) {
	fmat::Column<2> diff = newPos - center;
	center = newPos;
	bBox.min += diff;
	bBox.max += diff;
	points.column(0)+=diff[0];
	points.column(1)+=diff[1];
	
	diff = unrot * diff;
	minEx += diff;
	maxEx += diff;
}

bool RectangularObstacle::collides(const RectangularObstacle& other) const {
	if( !bBox.collides(other.bBox) ) // test cheap aabb first
		return false;
	if(unrot(0,0)==1 && other.unrot(0,0)==1)
		return true; // rectangle is axis aligned, bb hit is all we needed
	
	// first test of other against current axes
	fmat::Matrix<4,2> p2 = other.points * unrot.transpose();
	float minX,maxX,minY,maxY;
	minX=p2.column(0).min();
	maxX=p2.column(0).max();
	minY=p2.column(1).min();
	maxY=p2.column(1).max();
	if(maxX <= minEx[0] || maxEx[0] <= minX
	   || maxY <= minEx[1] || maxEx[1] <= minY)
		return false;
	
	if(unrot(0,0)!=other.unrot(0,0)) {
		// test current points against other's axes
		p2 = points * other.unrot.transpose();
		minX=p2.column(0).min();
		maxX=p2.column(0).max();
		minY=p2.column(1).min();
		maxY=p2.column(1).max();
		if(maxX <= other.minEx[0] || other.maxEx[0] <= minX
		   || maxY <= other.minEx[1] || other.maxEx[1] <= minY)
			return false;
	}
	
	return true;
}

bool RectangularObstacle::collides(const CircularObstacle& other) const {	
	const fmat::Column<2> p = unrot * other.getCenter();
	const fmat::Column<2> pmin = p-other.getRadius(), pmax = p+other.getRadius();
	
	if(pmax[0]<=minEx[0] || maxEx[0]<=pmin[0] || pmax[1]<=minEx[1] || maxEx[1]<=pmin[1]) {
		return false;
	} else if(p[0]<minEx[0]) { // left margin
		if(p[1]<minEx[1]) { // bottom left corner
			return ((minEx - p).sumSq() < other.getRadius()*other.getRadius());
		} else if(maxEx[1]<p[1]) { // top left corner
			return ((fmat::pack(minEx[0],maxEx[1]) - p).sumSq() < other.getRadius()*other.getRadius());
		}
		return true; // center left
	} else if(maxEx[0] < p[0]) { // right margin
		if(p[1]<minEx[1]) { // bottom right corner
			return ((fmat::pack(maxEx[0],minEx[1]) - p).sumSq() < other.getRadius()*other.getRadius());
		} else if(maxEx[1]<p[1]) { // top right corner
			return ((maxEx - p).sumSq() < other.getRadius()*other.getRadius());
		}
		return true; // center right
	} else {
		return true; // center top or bottom margin
	}
}

void RectangularObstacle::rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
								 const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot) {
	fmat::Column<2> newCenter = rot * (center - origin) + origin;
	reset(newCenter, getExtents(), unrot.transpose() * rot);
}

fmat::Column<2> RectangularObstacle::gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const {
	const fmat::Column<2> p = unrot * pt;
	
	if(p[0]<minEx[0]) { // left margin
		if(p[1]<minEx[1]) { // bottom left corner
			return fmat::Column<2>(points.row(BOTTOM_LEFT).transpose()) - pt;
		} else if(maxEx[1]<p[1]) { // top left corner
			return fmat::Column<2>(points.row(TOP_LEFT).transpose()) - pt;
		}
		return unrot.row(0).transpose() * (minEx[0]-p[0]); // center left
	} else if(maxEx[0] < p[0]) { // right margin
		if(p[1]<minEx[1]) { // bottom right corner
			return fmat::Column<2>(points.row(BOTTOM_RIGHT).transpose()) - pt;
		} else if(maxEx[1]<p[1]) { // top right corner
			return fmat::Column<2>(points.row(TOP_RIGHT).transpose()) - pt;
		}
		return unrot.row(0).transpose() * (maxEx[0]-p[0]); // center right
	} else if(p[1]<minEx[1]) {
		return unrot.row(1).transpose() * (minEx[1]-p[1]); // bottom margin
	} else if(maxEx[1]<p[1]) {
		return unrot.row(1).transpose() * (maxEx[1]-p[1]); // top margin
	} else {
		// inside
		fmat::fmatReal dist[4];
		dist[0] = p[0]-minEx[0]; // left
		dist[1] = p[1]-minEx[1]; // bottom
		dist[2] = maxEx[0]-p[0]; // right
		dist[3] = maxEx[1]-p[1]; // top
		if(dist[0] < dist[1]) {
			if(dist[0] < dist[2]) {
				if(dist[0] < dist[3]) {
					return unrot.row(0).transpose() * (minEx[0]-p[0]); // left
				}
			} else {
				if(dist[2] < dist[3]) {
					return unrot.row(0).transpose() * (maxEx[0]-p[0]); // right
				}
			}
		} else {
			if(dist[1] < dist[2]) {
				if(dist[1] < dist[3]) {
					return unrot.row(1).transpose() * (minEx[1]-p[1]); // bottom
				}
			} else {
				if(dist[2] < dist[3]) {
					return unrot.row(0).transpose() * (maxEx[0]-p[0]); // right
				}
			}
		}
		return unrot.row(1).transpose() * (maxEx[1]-p[1]); // top
	}
}

std::string RectangularObstacle::toString() const {
	std::ostringstream os;
	os << "RectangularObstacle[" << name << ",points=" << points.fmt() << ",unrot=" << unrot.fmt() << "]";
	return os.str();
}

bool RectangularObstacle::collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const {
	const fmat::Column<2> p2 = unrot * point;
	return (minEx[0]<p2[0] && p2[0]<maxEx[0] &&
			minEx[1]<p2[1] && p2[1]<maxEx[1]);
}

fmat::Column<2> RectangularObstacle::getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const {
	const fmat::Column<2> dir = unrot * direction;
	// choose the closest corner
	if (dir[0]>0) {
		if (dir[1]>0)
			return points.row(TOP_RIGHT).transpose();
		else
			return points.row(BOTTOM_RIGHT).transpose();
	}
	else {
		if (dir[1]>0)
			return points.row(TOP_LEFT).transpose();
		else
			return points.row(BOTTOM_LEFT).transpose();
	}
}

void RectangularObstacle::bloat(float amount) {
	reset(center,maxEx+amount,unrot.transpose());
}

//! Decreases the size of the obstacle in all directions by at least @a amount
void RectangularObstacle::contract(float amount) {
	bloat(-amount);
}

void RectangularObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(2,0,false);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > d(2,0,false);
	plist::Angle ang;
	addEntry("Center",c);
	addEntry("Dimensions",d);
	addEntry("Orientation",ang);
	PlannerObstacle2D::loadXML(node);
	removeEntry("Center");
	removeEntry("Dimensions");
	removeEntry("Orientation");
	reset(fmat::pack(c[0],c[1]), fmat::pack(d[0]/2,d[1]/2), ang);
}
void RectangularObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(2,0,false);
	center.exportTo(c);
	c.setSaveInlineStyle(true);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > d(2,0,false);
	d[0] = getWidth();
	d[1] = getHeight();
	d.setSaveInlineStyle(true);
	plist::Angle ang = getOrientation();
	plist::Dictionary tmp(*this);
	tmp.addEntry("Center",c,"Center point");
	tmp.addEntry("Dimensions",d,"Width and height");
	tmp.addEntry("Orientation",ang,"Rotation (radians unless you specify ° suffix)");
	tmp.saveXML(node);
}

float RectangularObstacle::getOrientation() const {
	return std::atan2(unrot(0,1),unrot(0,0));
}

fmat::Column<2> CircularObstacle::gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const {
	fmat::Column<2> v = pt - center;
	fmat::fmatReal n = v.norm();
	if(n == 0)
		return fmat::pack(radius,0);
	return v * ((radius - n)/n);
}

std::string CircularObstacle::toString() const {
	std::ostringstream os;
	os << "CircularObstacle[" << name << ",center=" << center << ",radius=" << radius << "]";
	return os.str();
}

bool CircularObstacle::collides(const CircularObstacle& other) const {
	const fmat::Column<2> diff = center - other.center;
	const float r = radius + other.radius;
	return diff.sumSq() < r*r;
}

fmat::Column<2> CircularObstacle::getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const {
	return center + (direction / direction.norm() * radius);
}

void CircularObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries for serialization
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(2,0,false);
	plist::Primitive<fmat::fmatReal> r;
	addEntry("Center",c);
	addEntry("Radius",r);
	PlannerObstacle2D::loadXML(node);
	removeEntry("Center");
	removeEntry("Radius");
	center.importFrom(c);
	radius = r;
}
void CircularObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(2,0,false);
	c.setSaveInlineStyle(true);
	center.exportTo(c);
	plist::Primitive<fmat::fmatReal> r = radius;
	plist::Dictionary tmp(*this);
	tmp.addEntry("Center",c);
	tmp.addEntry("Radius",r);
	tmp.saveXML(node);
}

void EllipticalObstacle::reset(const fmat::Column<2>& f1, const fmat::Column<2>& f2, fmat::fmatReal s) {
	center = ( (focus1 = f1) + (focus2 = f2) ) / 2;
	semimajor = std::abs(s);
	fmat::fmatReal d = s*s - (f1-center).sumSq();
	if(d<0)
		throw std::invalid_argument("EllipticalObstacle::reset with two foci and too-short semimajor");
	semiminor = std::sqrt(d);
}

void EllipticalObstacle::reset(const fmat::Column<2>& c, fmat::fmatReal _semimajor, fmat::fmatReal _semiminor,
			       fmat::fmatReal orientation) {
	center = c;
	semimajor = _semimajor;
	semiminor = _semiminor;
	if(semimajor < semiminor) {
		std::swap(semimajor, semiminor);
		orientation+=static_cast<fmat::fmatReal>(M_PI_2);
	}
	fmat::fmatReal focusDist = std::sqrt(semimajor*semimajor - semiminor*semiminor);
	fmat::Column<2> ax = fmat::pack(focusDist*std::cos(orientation),focusDist*std::sin(orientation));
	focus1 = center + ax;
	focus2 = center - ax;
}

fmat::Column<2> EllipticalObstacle::getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const {
	// handle circle case (semimajor == semiminor)
	if (semimajor == semiminor)
		return center + (direction / direction.norm() * semimajor); // or semiminor...
	
	// otherwise, we use a Lagrange Multiplier.
	fmat::Column<2> dir = getOrientation().transpose() * direction;
	
	// if we're zero on any axis, it's easy.
	if (dir[0] == 0) {
		return getOrientation() * fmat::pack(0, sgn(dir[1]) * semiminor) + center;
	}
	else if (dir[1] == 0) {
		return getOrientation() * fmat::pack(sgn(dir[0]) * semimajor, 0) + center;
	}
	
	// for efficiency
	fmat::fmatReal a2 = semimajor*semimajor, b2 = semiminor*semiminor;
	
	// solve for x
	fmat::fmatReal k = (dir[1]*b2)/(dir[0]*a2);
	fmat::fmatReal x = sgn(dir[0]) * std::sqrt(1/((1/(a2)) + (k*k/(b2))));
	
	return getOrientation() * fmat::pack(x, k*x) + center;
}

void EllipticalObstacle::updatePosition(const fmat::SubVector<2,const fmat::fmatReal>& newPos) {
	fmat::Column<2> diff = newPos - center;
	center = newPos;
	focus1 += diff;
	focus2 += diff;
}

void EllipticalObstacle::rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
								const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot) {
	focus1 = rot * (focus1 - origin) + origin;
	focus2 = rot * (focus2 - origin) + origin;
	center = (focus1+focus2)/2;
}

fmat::Matrix<2,2> EllipticalObstacle::getOrientation() const {
	if(semimajor==semiminor)
		return fmat::Matrix<2,2>::identity();
	fmat::fmatReal x[4] = { focus1[0]-center[0], focus1[1]-center[1] };
	fmat::fmatReal n = fmat::SubVector<2>(x).norm();
	x[0]/=n; x[1]/=n; x[2]=-x[1]; x[3]=x[0];
	return fmat::Matrix<2,2>(x);
}

fmat::Column<2> EllipticalObstacle::getPointOnEdge(const fmat::Column<2>& direction) const {
	fmat::Matrix<2,2> rot = getOrientation();
	fmat::Column<2> x = rot.transpose() * direction;
	fmat::fmatReal ratio = semimajor/semiminor;
	x[1]*=ratio;
	x *= semimajor/x.norm();
	x[1]/=ratio;
	return rot * x + center;
}

BoundingBox2D EllipticalObstacle::getBoundingBox() const {
	float orientation = getAngle();
	float t_x = std::atan(-semiminor * tan(orientation) / semimajor);
	float t_y = std::atan( semiminor / tan(orientation) / semimajor);
	BoundingBox2D b;
	// derived from parametric equation of the ellipse
	float o_sin = std::sin(orientation), t_x_sin = std::sin(t_x), t_y_sin = std::sin(t_y);
	float o_cos = std::cos(orientation), t_x_cos = std::cos(t_x), t_y_cos = std::cos(t_y);
	b.expand(fmat::pack(center[0] + semimajor*t_x_cos*o_cos - semiminor*t_x_sin*o_sin,
						center[1] + semiminor*t_y_sin*o_cos + semimajor*t_y_cos*o_sin));
	// shift t by PI for both x and y, which just flips the signs of the sin/cos
	b.expand(fmat::pack(center[0] - semimajor*t_x_cos*o_cos + semiminor*t_x_sin*o_sin,
						center[1] - semiminor*t_y_sin*o_cos - semimajor*t_y_cos*o_sin));
	return b;
}

fmat::Column<2> EllipticalObstacle::gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const {
	// TODO: completely re-do this. the circle approximation might be fine.
	if (semimajor == semiminor)
		return getPointOnEdge(pt - center) - pt;
	
	fmat::Column<2> newPt = getOrientation().transpose() * (pt - center);
	
	float a = (semimajor*semimajor - semiminor*semiminor);
	float b = semimajor * newPt[0];
	float c = semiminor * newPt[1];
	
	// http://planetmath.org/encyclopedia/QuarticFormula.html -- explanation
	
	float aa = a*a, bb = b*b;
	
	float a3 = -2*b/a;
	float a2 = (bb-aa+c*c)/aa;
	float a1 = -a3;
	float a0 = -bb/aa;
	
	float a2a2 = a2*a2, a3a3 = a3*a3;
	
	float t1 = -a3/4;
	float t2 = a2a2 - 3*a3*a1 + 12*a0;
	float t3 = (2*a2a2*a2 - 9*a3*a2*a1 + 27*a1*a1 + 27*a3a3*a0 - 72*a2*a0)/2;
	float t4 = (-a3a3*a3 + 4*a3*a2 - 8*a1)/32;
	float t5 = (3*a3a3 - 8*a2)/48;
	
	float s1 = std::sqrt(t3*t3 - std::pow(t2,3));
	float s2 = std::pow(t3 + s1, 1.0f/3.0f);
	float s3 = (1.0f/12.0f)*(t2/s2 + s2);
	float s4 = std::sqrt(t5 + s3);
	float s5 = 2*t5 - s3;
	float s6 = t4/s4;
	
	float sq1 = std::sqrt(s5 - s6);
	float sq2 = std::sqrt(s5 + s6);
	
	float sols[8];
	
	sols[0] = std::acos(t1 - s4 - sq1);
	sols[1] = std::acos(t1 - s4 + sq1);
	sols[2] = std::acos(t1 + s4 - sq2);
	sols[3] = std::acos(t1 + s4 + sq2);
	
	if (std::isnan(sols[0]) && std::isnan(sols[1]) && std::isnan(sols[2]) && std::isnan(sols[3])) {
		//std::cout << a << ' ' << b << ' ' << c << std::endl;
		return fmat::pack(0,0);
	}
	
	// if we're below the x-axis, negative angles
	if (newPt[1] < 0) {
		for (int i = 0; i < 4; i++)
			sols[i] = -sols[i];
	}
	
	float ori = getAngle(), so = std::sin(ori), co = std::cos(ori);
	
	fmat::Column<2> grads[4];
	float minMag = std::numeric_limits<float>::infinity();
	int minMagIndex = 0;
	for (int i = 0; i < 4; i++) {
		if (std::isnan(sols[i])) continue;
		float st = std::sin(sols[i]), ct = std::cos(sols[i]);
		grads[i] = fmat::pack(semimajor*ct*co - semiminor*st*so,
							  semimajor*ct*so + semiminor*st*co) + center - pt;
		float sumSq = grads[i].sumSq();
		if (sumSq < minMag) {
			minMag = sumSq;
			minMagIndex = i;
		}
	}
	
	return grads[minMagIndex];
}

std::string EllipticalObstacle::toString() const {
	std::ostringstream os;
	os << "EllipticalObstacle[" << name << ",focus1=" << focus1 << ",focus2=" << focus2 << "," << std::endl
	<< "				   center=" << center << ",semimajor=" << semimajor << ",semiminor=" << semiminor << std::endl
	<< "				   orientation=" << getAngle() << "]";
	return os.str();
}

void EllipticalObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries for serialization
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(2,0,false);
	plist::Primitive<fmat::fmatReal> smajor;
	plist::Primitive<fmat::fmatReal> sminor;
	plist::Angle ori;
	addEntry("Center",c);
	addEntry("Orientation",ori);
	addEntry("Semimajor",smajor);
	addEntry("Semiminor",sminor);
	PlannerObstacle2D::loadXML(node);
	removeEntry("Center");
	removeEntry("Orientation");
	removeEntry("Semimajor");
	removeEntry("Semiminor");
	center.importFrom(c);
	reset(center,smajor,sminor,ori);
}

void EllipticalObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(2,0,false);
	c.setSaveInlineStyle(true);
	center.exportTo(c);
	plist::Primitive<fmat::fmatReal> smajor = semimajor;
	plist::Primitive<fmat::fmatReal> sminor = semiminor;
	plist::Angle ori = getAngle();
	plist::Dictionary tmp(*this);
	tmp.addEntry("Center",c);
	tmp.addEntry("Orientation",ori);
	tmp.addEntry("Semimajor",smajor);
	tmp.addEntry("Semiminor",sminor);
	tmp.saveXML(node);
}

void ConvexPolyObstacle::hull(const std::set<fmat::Column<2> >& p) {
	size_t k = 0;
	points.resize(p.size()+1);
	
	// Build lower hull
	for(std::set<fmat::Column<2> >::const_iterator it=p.begin(); it!=p.end(); ++it) {
		while(k>=2 && fmat::crossProduct(points[k-1]-points[k-2], *it-points[k-2])[2] <= 0)
			--k;
		points[k++] = *it;
	}
	
	// Build upper hull
	const size_t t = k+1;
	std::set<fmat::Column<2> >::const_reverse_iterator it=p.rbegin();
	for(++it; it!=p.rend(); ++it) {
		while(k >= t && fmat::crossProduct(points[k-1]-points[k-2], *it-points[k-2])[2] <= 0)
			--k;
		points[k++] = *it;
	}
	
	// update normals
	normals.resize(k-1);
	for(size_t i=0; i<k-1; ++i) {
		fmat::Column<2> d = points[i+1] - points[i];
		// this is why we specify counter-clockwise... right normal is outside
		normals[i] = (fmat::pack(d[1],-d[0]) / d.norm());
	}
	points.resize(k-1);
}

void ConvexPolyObstacle::addPoint(const fmat::Column<2>& p) {
	if(points.size()>0) {
		fmat::Column<2> d = p - points.back();
		// this is why we specify counter-clockwise... right normal is outside
		d = fmat::pack(d[1],-d[0]) / d.norm();
		if(normals.size()==points.size())
			normals.back() = d;
		else
			normals.push_back(d);
	}
	points.push_back(p);
}

void ConvexPolyObstacle::close() {
	ASSERTRET(points.size()==normals.size()+1,"ConvexPolyObstacle already closed: " << points.size() << " points " << normals.size() << " normals");
	fmat::Column<2> d = points.front() - points.back();
	// this is why we specify counter-clockwise... right normal is outside
	d = fmat::pack(d[1],-d[0]) / d.norm();
	normals.push_back(d);
}

bool ConvexPolyObstacle::collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const {
	ASSERTRETVAL(points.size()>=3,"ConvexPolyObstacle: not enough points (" << points.size() << ", need at least 3)",false);
	ASSERTRETVAL(points.size()==normals.size(),"ConvexPolyObstacle not closed",false);
	for(size_t i=0; i<points.size(); ++i) {
		if(fmat::dotProduct(point-points[i],normals[i]) >= 0)
			return false;
	}
	return true;
}

bool ConvexPolyObstacle::collides(const RectangularObstacle& other) const {
	ASSERTRETVAL(points.size()>=3,"ConvexPolyObstacle: not enough points (" << points.size() << ", need at least 3)",false);
	ASSERTRETVAL(points.size()==normals.size(),"ConvexPolyObstacle not closed",false);
	for(size_t i=0; i<points.size(); ++i) {
		fmat::Matrix<4,2> c(other.points);
		c.column(0)-=points[i][0];
		c.column(1)-=points[i][1];
		fmat::Column<4> d = c * normals[i];
		if(d.min() >= 0)
			return false;
	}
	fmat::Column<2> p = other.unrot * points[0];
	fmat::fmatReal minX=p[0],maxX=p[0],minY=p[1],maxY=p[1];
	for(size_t i=1; i<points.size(); ++i) {
		p = other.unrot * points[i];
		if(p[0]<minX)
			minX=p[0];
		else if(maxX<p[0])
			maxX=p[0];
		if(p[1]<minY)
			minY=p[1];
		else if(maxY<p[1])
			maxY=p[1];
	}
	return (other.minEx[0]<maxX && minX<other.maxEx[0] && other.minEx[1]<maxY && minY<other.maxEx[1]);
}

bool ConvexPolyObstacle::collides(const CircularObstacle& other) const {
	ASSERTRETVAL(points.size()>=3,"ConvexPolyObstacle: not enough points (" << points.size() << ", need at least 3)",false);
	ASSERTRETVAL(points.size()==normals.size(),"ConvexPolyObstacle not closed",false);
	const fmat::fmatReal r2 = other.getRadius() * other.getRadius();
	bool outside = false; // once we see we are outside, must 'hit' corner or edge (see final return)
	bool testedLast = false; // prevents duplicate corner tests on consecutive edges (not a big issue though)
	for(size_t i=0; i<points.size(); ++i) {
		fmat::Column<2> v = other.getCenter()-points[i];
		fmat::fmatReal d = fmat::dotProduct(v,normals[i]);
		if(d >= other.getRadius())
			return false;
		if(d >= 0) {
			outside = true;
			// close... now project onto the edge
			d = v[0]*-normals[i][1] + v[1]*normals[i][0]; // reuse the normal, just flip it
			// see if it hits either endpoint or the edge itself
			if(d >= 0) {
				const fmat::Column<2>& next = (i+1==points.size() ? points[0] : points[i+1]);
				fmat::fmatReal len = (points[i] - next).norm();
				if(d <= len) // within edge itself
					return true;
				if(testedLast) {
					testedLast=false;
				} else if(d < len+other.getRadius()) {
					// may hit next corner point
					if((other.getCenter() - next).sumSq() < r2)
						return true;
					testedLast=true;
				}
			} else {
				if(d > -other.getRadius()) {
					// may hit this corner point
					if(v.sumSq() < r2)
						return true;
				}
				testedLast=false;
			}
		}
	}
	return !outside;
}

bool ConvexPolyObstacle::collides(const ConvexPolyObstacle& other) const {
	ASSERTRETVAL(points.size()>=3,"ConvexPolyObstacle: not enough points (" << points.size() << ", need at least 3)",false);
	ASSERTRETVAL(points.size()==normals.size(),"ConvexPolyObstacle not closed",false);
	ASSERTRETVAL(other.points.size()==other.normals.size(),"ConvexPolyObstacle not closed",false);
	for(size_t i=0; i<points.size(); ++i) {
		bool inter=false;
		for(size_t j=0; j<other.points.size(); ++j) {
			if(fmat::dotProduct(other.points[j]-points[i],normals[i]) < 0) {
				inter=true;
				break;
			}
		}
		if(!inter)
			return false;
	}
	for(size_t i=0; i<other.points.size(); ++i) {
		bool inter=false;
		for(size_t j=0; j<points.size(); ++j) {
			if(fmat::dotProduct(points[j]-other.points[i],other.normals[i]) < 0) {
				inter=true;
				break;
			}
		}
		if(!inter)
			return false;
	}
	return true;
}

fmat::Column<2> ConvexPolyObstacle::getSupport(const fmat::SubVector<2,const fmat::fmatReal>& direction) const {
	ASSERTRETVAL(points.size()>0,"ConvexPolyObstacle: no points",fmat::pack(0,0));
	fmat::fmatReal max = fmat::dotProduct(direction, points[0]);
	int maxIndex = 0;
	for (unsigned int i = 1; i < points.size(); i++) {
		fmat::fmatReal newMax = fmat::dotProduct(direction, points[i]);
		if (newMax > max) {
			max = newMax;
			maxIndex = i;
		}
	}
	return points[maxIndex];
}

fmat::Column<2> ConvexPolyObstacle::getCenter() const {
	ASSERTRETVAL(points.size()>0,"ConvexPolyObstacle empty (" << points.size() << ", need at least 1 for getCenter())",fmat::Column<2>());
	fmat::Column<2> ans=points.front();
	for(size_t i=1; i<points.size(); ++i) {
		ans+=points[i];
	}
	return ans/points.size();
}

void ConvexPolyObstacle::rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
								const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot) {
	std::vector<fmat::Column<2> > tmpPoints = points;
	clear();
	for(size_t i=0; i<tmpPoints.size(); ++i) {
		addPoint(rot*(tmpPoints[i] - origin) + origin);
	}
	close();
}

BoundingBox2D ConvexPolyObstacle::getBoundingBox() const {
	BoundingBox2D bb;
	for(size_t i=0; i<points.size(); ++i)
		bb.expand(points[i]);
	return bb;
}

void ConvexPolyObstacle::offset(const fmat::Column<2>& off) {
	for(size_t i=0; i<points.size(); ++i) {
		points[i]+=off;
	}
}

fmat::Column<2> ConvexPolyObstacle::gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const {
	ASSERTRETVAL(points.size()>=3,"ConvexPolyObstacle: not enough points (" << points.size() << ", need at least 3)",fmat::Column<2>());
	ASSERTRETVAL(points.size()==normals.size(),"ConvexPolyObstacle not closed",fmat::Column<2>());
	fmat::fmatReal closest2=std::numeric_limits<fmat::fmatReal>::infinity();
	fmat::Column<2> ans;
	bool testedLast = false; // prevents duplicate corner tests on consecutive edges (not a big issue though)
	for(size_t i=0; i<points.size(); ++i) {
		fmat::Column<2> v = pt-points[i];
		// project onto the edge
		const fmat::fmatReal d = v[0]*-normals[i][1] + v[1]*normals[i][0]; // reuse the normal, just flip it
		// is it closer to an endpoint or the edge itself
		if(d >= 0) {
			const fmat::Column<2>& next = (i+1==points.size() ? points[0] : points[i+1]);
			fmat::fmatReal len = (points[i] - next).norm();
			if(d <= len) {
				fmat::fmatReal n = fmat::dotProduct(v,normals[i]);
				fmat::fmatReal n2 = n*n;
				if(n2<closest2) {
					closest2 = n2;
					ans = normals[i]*-n;
				}
			}
			if(testedLast) {
				testedLast=false;
			} else if(d < len) {
				// test corner point
				fmat::fmatReal n2 = (next - pt).sumSq();
				if(n2 < closest2) {
					closest2 = n2;
					ans = next - pt;
				}
				testedLast=true;
			}
		} else {
			// may hit this corner point
			fmat::fmatReal n2 = v.sumSq();
			if(n2 < closest2) {
				closest2 = n2;
				ans = -v;
			}
			testedLast=false;
		}
	}
	return ans;
}

std::string ConvexPolyObstacle::toString() const {
	std::ostringstream os;
	os << "ConvexPolyObstacle[" << name << ",#points=" << points.size() << ",#normals=" << normals.size() << "]";
	return os.str();
}


void ConvexPolyObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries for serialization
	plist::ArrayOf< plist::Point > ps;
	addEntry("Points",ps);
	PlannerObstacle2D::loadXML(node);
	removeEntry("Points");
	points.resize(ps.size());
	for(size_t i=0; i<ps.size(); ++i)
		points[i].importFrom(ps[i]);
}

void ConvexPolyObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Point > ps;
	for(size_t i=0; i<ps.size(); ++i) {
		points[i].exportTo(ps[i]);
		ps.removeEntry(2); // only two dimensions
	}
	plist::Dictionary tmp(*this);
	tmp.addEntry("Points",ps);
	tmp.saveXML(node);
}

void HierarchicalObstacle::recalculateBoundingBox() {
	aabb = BoundingBox2D();
	for (std::vector<PlannerObstacle2D*>::iterator it = components.begin(); it != components.end(); ++it)
		expandBoundingBox(**it);
}

void HierarchicalObstacle::expandBoundingBox(PlannerObstacle2D& other) {
	other.rotate(fmat::ZERO2, rotation);
	fmat::Column<2> oldCenter = other.getCenter();
	other.updatePosition(oldCenter+center);
	
	aabb.expand(other.getBoundingBox());
	
	other.updatePosition(oldCenter);
	other.rotate(fmat::ZERO2, rotation.transpose());
}

bool HierarchicalObstacle::collides(const fmat::SubVector<2,const fmat::fmatReal>& point) const {
	// if we're not inside the box, no collision
	if( !aabb.collides(point) )
		return false;
	
	// rotate point into Hierarchical Obstacle frame
	const fmat::Column<2> newPt = rotation.transpose() * (point - center);
	
	// if we are, test further
	for (unsigned int i = 0; i < components.size(); i++)
		if (components[i]->collides(newPt))
			return true;
	
	return false;
}

bool HierarchicalObstacle::collides(const PlannerObstacle2D& other) const {
	/*RectangularObstacle bounds(aabb.getCenter(), aabb.getDimensions()/2, 0);
	// if no collision with the box, done.
	if (!bounds.collides(other))
		return false;*/
	
	PlannerObstacle2D* transformed = dynamic_cast<PlannerObstacle2D*>(other.clone());
	transformed->updatePosition(transformed->getCenter()-center);
	transformed->rotate(fmat::ZERO2, rotation.transpose());
	
	// if there is, test further
	bool collision = false;
	for (unsigned int i = 0; i < components.size(); i++)
		if (components[i]->collides(*transformed)) {
			collision = true;
			break;
		}
	delete transformed;
	/*
	std::cout << (collision ? "" : "no ") 
						<< "collides: center=" << center << ",rot=" << atan2(rotation(1,0),rotation(0,0))/M_PI*180 <<" deg. with "
						<< other << " = " << collision << std::endl;
	*/
	return collision;
}

void HierarchicalObstacle::updatePosition(const fmat::SubVector<2,const fmat::fmatReal>& newPos) {
	fmat::Column<2> diff = newPos - center;
	aabb.min += diff;
	aabb.max += diff;
	center = newPos;
}

void HierarchicalObstacle::rotate(const fmat::SubVector<2,const fmat::fmatReal>& origin,
								  const fmat::SubMatrix<2,2,const fmat::fmatReal>& rot) {
	rotation = rot * rotation;
	center = rot * (center - origin) + origin;
	recalculateBoundingBox();
}

fmat::Column<2> HierarchicalObstacle::gradient(const fmat::SubVector<2,const fmat::fmatReal>& pt) const {
	fmat::Column<2> transformedPt = rotation.transpose() * (pt - center);
	ASSERTRETVAL(components.size()>0,"HierarchicalObstacle: no components, can't determine gradient",fmat::Column<2>());
	fmat::Column<2> minGradient;
	bool minSet = false;
	for (unsigned int i = 0; i < components.size(); ++i) {
		fmat::Column<2> newGradient = components[i]->gradient(transformedPt);
		bool collision = false;
		for (unsigned int j = 0; j < components.size(); ++j) {
			if (i == j) continue;
			if (components[j]->collides(transformedPt+newGradient)) {
				collision = true;
				break;
			}
		}
		
		if (collision)
			continue;
		
		if (!minSet) {
			minGradient = newGradient;
			minSet = true;
		}
		else if (newGradient.sumSq() < minGradient.sumSq())
			minGradient = newGradient;
	}
	
	return rotation * minGradient;
}

std::string HierarchicalObstacle::toString() const {
	std::ostringstream os;
	os << "HierarchicalObstacle[" << name << ",#components=" << components.size() << "]";
	return os.str();
}

std::string HierarchicalObstacle::componentsToString() const {
	std::ostringstream os;
	os << toString() << std::endl;
	for (size_t i = 0; i < components.size(); i++) {
		HierarchicalObstacle* ho = dynamic_cast<HierarchicalObstacle*>(components[i]);
		if (ho)
			os << ho->componentsToString() << std::endl;
		else
			os << components[i]->toString() << std::endl;
	}
	return os.str();
}

void BoxObstacle::reset(fmat::Column<3> centerPoint,
						const fmat::SubVector<3,const fmat::fmatReal>& extents,
						const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot) {
	center = centerPoint;
	fmat::Column<3> off = rot * extents;
	points.row(TOP_UPPER_RIGHT) = &(center + off)[0]; // 0
	points.row(BOTTOM_LOWER_LEFT) = &(center - off)[0]; // 6
	off = rot * fmat::pack(extents[0],-extents[1],extents[2]);
	points.row(TOP_LOWER_RIGHT) = &(center + off)[0]; // 1
	points.row(BOTTOM_UPPER_LEFT) = &(center - off)[0]; // 7
	off = rot * fmat::pack(-extents[0],-extents[1],extents[2]);
	points.row(TOP_LOWER_LEFT) = &(center + off)[0]; // 2
	points.row(BOTTOM_UPPER_RIGHT) = &(center - off)[0]; // 4
	off = rot * fmat::pack(-extents[0],extents[1],extents[2]);
	points.row(TOP_UPPER_LEFT) = &(center + off)[0]; // 3
	points.row(BOTTOM_LOWER_RIGHT) = &(center - off)[0]; // 5
	
	bBox.min = &points.minR()[0];
	bBox.max = &points.maxR()[0];
	
	unrot = rot.transpose(); // now switch to inverse
	centerPoint = unrot * centerPoint;
	maxEx = centerPoint + extents;
	minEx = centerPoint - extents;
}

void BoxObstacle::updatePosition(const fmat::SubVector<3, const fmat::fmatReal>& newPos) {
	fmat::Column<3> diff = newPos - center;
	center = newPos;
	bBox.min += diff;
	bBox.max += diff;
	points.column(0)+=diff[0];
	points.column(1)+=diff[1];
	points.column(2)+=diff[2];
	
	diff = unrot * diff;
	minEx += diff;
	maxEx += diff;
}

void BoxObstacle::rotate(const fmat::SubVector<3,const fmat::fmatReal>& origin,
						 const fmat::SubMatrix<3,3,const fmat::fmatReal>& rot) {
	fmat::Column<3> newCenter = rot * (center - origin) + origin;
	reset(newCenter, getExtents(), unrot.transpose() * rot);
}

std::string BoxObstacle::toString() const {
	std::ostringstream os;
	os << "BoxObstacle[" << name << ",points=" << points.fmt() << ",unrot=" << unrot.fmt() << "]";
	return os.str();
}

bool BoxObstacle::collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const {
	const fmat::Column<3> p2 = unrot * point;
	return (minEx[0]<p2[0] && p2[0]<maxEx[0] &&
			minEx[1]<p2[1] && p2[1]<maxEx[1] &&
			minEx[2]<p2[2] && p2[2]<maxEx[2]);
}

bool BoxObstacle::collides(const BoxObstacle& other) const {
	float ra, rb;
	fmat::Matrix<3,3> r, absR;
	
	// Compute rotation matrix expressing the other in this coordinate frame
	r = other.getOrientation() * unrot;
	
	// Compute translation, in terms of a's coordinate frame
	fmat::Column<3> diff = unrot * (other.getCenter() - center);
	
	// Extents
	fmat::Column<3> extents = getExtents();
	fmat::Column<3> otherExtents = other.getExtents();
	
	// Compute common subexpressions. Add in an epsilon term to
	// counteract arithmetic errors when two edges are parallel and
	// their cross product is (near) null (see text for details)
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			absR(i,j) = std::abs(r(i,j)) + 1e-10;
	
	// Test axes L = A0, L = A1, L = A2
	for (int i = 0; i < 3; i++) {
		ra = extents[i];
		rb = fmat::dotProduct(otherExtents, fmat::Row<3>(absR.row(i)));
		if (std::abs(diff[i]) > ra + rb) return false;
	}
	
	// Test axes L = B0, L = B1, L = B2
	for (int i = 0; i < 3; i++) {
		ra = fmat::dotProduct(extents, absR.column(i));
		rb = otherExtents[i];
		if (std::abs(fmat::dotProduct(diff, r.column(i))) > ra + rb) return false;
	}
	
	// Test axis L = A0 x B0
	ra = extents[1] * absR(2,0) + extents[2] * absR(1,0);
	rb = otherExtents[1] * absR(0,2) + otherExtents[2] * absR(0,1);
	if (std::abs(diff[2] * r(1,0) - diff[1] * r(2,0)) > ra + rb) return false;
	
	// Test axis L = A0 x B1
	ra = extents[1] * absR(2,1) + extents[2] * absR(1,1);
	rb = otherExtents[0] * absR(0,2) + otherExtents[2] * absR(0,0);
	if (std::abs(diff[2] * r(1,1) - diff[1] * r(2,1)) > ra + rb) return false;
	
	// Test axis L = A0 x B2
	ra = extents[1] * absR(2,2) + extents[2] * absR(1,2);
	rb = otherExtents[0] * absR(0,1) + otherExtents[1] * absR(0,0);
	if (std::abs(diff[2] * r(1,2) - diff[1] * r(2,2)) > ra + rb) return false;
	
	// Test axis L = A1 x B0
	ra = extents[0] * absR(2,0) + extents[2] * absR(0,0);
	rb = otherExtents[1] * absR(1,2) + otherExtents[2] * absR(1,1);
	if (std::abs(diff[0] * r(2,0) - diff[2] * r(0,0)) > ra + rb) return false;
	
	// Test axis L = A1 x B1
	ra = extents[0] * absR(2,1) + extents[2] * absR(0,1);
	rb = otherExtents[0] * absR(1,2) + otherExtents[2] * absR(1,0);
	if (std::abs(diff[0] * r(2,1) - diff[2] * r(0,1)) > ra + rb) return false;
	
	// Test axis L = A1 x B2
	ra = extents[0] * absR(2,2) + extents[2] * absR(0,2);
	rb = otherExtents[0] * absR(1,1) + otherExtents[1] * absR(1,0);
	if (std::abs(diff[0] * r(2,2) - diff[2] * r(0,2)) > ra + rb) return false;
	
	// Test axis L = A2 x B0
	ra = extents[0] * absR(1,0) + extents[1] * absR(0,0);
	rb = otherExtents[1] * absR(2,2) + otherExtents[2] * absR(2,1);
	if (std::abs(diff[1] * r(0,0) - diff[0] * r(1,0)) > ra + rb) return false;
	
	// Test axis L = A2 x B1
	ra = extents[0] * absR(1,1) + extents[1] * absR(0,1);
	rb = otherExtents[0] * absR(2,2) + otherExtents[2] * absR(2,0);
	if (std::abs(diff[1] * r(0,1) - diff[0] * r(1,1)) > ra + rb) return false;
	
	// Test axis L = A2 x B2
	ra = extents[0] * absR(1,2) + extents[1] * absR(0,2);
	rb = otherExtents[0] * absR(2,1) + otherExtents[1] * absR(2,0);
	if (std::abs(diff[1] * r(0,2) - diff[0] * r(1,2)) > ra + rb) return false;
	
	// Since no separating axis found, the OBBs must be intersecting
	return true;
}

fmat::Column<3> BoxObstacle::getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const {
	const fmat::Column<3> dir = unrot * direction;
	// choose the closest corner
	if (dir[0]>0) {
		if (dir[1]>0) {
			if (dir[2]>0) {
				return points.row(TOP_UPPER_RIGHT).transpose();
			} else {
				return points.row(BOTTOM_UPPER_RIGHT).transpose();
			}
		} else {
			if (dir[2]>0) {
				return points.row(TOP_LOWER_RIGHT).transpose();
			} else {
				return points.row(BOTTOM_LOWER_RIGHT).transpose();
			}
		}
	} else {
		if (dir[1]>0) {
			if (dir[2]>0) {
				return points.row(TOP_UPPER_LEFT).transpose();
			} else {
				return points.row(BOTTOM_UPPER_LEFT).transpose();
			}
		} else {
			if (dir[2]>0) {
				return points.row(TOP_LOWER_LEFT).transpose();
			} else {
				return points.row(BOTTOM_LOWER_LEFT).transpose();
			}
		}
	}
}

fmat::Column<3> BoxObstacle::gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const {
	fmat::Column<3> d = pt - center;
	// Start result at center of box; make steps from there
	fmat::Column<3> q = center;
	// For each OBB axis...
	for (int i = 0; i < 3; i++) {
		// ...project d onto that axis to get the distance
		// along the axis of d from the box center
		float dist = fmat::dotProduct(d, fmat::Row<3>(unrot.row(i)));
		// If distance farther than the box extents, clamp to the box
		if (dist > getExtents()[i]) dist = getExtents()[i];
		if (dist < -getExtents()[i]) dist = -getExtents()[i];
		// Step that distance along the axis to get world coordinate
		q += dist * getOrientation().column(i);
	}
	return q;
}

void BoxObstacle::bloat(float amount) {
	reset(center,maxEx+amount,unrot.transpose());
}

void BoxObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > d(3,0,false);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > r(9,0,false);
	addEntry("Center",c);
	addEntry("Dimensions",d);
	addEntry("Orientation",r);
	PlannerObstacle3D::loadXML(node);
	removeEntry("Center");
	removeEntry("Dimensions");
	removeEntry("Orientation");
	fmat::Matrix<3,3> rot;
	reset(fmat::pack(c[0],c[1],c[2]), fmat::pack(d[0]/2,d[1]/2,d[2]/2), rot.importFromCMajor(r));
}

void BoxObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	center.exportTo(c);
	c.setSaveInlineStyle(true);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > d(3,0,false);
	d[0] = getLength();
	d[1] = getWidth();
	d[2] = getHeight();
	d.setSaveInlineStyle(true);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > r(9,0,false);
	unrot.transpose().exportToCMajor(r);
	plist::Dictionary tmp(*this);
	tmp.addEntry("Center",c,"Center point");
	tmp.addEntry("Dimensions",d,"Width and height");
	tmp.addEntry("Orientation",r,"Rotation (3x3 Matrix condensed to float[9])");
	tmp.saveXML(node);
}

BoundingBox3D CylindricalObstacle::getBoundingBox() const { return BoundingBox3D(); }

bool CylindricalObstacle::collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const {
	fmat::Column<3> newPt = orientation.transpose() * point;
	if (std::abs(newPt[2]) < halfHeight && std::sqrt(newPt[0]*newPt[0] + newPt[1]*newPt[1]) < radius)
		return true;
	else
		return false;
}

fmat::Column<3> CylindricalObstacle::getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const {
	fmat::Column<3> dir = orientation.transpose() * direction;
	fmat::fmatReal sigma = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
	if (sigma > 0) {
		return orientation.transpose() *
		fmat::pack(radius / sigma * dir[0],
				   radius / sigma * dir[1],
				   sgn(dir[2]) * halfHeight);
	}
	else {
		return orientation.transpose() *
		fmat::pack(0, 0, sgn(dir[2]) * halfHeight);
	}
}

fmat::Column<3> CylindricalObstacle::gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const { return fmat::Column<3>(); }

std::string CylindricalObstacle::toString() const {
	std::ostringstream os;
	os << "CylindricalObstacle[" << name << ",center=" << center << std::endl
	<< ",orientation=" << orientation << std::endl << ",half-height=" << halfHeight << ",radius=" << radius << std::endl;
	return os.str();
}

void CylindricalObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries for serialization
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > o(9,0,false);
	plist::Primitive<fmat::fmatReal> r;
	plist::Primitive<fmat::fmatReal> hh;
	addEntry("Center",c);
	addEntry("Orientation",o);
	addEntry("Radius",r);
	addEntry("HalfHeight",hh);
	PlannerObstacle3D::loadXML(node);
	removeEntry("Center");
	removeEntry("Orientation");
	removeEntry("Radius");
	removeEntry("HalfHeight");
	center.importFrom(c);
	radius = r;
	orientation.importFromCMajor(o);
	halfHeight = hh;
}

void CylindricalObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	c.setSaveInlineStyle(true);
	center.exportTo(c);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > o(9,0,false);
	o.setSaveInlineStyle(true);
	orientation.exportToCMajor(o);
	plist::Primitive<fmat::fmatReal> r = radius;
	plist::Primitive<fmat::fmatReal> hh = halfHeight;
	plist::Dictionary tmp(*this);
	tmp.addEntry("Center",c);
	tmp.addEntry("Radius",r);
	tmp.addEntry("Orientation",o);
	tmp.addEntry("HalfHeight",hh);
	tmp.saveXML(node);
}

bool SphericalObstacle::collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const {
	// point intersects if its distance from the center is less than the radius
	return (point - center).sumSq() <= radius * radius;
}

bool SphericalObstacle::collides(const SphericalObstacle& other) const {
	const fmat::Column<3> diff = center - other.center;
	const float r = radius + other.radius;
	return diff.sumSq() < r*r;
}

fmat::Column<3> SphericalObstacle::getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const {
	return center + (direction / direction.norm() * radius);
}

fmat::Column<3> SphericalObstacle::gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const {
	fmat::Column<3> v = pt - center;
	fmat::fmatReal n = v.norm();
	if(n == 0)
		return fmat::pack(radius,0,0);
	return v * ((radius - n)/n);
}

std::string SphericalObstacle::toString() const {
	std::ostringstream os;
	os << "SphericalObstacle[" << name << ",center=" << center << ",radius=" << radius << "]" << std::endl;
	return os.str();
}

void SphericalObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries for serialization
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	plist::Primitive<fmat::fmatReal> r;
	addEntry("Center",c);
	addEntry("Radius",r);
	PlannerObstacle3D::loadXML(node);
	removeEntry("Center");
	removeEntry("Radius");
	center.importFrom(c);
	radius = r;
}

void SphericalObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	c.setSaveInlineStyle(true);
	center.exportTo(c);
	plist::Primitive<fmat::fmatReal> r = radius;
	plist::Dictionary tmp(*this);
	tmp.addEntry("Center",c);
	tmp.addEntry("Radius",r);
	tmp.saveXML(node);
}

BoundingBox3D EllipsoidObstacle::getBoundingBox() const { return BoundingBox3D(); }

bool EllipsoidObstacle::collides(const fmat::SubVector<3,const fmat::fmatReal>& point) const {
	fmat::Column<3> newPt = orientation.transpose() * point;
	
	// shrink ourselves to circle world, then test
	newPt[0] /= extents[0];
	newPt[1] /= extents[1];
	newPt[2] /= extents[2];
	if (newPt.sumSq() < 1)
		return true;
	else
		return false;
}

fmat::Column<3> EllipsoidObstacle::getSupport(const fmat::SubVector<3,const fmat::fmatReal>& direction) const {
	fmat::Column<3> dir = orientation.transpose() * direction;
	
	if (dir[0] == 0) {
		fmat::Column<2> s = get2Support(fmat::pack(dir[1],dir[2]));
		return orientation * fmat::pack(0, s[0], s[1]) + center;
	}
	else if (dir[1] == 0) {
		fmat::Column<2> s = get2Support(fmat::pack(dir[0],dir[2]));
		return orientation * fmat::pack(s[0], 0, s[1]) + center;
	}
	else if (dir[2] == 0) {
		fmat::Column<2> s = get2Support(fmat::pack(dir[0],dir[1]));
		return orientation * fmat::pack(s[0], s[1], 0) + center;
	}
	
	// convenience
	fmat::fmatReal a2 = extents[0]*extents[0], b2 = extents[1]*extents[1], c2 = extents[2]*extents[2];
	
	// solve for x (lagrange multiplier)
	fmat::fmatReal k1 = (dir[1]*b2)/(dir[0]*a2);
	fmat::fmatReal k2 = (dir[2]*c2)/(dir[0]*a2);
	fmat::fmatReal x = sgn(dir[0]) * std::sqrt(1/((1/(a2)) + (k1*k1/(b2)) + (k2*k2/(c2))));
	
	return orientation * fmat::pack(x, k1*x, k2*x) + center;
}

fmat::Column<2> EllipsoidObstacle::get2Support(const fmat::SubVector<2,const fmat::fmatReal>& direction) const {
	// if we're zero on either axis, we just use the extents of the other.
	if (direction[0] == 0) {
		return fmat::pack(0, sgn(direction[1]) * extents[1]);
	}
	else if (direction[1] == 0) {
		return fmat::pack(sgn(direction[0]) * extents[0], 0);
	}
	
	// convenience
	fmat::fmatReal a2 = extents[0]*extents[0], b2 = extents[1]*extents[1];
	
	// solve for x (lagrange multiplier)
	fmat::fmatReal k = (direction[1]*b2)/(direction[0]*a2);
	fmat::fmatReal x = sgn(direction[0]) * std::sqrt(1/((1/(a2)) + (k*k/(b2))));
	
	return fmat::pack(x, k*x);
}

fmat::Column<3> EllipsoidObstacle::gradient(const fmat::SubVector<3,const fmat::fmatReal>& pt) const { return fmat::Column<3>(); }

std::string EllipsoidObstacle::toString() const {
	std::ostringstream os;
	os << "EllipsoidObstacle[" << name << ",center=" << center << std::endl
	<< ",orientation=" << orientation << std::endl << ",extents=" << extents << std::endl;
	return os.str();
}

void EllipsoidObstacle::loadXML(xmlNode* node) {
	// temporarily add plist entries for serialization
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > o(9,0,false);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > e(3,0,false);
	addEntry("Center",c);
	addEntry("Orientation",o);
	addEntry("Extents",e);
	PlannerObstacle3D::loadXML(node);
	removeEntry("Center");
	removeEntry("Orientation");
	removeEntry("Extents");
	center.importFrom(c);
	orientation.importFromCMajor(o);
	extents.importFrom(e);
}

void EllipsoidObstacle::saveXML(xmlNode * node) const {
	// temporarily add plist entries to get 'normalized' coordinates, then convert
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > c(3,0,false);
	c.setSaveInlineStyle(true);
	center.exportTo(c);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > o(9,0,false);
	o.setSaveInlineStyle(true);
	orientation.exportToCMajor(o);
	plist::ArrayOf< plist::Primitive<fmat::fmatReal> > e(3,0,false);
	e.setSaveInlineStyle(true);
	extents.exportTo(e);
	plist::Dictionary tmp(*this);
	tmp.addEntry("Center",c);
	tmp.addEntry("Orietation",o);
	tmp.addEntry("Extents",e);
	tmp.saveXML(node);
}
