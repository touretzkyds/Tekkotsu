#include "Shared/plist.h"

using namespace std;
// you might want to 'using namespace plist'...
// we'll be using plist:: scoping everywhere below just for clarity

/* Generic base class for "shapes"... we specify a generic Collection,
 * as its base, allows either Dictionary or Array-based! */
class Shape : virtual public plist::Collection {};


/* We will use a float for coordinates, wrapped by a plist::Primitive<>
 * The Primitive class provides transparent conversions and operations,
 * so we can usually pretend this is just a regular float! */
typedef plist::Primitive<float> coord_t;


/* A point is defined as an array of coordinates, one for each dimension.
 * We'll assume 2D points unless otherwise directed.
 * An alternative definition could use explicit 'x' and 'y' members, and/or
 * use a Dictionary with labels instead of an Array with subscripts... */
class Point : public Shape, virtual public plist::ArrayOf<coord_t> {
public:
	explicit Point(size_t n=2) : plist::ArrayOf<coord_t>(n,0), Shape() { }
	Point(float xVal, float yVal) : plist::ArrayOf<coord_t>(2,0), Shape() {
		getEntry(0)=xVal;
		getEntry(1)=yVal;
	}
};


/* We'll define a rectangle by the lower-left point and the upper-right point.
 * Further, we'll set up an accessor to maintain this invariant. */
class Rectangle : public Shape, public virtual plist::DictionaryOf<Point> {
public:
	Rectangle()
		: plist::DictionaryOf<Point>(), Shape(), lower(), upper(),
		xm(lower[0],upper[0]), ym(lower[1],upper[1])
	{
		/* The next line 'fixes' the entries during load, save, or assignment.
		 * The default is to allow dynamic resizing, so you should call this when
		 * you expect the entry structure to be immutable. (e.g. if you are using
		 * members as collection entries.) */
		setLoadSavePolicy(FIXED,SYNC);
		addEntry("lower",lower); // still can always 'explicitly' add or remove entries
		addEntry("upper",upper); // ... the LoadSavePolicy only restricts 'implicit' changes
	}
	
	/* Can provide public access to members, the Monitor will provide
	 * the flexibility usually provided by get/set accessor functions. */
	Point lower, upper;
	
	/* Note automatic conversion to value type, will never be negative due to our Monitor */
	float getWidth() const { return upper[0] - lower[0]; }
	float getHeight() const { return upper[1] - lower[1]; }
	
protected:
	/* Implements plist::PrimitiveListener to receive notification when
	 * a member's value is changed.  We could also do this by having
	 * rectangle itself be the listener instead of an inner class. */
	class Monitor : public plist::PrimitiveListener {
	public:
		Monitor(coord_t& low, coord_t& high) : _low(low), _high(high) {
			_low.addPrimitiveListener(this);
			_high.addPrimitiveListener(this);
		}
		virtual void plistValueChanged(const plist::PrimitiveBase&) {
			if(_low>_high) std::swap(_low,_high);
		}
	protected:
		coord_t &_low, &_high;
	} xm, ym;
};


/* Here's an example of a purely dynamic class... just a resizeable list
 * of points!  For a "real" implementation, you'd probably want to use
 * a dictionary, add a few more properties (e.g. 'isClosed'), and have
 * the point list as a member variable... */
class Polygon : public Shape, public virtual plist::ArrayOf<Point> {};


// Here's some sample usage...
int main(int argc, const char** argv) {
	Point p;
	p[0] = 1; // transparent conversion from value type to plist::Primitive
	p[1] = 2.5;
	p.saveFile("point.plist");
	
	Rectangle r;
	r.upper = Point(4,5); // we can assign to directly to member...
	r["lower"] = p; // ... or dynamically via operator[] lookup
	r.saveFile("rectangle.plist");
	
	// test the low/high invariant
	cout << "Original r, width: " << r.getWidth() << '\n' << r;
	/* Original r, width: 3
	 * lower.0 = 1
	 * lower.1 = 2.5
	 * upper.0 = 4
	 * upper.1 = 5 */
	
	r.upper[0] = -4; // note we assign to upper.x, but it's lower.x that will be -4
	// (because of the PrimitiveListener added by Rectangle's constructor)
	cout << "Negated r, width: " << r.getWidth() << '\n' << r;
	/* Negated r, width: 5
	 * lower.0 = -4
	 * lower.1 = 2.5
	 * upper.0 = 1
	 * upper.1 = 5 */
	
	// dynamic resizing:
	Polygon poly;
	/* Notice we are adding pointers here, vs. references in Rectangle,
	 * vs. float values in Point.  (De)allocation from pointers or converted 
	 * values is handled by the collection, whereas entries added from
	 * references are *not* deallocated. */
	poly.addEntry(new Point(1,2));
	poly.addEntry(new Point(3,4));
	poly.addEntry(new Point(5,0));
	poly.saveFile("poly.plist");
	
	return 0;
}
