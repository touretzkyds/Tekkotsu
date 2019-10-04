#include <vector>
#include "Shared/plist.h"
#include "Shared/plistSpecialty.h"
#include "Events/VisionObjectEvent.h"
#include <cmath>

using namespace plist;
using namespace std;

/* Generic base class for "shapes"... we specify a generic Collection,
 * as its base, allows either Dictionary or Array-based! */
class Shape : virtual public plist::Collection {
public:
	Shape() : plist::Collection() { setLoadSavePolicy(FIXED,SYNC); }
	~Shape()=0;
};
Shape::~Shape() {}

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
	explicit Point(size_t n=2) : plist::ArrayOf<coord_t>(n,0), Shape() {}
	Point(float xVal, float yVal) : plist::ArrayOf<coord_t>(2,0), Shape() {
		getEntry(0)=xVal;
		getEntry(1)=yVal;
	}
};

/* A simple circle: a point, a radius, and a color... */
class Circle : public Shape, public virtual plist::Dictionary {
public:
	Circle() : plist::Dictionary(), center(), radius(), fcol(0,0.5,1), icol(0,128,255) {
		addEntry("center",center);
		addEntry("radius",radius);
		addEntry("fcol",fcol,"testing floating point colors (0 to 1)");
		addEntry("icol",icol,"testing integral colors (0 to 255)");
	}
	::Point center;
	plist::Primitive<float> radius;
	
	/* A specialty type for specifying colors, can be either floating
	 * point (0 to 1) or integral (0 to -1U) ... testing both here */
	plist::RGBColor<float> fcol;
	plist::RGBColor<unsigned char> icol;
};

/* We'll define a rectangle by the lower-left point and the upper-right point.
 * Further, we'll set up an accessor to maintain this invariant. */
class Rectangle : public Shape, public virtual plist::DictionaryOf< ::Point> {
public:
	Rectangle()
		: plist::DictionaryOf< ::Point>(), Shape(), lower(), upper(),
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
	::Point lower, upper;
	
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
class Polygon : public Shape, public virtual plist::ArrayOf< ::Point> {};

// Here's some sample usage...
int main(int argc, const char** argv) {
	::Point p;
	p[0]=1; // transparent conversion from value type to plist::Primitive
	p[1]=2.5;
	p.saveFile("point.plist");
	
	Circle circ;
	circ.radius=-INFINITY;
	circ.fcol.red = 0.1337;
	circ.fcol.alpha = 0.75;
	circ.icol[0] = 13;
	circ.saveFile("circle.plist");
	circ.loadFile("circle.plist");
	circ.saveFile("circle2.plist");
	
	Rectangle r;
	r.upper=::Point(4,5); // we can assign to directly to member...
	r["lower"]=p; // ... or dynamically via operator[] lookup
	r.saveFile("rectangle.plist");
	
	// test the low/high invariant
	cout << "Original r, width: " << r.getWidth() << '\n' << r << endl;
	/* Original r, width: 3
	 * lower.0 = 1
	 * lower.1 = 2.5
	 * upper.0 = 4
	 * upper.1 = 5 */
	
	r.upper[0] = -4; // note we assign to upper.x, but it's lower.x that will be -4
	// (because of the PrimitiveListener added by Rectangle's constructor)
	cout << "Negated r, width: " << r.getWidth() << '\n' << r << endl;
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
	poly.addEntry(new ::Point(1,2));
	poly.addEntry(new ::Point(3,4));
	poly.addEntry(new ::Point(5,0));
	poly.saveFile("poly.plist");

	
	// more random tests...
	
	Dictionary outer;
	
	// Dictionary within a dictionary
	Dictionary inner;
	// This inner dictionary will be labeled "numeric" in the outer dictionary
	outer.addEntry("numeric",inner,"numeric contains some numeric values\n\tOne is a float\n\tThe other is an int\n\t...and a char as text for good measure");
	
	// These numeric tests will be stored in the inner dictionary
	Primitive<int> fooI=42;
	inner.addEntry("fooI",fooI,"This is an integer value, the other is a float");
	
	Primitive<float> fooF=M_PI;
	inner.addEntry("fooF",fooF);
	
	// chars can be treated as either a character (i.e. one letter
	// string) or a byte (i.e. 8 bit value)
	Primitive<char> fooC1='a'; // by default is a letter
	fooC1.setNumeric(false);  // can confirm this through setNumeric
	inner.addEntry("char-text",fooC1);
	
	Primitive<char> fooC2('a',true); //...can also setNumeric through constructor
	inner.addEntry("char-num",fooC2);
	
	// OK, back to add some more to the outer dictionary
	Primitive<std::string> str("Hello world!");
	outer.addEntry("test str",str,"Seen this one before?");
	
	Array mixed;
	outer.addEntry("Mixed",mixed,"A collection of mixed types of specified length");
	Primitive<int> itema=0; mixed.addEntry(itema);
	Primitive<int> itemb=1; mixed.addEntry(itemb);
	Primitive<int> itemc=2; mixed.addEntry(itemc);
	Primitive<std::string> item1="This is item 1"; mixed.addEntry(item1);
	Primitive<std::string> item2="item 2: arrays can contain mixed types"; mixed.addEntry(item2);
	Primitive<float> item3 = 123; mixed.addEntry(item3,"like this int");
	Primitive<bool> item4 = true; mixed.addEntry(item4,"or this bool");
	//mixed.addEntry("dynamic, heap-allocated entries supported too");
	
	Array vec; // there's also variable sized collections
	outer.addEntry("vector",vec,"The squares, from 0 to 5");
	for(int i=0; i<5; i++)
		vec.addValue(i*i); //now we use value types, the vector maintains the Primitive<>'s internally
	vec.addValue("can also handle mixed");
	
	Dictionary m; // named variable sized collections
	outer.addEntry("map",m,"The squares, from 0 to 5, but with keys");
	for(int i=0; i<5; i++) {
		stringstream ss;
		ss << i << "^2";
		m.addValue(ss.str(),i*i); //now we use value types, the vector maintains the Primitive<>'s internally
	}
	m.addValue("tada","mixed types -- no surprise");
	
	cout << outer;
	
	if(!outer.saveFile("virgin.plist")) {
		cerr << "Save virgin had error" << endl;
		return 1;
	}
	if(!outer.loadFile("loadtest.plist")) {
		cerr << "Load test had error" << endl;
		return 1;
	}
	str="Hello World, again!";
	if(!outer.saveFile("savetest.plist")) {
		cerr << "Save test had error" << endl;
		return 1;
	}
	
	// not actually a plist, but a custom XML schema we'll test while we're at it...
	VisionObjectEvent e(0,EventBase::activateETID);
	e.setTimeStamp(0); // ensures precisely repeatable results
	e.saveFile("vision.event");
	return 0;
}
