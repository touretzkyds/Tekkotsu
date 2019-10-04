#ifndef INCLUDED_Measures_h_
#define INCLUDED_Measures_h_

#include "Shared/fmatCore.h"

#include <cmath>

//! Representation of a plane as ax + by + cz = d; used for representing the ground plane
class PlaneEquation {
private:
	fmat::Column<3> direction;  //!< Unit vector normal to the plane
	float displacement; //!< distance along normal from origin
public:
	PlaneEquation(const fmat::Column<3> &dir, float disp); //!< constructor, will normalize @a dir and apply to @a disp as well to maintain relative relationship
	PlaneEquation(float a, float b, float c, float d); //!< constructor, pass plane equation representing ax + by + cz = d
	PlaneEquation() : direction(fmat::pack(0.f, 0.f, 1.f)), displacement(0) {} //!< constructor, assumes upward z normal
	const fmat::Column<3>& getDirection() const { return direction; } //!< returns #direction
	void setDirection(const fmat::Column<3>& dir) { direction = dir / dir.norm(); } //!< normalizes and assigns #direction (normalization here does not affect #displacement, unlike constructor)
	
	float getDisplacement() const { return displacement; } //!< returns #displacement
	void setDisplacement(float disp) { displacement = disp; } //!< assigns #displacement

	float getZsign() const { return direction[2] < 0 ? -1.f : 1.f; } //!< returns sign on #displacement
	float getDsign() const { return displacement < 0 ? -1.f : 1.f; } //!< returns sign on #displacement
	
	//! assign to a four-element vector via operator[], with displacement in the last element ([3])
	template<class T> void exportTo(T& x) const { x[0]=direction[0]; x[1]=direction[1]; x[2]=direction[2]; x[3]=displacement; }
	//! pull values from a four-element vector via operator[], with displacement in the last element ([3])
	template<class T> void importFrom(const T& x) { direction[0]=x[0]; direction[1]=x[1]; direction[2]=x[2]; displacement=x[3]; }
};

std::ostream& operator<<(std::ostream &os, const PlaneEquation& p);

typedef float coordinate_t; //!< type used for positional coordinates
typedef float orientation_t; //!< type used for orientation values (0 to Pi)
typedef float direction_t; //!< type used for direction values (0 to 2*Pi)

const direction_t Pi=static_cast<direction_t>(M_PI); //!< shorthand for ::M_PI from math.h
const direction_t TwoPi=static_cast<direction_t>(2*M_PI); //!< shorthand for 2*M_PI 

typedef coordinate_t Slope; //!< type used for ratio of coordinate offsets
const Slope BIG_SLOPE=static_cast<Slope>(5000); //!< slopes larger than this are considered vertical, or in other words, infinite slopes are rounded to this

//! Circular arithmetic on angles between 0 and pi (180 degrees)
class AngPi {
	friend AngPi angdist(AngPi const &arg1, AngPi const &arg2);
public:
	AngPi(void) : value(0) {}; //!< constructor, #value defaults to 0
	AngPi(orientation_t const &v) : value(v) { normalize(); } //!< conversion operator allows implicit construction from primitive
	
	AngPi operator+(AngPi const &arg) const { return AngPi(value+arg.value); };
	AngPi operator-(AngPi const &arg) const { return AngPi(value-arg.value); };
	AngPi operator*(orientation_t const &arg) const { return AngPi(value*arg); };
	AngPi operator/(orientation_t const &arg) const { return AngPi(value/arg); };
	
	AngPi& operator=(AngPi const &arg) { value = arg.value; return(*this); };
	AngPi& operator=(orientation_t const &arg) { value = arg; normalize(); return(*this); };
	AngPi& operator+=(orientation_t const &arg) { value += arg; normalize(); return(*this); };
	AngPi& operator-=(orientation_t const &arg) { value -= arg; normalize(); return(*this); };
	AngPi& operator*=(orientation_t const &arg) { value *= arg; normalize(); return(*this); };
	AngPi& operator/=(orientation_t const &arg) { value /= arg; normalize(); return(*this); };
	
	operator orientation_t() const { return value; }; //!< conversion operator for going back to the primitive type
	
protected:
	void normalize(); //!< modifies #value to put it back in range
	orientation_t value; //!< holds the angle, should be kept normalized at all times
};

//! Angular distance: value is between 0 and pi/2
AngPi angdist(AngPi const &arg1, AngPi const &arg2);

//! Circular arithmetic on angles between 0 and two pi (360 degrees)
class AngTwoPi {
	friend AngPi angdist(AngTwoPi const &arg1, AngTwoPi const &arg2);
public:
	AngTwoPi(void) : value(0) {}; //!< constructor, #value defaults to 0
	AngTwoPi(direction_t const &v) : value(v) { normalize(); } //!< conversion operator allows implicit construction from primitive
	
	AngTwoPi operator+(AngTwoPi const &arg) const { return AngTwoPi(value+arg.value); };
	AngTwoPi operator-(AngTwoPi const &arg) const { return AngTwoPi(value-arg.value); };
	AngTwoPi operator*(direction_t const &arg) const { return AngTwoPi(value*arg); };
	AngTwoPi operator/(direction_t const &arg) const { return AngTwoPi(value/arg); };
	
	AngTwoPi& operator=(AngTwoPi const &arg) { value = arg.value; return(*this); };
	AngTwoPi& operator=(direction_t const &arg) { value = arg; normalize(); return(*this); };
	AngTwoPi& operator+=(direction_t const &arg) { value += arg; normalize(); return(*this); };
	AngTwoPi& operator-=(direction_t const &arg) { value -= arg; normalize(); return(*this); };
	AngTwoPi& operator*=(direction_t const &arg) { value *= arg; normalize(); return(*this); };
	AngTwoPi& operator/=(direction_t const &arg) { value /= arg; normalize(); return(*this); };
	
	operator direction_t() const { return value; }; //!< conversion operator for going back to the primitive type
	
protected:
	void normalize(); //!< modifies #value to put it back in range
	direction_t value; //!< holds the angle, should be kept normalized at all times
};

//! Angular distance: value is between 0 and pi
AngPi angdist(AngTwoPi const &arg1, AngTwoPi const &arg2);

//! Circular arithmetic on angles between -pi and pi (360 degrees)
class AngSignPi {
	friend AngPi angdist(AngSignPi const &arg1, AngSignPi const &arg2);
public:
	AngSignPi(void) : value(0) {}; //!< constructor, #value defaults to 0
	AngSignPi(direction_t const &v) : value(v) { normalize(); } //!< conversion operator allows implicit construction from primitive
	
	AngSignPi operator+(AngSignPi const &arg) const { return AngSignPi(value+arg.value); };
	AngSignPi operator-(AngSignPi const &arg) const { return AngSignPi(value-arg.value); };
	AngSignPi operator*(direction_t const &arg) const { return AngSignPi(value*arg); };
	AngSignPi operator/(direction_t const &arg) const { return AngSignPi(value/arg); };
	
	AngSignPi& operator=(AngSignPi const &arg) { value = arg.value; return(*this); };
	AngSignPi& operator=(direction_t const &arg) { value = arg; normalize(); return(*this); };
	AngSignPi& operator+=(direction_t const &arg) { value += arg; normalize(); return(*this); };
	AngSignPi& operator-=(direction_t const &arg) { value -= arg; normalize(); return(*this); };
	AngSignPi& operator*=(direction_t const &arg) { value *= arg; normalize(); return(*this); };
	AngSignPi& operator/=(direction_t const &arg) { value /= arg; normalize(); return(*this); };
	
	operator direction_t() const { return value; }; //!< conversion operator for going back to the primitive type

protected:
	void normalize(); //!< modifies #value to put it back in range
	direction_t value; //!< holds the angle, should be kept normalized at all times
};

//! Angular distance: value is between 0 and pi
AngPi angdist(AngSignPi const &arg1, AngSignPi const &arg2);

//! Turn with an explicit direction:  angles between -2*pi and 2*pi (+/- 360 degrees)
class AngSignTwoPi {
public:
	AngSignTwoPi(void) : value(0) {}; //!< constructor, #value defaults to 0
	AngSignTwoPi(direction_t const &v) : value(v) { normalize(); } //!< conversion operator allows implicit construction from primitive
	
	AngSignTwoPi operator+(AngSignTwoPi const &arg) const { return AngSignTwoPi(value+arg.value); };
	AngSignTwoPi operator-(AngSignTwoPi const &arg) const { return AngSignTwoPi(value-arg.value); };
	AngSignTwoPi operator*(direction_t const &arg) const { return AngSignTwoPi(value*arg); };
	AngSignTwoPi operator/(direction_t const &arg) const { return AngSignTwoPi(value/arg); };
	
	AngSignTwoPi& operator=(AngSignTwoPi const &arg) { value = arg.value; return(*this); };
	AngSignTwoPi& operator=(direction_t const &arg) { value = arg; normalize(); return(*this); };
	AngSignTwoPi& operator+=(direction_t const &arg) { value += arg; normalize(); return(*this); };
	AngSignTwoPi& operator-=(direction_t const &arg) { value -= arg; normalize(); return(*this); };
	AngSignTwoPi& operator*=(direction_t const &arg) { value *= arg; normalize(); return(*this); };
	AngSignTwoPi& operator/=(direction_t const &arg) { value /= arg; normalize(); return(*this); };
	
	operator direction_t() const { return value; }; //!< conversion operator for going back to the primitive type

protected:
	void normalize(); //!< modifies #value to put it back in range
	direction_t value; //!< holds the angle, should be kept normalized at all times
};

#endif
