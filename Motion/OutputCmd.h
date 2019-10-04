//-*-c++-*-
#ifndef INCLUDED_OutputCmd_h
#define INCLUDED_OutputCmd_h

//! This object holds all the information needed to control a single output
class OutputCmd {
public:

	OutputCmd() : value(0), weight(0) {} //!< Constructor
	OutputCmd(float v) : value(v), weight(1) {} //!< Constructor (provides implicit conversion from float)
	OutputCmd(float v, float w) : value(v), weight(w) {} //!< Constructor
	OutputCmd(const OutputCmd& a, const OutputCmd& b, float w) : value(a.value*w+b.value*(1-w)), weight(a.weight*w+b.value*(1-w)) {} //!< Constructor, see set(a,b,w)
	
	//! assignment from another OutputCmd (just copy everything, straightforward)
	OutputCmd& operator=(const OutputCmd& oc) { value=oc.value; weight=oc.weight; return *this; }
	//! assignment from a float, set weight to 1
	OutputCmd& operator=(float v) { value=v; weight=1; return *this; }

	inline void set(float v, float w=1) { value=v; weight=w; } //!< sets the value to @a v and weight to @a w
	inline void set(const OutputCmd& a, const OutputCmd& b, float w) { value=a.value*w+b.value*(1-w); weight=a.weight*w+b.weight*(1-w); } //!< sets the value to a weighted average of @a a and @a b (higher @a w, more @a a)
	inline void unset() { value=weight=0; } //!< sets value and weight to 0
	bool operator==(const OutputCmd& c) const { return value==c.value && weight==c.weight; } //!< tests for equality of weight and value
	bool operator!=(const OutputCmd& c) const { return value!=c.value || weight!=c.weight; } //!< tests for inequality of weight and value

	float value; //!< value of the output
	float weight; //!< weight to be used in averaging, 0 to "fall through"
	static OutputCmd unused; //!< handy sometimes for returning a reference to a 0,0 cmd
};

/*! @file
 * @brief Describes OutputCmd, holds information needed to control a single output
 * @author ejt (Creator)
 */

#endif
