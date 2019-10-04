//-*-c++-*-
#ifndef INCLUDED_OutputPID_h
#define INCLUDED_OutputPID_h

//! This object holds all the information needed to control a single output
class OutputPID {
public:
	OutputPID() : weight(0) { pid[0]=pid[1]=pid[2]=0; } //!< Constructor
	OutputPID(const float p[3]) : weight(1) { set_pid(p); } //!< Constructor, allows non-explicit conversion
	OutputPID(const float p[3], float w) : weight(w) {set_pid(p);} //!< Constructor
	OutputPID(const float p, const float i, const float d) : weight(1) {set_pid(p,i,d);} //!< Constructor
	OutputPID(const float p, const float i, const float d, float w) : weight(w) {set_pid(p,i,d);} //!< Constructor
	OutputPID(const OutputPID& a, const OutputPID& b, float w) : weight(0) { set(a,b,w); } //!< Constructor, see set(a,b,w)

	inline void set(const float p[3], float w=1) { set_pid(p); weight=w; } //!< sets the value to @a v and weight to @a w
	inline void set(const float p, const float i, const float d, float w=1) { set_pid(p,i,d); weight=w; } //!< sets the value to @a v and weight to @a w
	
	//! sets the value to a weighted average of @a a and @a b (higher @a w, more @a a)
	inline void set(const OutputPID& a, const OutputPID& b, float w) {
		pid[0]=a.pid[0]*w+b.pid[0]*(1-w);
		pid[1]=a.pid[1]*w+b.pid[1]*(1-w);
		pid[2]=a.pid[2]*w+b.pid[2]*(1-w);
		weight=a.weight*w+b.weight*(1-w); 
	} 
	inline void unset() { weight=0; } //!< sets value and weight to 0

	float pid[3]; //!< pid value of the output
	float weight; //!< weight to be used in averaging, 0 to "fall through"

protected:
	 //! handy utility function, sets #pid[0:2] to @a p[0:2]
	inline void set_pid(const float p[3]) {
		pid[0]=p[0];
		pid[1]=p[1];
		pid[2]=p[2];
	}
	 //! handy utility function, sets #pid[0:2] to [@a p,@a i,@a d]
	inline void set_pid(const float p, const float i, const float d) {
		pid[0]=p;
		pid[1]=i;
		pid[2]=d;
	}
};

/*! @file
 * @brief Describes OutputPID, holds information needed to control a single output
 * @author ejt (Creator)
 */

#endif
