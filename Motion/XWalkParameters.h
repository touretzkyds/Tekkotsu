//-*-c++-*-
#ifndef INCLUDED_XWalkParameters_h_
#define INCLUDED_XWalkParameters_h_

#include "Shared/plist.h"

//! Parameters used by XWalkMC to control gait
/*! One instance of this class is a public superclass which clients can modify directly, and
 *  a second instance is stored internally by XWalkMC which allows it to smoothly transition
 *  parameter values when they are updated.  (See XWalkMC::ParameterTransition) */
class XWalkParameters : public virtual plist::Dictionary {
protected:
	static const float EPSILON; //!< limit of resolution for various computations
	
public:
	//! constructor, sets default values
	XWalkParameters()
	: plist::Dictionary(), groundPlane(4,0,false), gravityVector(3,0,false),
	offsetX(0), offsetY(0), offsetA(0), resetOnStop(true), frictionCoef(0), strideLenX(100), strideLenY(60), adaptiveLegOrder(true), rotateBodyMotion(true), transitionDuration(1500),
	bounce(), sway(), surge(), legParams(NumLegs), nonLegJoints()
	{
		init();
	}
	
	//! copy constructor, forwards to operator=
	XWalkParameters(const XWalkParameters& p) 
	: plist::Dictionary(), groundPlane(4,0,false), gravityVector(3,0,false),
	offsetX(0), offsetY(0), offsetA(0), resetOnStop(true), frictionCoef(0), strideLenX(100), strideLenY(60), adaptiveLegOrder(true), rotateBodyMotion(true), transitionDuration(1500),
	bounce(), sway(), surge(), legParams(NumLegs), nonLegJoints()
	{
		init();
		*this = p;
	}
	
	//! projects point to ground plane along gravity vector
	static void projectToGround(const fmat::Column<3>& ground, float height, const fmat::Column<3>& gravity, fmat::Column<3>& tgt);
	
	//! converts the gravity and ground configuration parameters to fmat data structures for further calculations
	/*! @a gravity will be normalized, @a ground will just be the first three terms of #groundPlane */
	void packGroundGravity(const fmat::SubVector<3>& ground, const fmat::SubVector<3> gravity) const;
	
	float getMaxXVel() const; //!< returns fastest possible foward velocity given the current parameters (mm/s)
	float getMaxYVel() const; //!< returns fastest possible sideways velocity given the current parameters (mm/s)
	float getMaxAVel() const; //!< returns fastest possible rotational velocity given the current parameters (rad/s)
	
	fmat::Column<2> computeNeutralPos(unsigned int leg) const; //!< returns center point of stride
	
	// ****** BODY PARAMETERS ******
	// *** Position ***
	
	//! equation of the ground plane relative to the base frame (in millimeters)
	/*! The equation is of the form @f$p_1x + p_2y + p_3z = p_4@f$, relative to the base frame, the first three parameters should be normalized.
	 *  See also #gravityVector. */
	plist::ArrayOf<plist::Primitive<float> > groundPlane;
	
	//! Vector indicating direction of gravity's pull, relative to the ground plane normal vector; this influences the projection of the foot positions onto the ground plane, magnitude is arbitrary
	/*! This vector is added to the normal vector to the ground plane from the body frame origin.  Thus,
	 *  leaving this vector at (0,0,0) means that the gravity vector will always point straight into the ground,
	 *  and adjusting the ground plane will change the attitude of the body without affecting foot positions.
	 *  However, adjusting this instead of the groundPlane adjusts feet positions while keeping body parallel
	 *  to the ground.  Adjusting both is necessarily if you want to keep the body Z-axis aligned with gravity
	 *  while walking uphill.  */
	plist::ArrayOf<plist::Primitive<float> > gravityVector;
	
	//! Bias the position of the body relative to the ground (in millimeters), so increasing this parameter moves the robot forward, parallel to the ground.
	/*! See also each leg's stanceWidth, which if changed en masse can provide the same adjustment parallel to the base frame instead of ground plane */
	plist::Primitive<float> offsetX;
	
	//! Bias the position of the body relative to the ground (in millimeters), so increasing this parameter moves the robot left, parallel to the ground.
	/*! See also each leg's strideBias, which if changed en masse can provide the same adjustment parallel to the base frame instead of ground plane */
	plist::Primitive<float> offsetY;
	
	//! Bias the orientation of the body relative to the ground (in radians), so increasing this parameter turns the robot counter-clockwise
	plist::Primitive<float> offsetA;
	
	//! Causes the feet to redistribute to their central positions when motion stops.
	/*! Looks better, but actually a little inefficient when you start walking again */
	plist::Primitive<bool> resetOnStop;
	
	//! Coefficient of friction with the ground (aka µ), limits the amount of non-normal force which can be applied
	plist::Primitive<float> frictionCoef;
	
	
	// *** Motion ***
	
	//! The size of step to take with each leg (all legs have the same period and travel same speed, so must have the same length of stride)
	plist::Primitive<float> strideLenX;
	
	//! The size of step to take with each leg (all legs have the same period and travel same speed, so must have the same length of stride)
	plist::Primitive<float> strideLenY;
	
	//! Whether to re-order the leg flights based on direction of motion
	plist::Primitive<bool> adaptiveLegOrder;
	
	//! Whether to rotate the sway and surge motions to match direction of motion
	plist::Primitive<bool> rotateBodyMotion;
	
	//! How much time to use getting into initial position, or when parameters change (milliseconds)
	plist::Primitive<unsigned int> transitionDuration;
	
	//! Specifies parameters for shifting the body position while in motion
	class SinusoidalParameters : public virtual plist::Dictionary {
	public:
		plist::Primitive<float> magnitude; //!< distance of motion (peak-to-center)
		plist::Primitive<float> phase; //!< offsets phase within stride (0-1)
		plist::Primitive<float> baseline; //!< offsets the median value to be oscillated about
		plist::Primitive<float> freqScale; //!< multiplies the frequency of the oscillation, so '2' would repeat the oscillation twice within a cycle
		plist::Primitive<int> legLink; //!< if a valid leg index (i.e. non-negative, less than number of legs), indicates the leg phase to use instead of global phase
		//! constructor
		SinusoidalParameters() : plist::Dictionary(), magnitude(0), phase(0), baseline(0), freqScale(2), legLink(-1) {
			addEntry("Magnitude",magnitude);
			addEntry("Phase",phase);
			addEntry("Baseline",baseline);
			addEntry("FreqScale",freqScale);
			addEntry("LegLink",legLink);
			setLoadSavePolicy(FIXED,SYNC);
		}
		inline float operator()(float globPhase, float* legPhases) {
			if(magnitude==0)
				return baseline;
			unsigned int leg = static_cast<unsigned int>(legLink);
			if(leg<NumLegs)
				return baseline-magnitude*std::cos((legPhases[leg] - phase)*freqScale*2*float(M_PI));
			else
				return baseline-magnitude*std::cos((globPhase - phase)*freqScale*2*float(M_PI));
		}
	};
	SinusoidalParameters bounce; //!< moves the body up and down
	SinusoidalParameters sway; //!< moves the body left and right
	SinusoidalParameters surge; //!< moves the body forward and back
	
	// ****** PER-LEG PARAMETERS ******
	//! Specifies parameters for each leg
	class LegParameters : public virtual plist::Dictionary {
	public:
		plist::Primitive<bool> usable; //!< if false, disables the leg from motion by the walk
		// *** Position ***
		plist::Primitive<float> stanceWidth; //!< 'y' position of the foot during forward travel, relative to body/base frame
		plist::Primitive<float> strideBias; //!< 'x' position of the foot during sideways travel, relative to 'x' position of first leg joint in body/base frame
		plist::Primitive<float> strideMargin; //!< distance to leave between this foot and the next when lowering
		// *** Motion ***
		plist::Primitive<float> flightHeight; //!< the minimum height above ground while in flight
		plist::Primitive<float> flightPhase; //!< the offset of this leg within the walk cycle (0-1)
		plist::Primitive<unsigned int> flightDuration; //!< the time to take in the air, returning a leg to the front of its stroke (ms)
		plist::Primitive<unsigned int> raiseDuration; //!< the time to take lifting a leg from the ground before flight (ms)
		plist::Primitive<unsigned int> lowerDuration; //!< the time to lower a leg back to the ground after flight (ms)
		//! constructor
		LegParameters() : plist::Dictionary(), usable(true), stanceWidth(250), strideBias(0), strideMargin(75), flightHeight(35), flightPhase(0), flightDuration(400), raiseDuration(250), lowerDuration(250) {
			addEntry("Usable",usable);
			addEntry("StanceWidth",stanceWidth);
			addEntry("StrideBias",strideBias);
			addEntry("StrideMargin",strideMargin);
			addEntry("FlightHeight",flightHeight);
			addEntry("FlightPhase",flightPhase);
			addEntry("FlightDuration",flightDuration);
			addEntry("RaiseDuration",raiseDuration);
			addEntry("LowerDuration",lowerDuration);
			setLoadSavePolicy(FIXED,SYNC);
		}
		//! returns sum of raise, flight, and lower durations
		unsigned int totalDuration() const { return raiseDuration + flightDuration + lowerDuration; }
	};
	plist::ArrayOf<LegParameters> legParams;
	
	plist::DictionaryOf<SinusoidalParameters> nonLegJoints;
	
protected:
	void init() {
		addEntry("GroundPlane",groundPlane,
				 "Specifies the ground plane relative to the base frame (in millimeters),\n"
				 "of the form: p₁·x + p₂·y + p₃·z = p₄\n"
				 "The first 3 entries should be normalized, so that the fourth is the\n"
				 "distance from the origin.");
		addEntry("GravityVector",gravityVector,
				 "Specifies an offset to be added to the GroundPlane normal vector to specify\n"
				 "the direction of gravity.  This influences the projection of the foot positions\n"
				 "onto the ground plane, magnitude is arbitrary.");
		addEntry("OffsetX",offsetX,"Bias the position of the body relative to the ground (in millimeters), so increasing this parameter moves the robot forward, parallel to the ground.");
		addEntry("OffsetY",offsetY,"Bias the position of the body relative to the ground (in millimeters), so increasing this parameter moves the robot left, parallel to the ground.");
		addEntry("OffsetA",offsetA,"Bias the orientation of the body relative to the ground (in radians), so increasing this parameter turns the robot counter-clockwise.");
		addEntry("ResetOnStop",resetOnStop,"Causes the feet to redistribute to their central positions when motion stops.");
		addEntry("FrictionCoefficient",frictionCoef,"Coefficient of friction with the ground (aka µ), limits the amount of non-normal force which can be applied");
		addEntry("StrideLengthX",strideLenX,"The size of forward (x-axis) step (mm) to take with each leg (all legs have the same period and travel same speed, so must have the same length of stride)");
		addEntry("StrideLengthY",strideLenY,"The size of sideways (y-axis) step (mm) to take with each leg (all legs have the same period and travel same speed, so must have the same length of stride)");
		addEntry("AdaptiveLegOrder",adaptiveLegOrder,"If true, re-orders the leg flights based on direction of motion");
		addEntry("RotateBodyMotion",rotateBodyMotion,"If true, rotate the sway and surge motions to match direction of motion");
		addEntry("TransitionDuration",transitionDuration,"How much time to use getting into initial position, or when parameters change (milliseconds)");
		addEntry("Bounce",bounce,"Movement up and down while walking");
		addEntry("Sway",sway,"Movement left and right while walking");
		addEntry("Surge",surge,"Movement forward and backward while walking");
		addEntry("LegParameters",legParams);
		addEntry("NonLegJoints",nonLegJoints,"Controls motion of non-leg joints, e.g. swaying arms to balance legs");
		nonLegJoints.setLoadSavePolicy(SYNC,SYNC);
		setLoadSavePolicy(FIXED,SYNC);
		groundPlane[2]=1;
		groundPlane[3]=0;
		float dur = 1.f/(NumLegs+NumWheels);
		float phase = 0;
		for(unsigned int i=NumLegs-1; i<NumLegs; i-=4) {
			if(i+1<NumLegs) {
				legParams[i+1].flightPhase=phase;
				phase+=dur;
			}
			legParams[i].flightPhase=phase;
			phase+=dur;
		}
		for(unsigned int i=NumLegs-2; i<NumLegs; i-=4) {
			legParams[i].flightPhase=phase;
			phase+=dur;
			if(i-1U<NumLegs) {
				legParams[i-1U].flightPhase=phase;
				phase+=dur;
			}
		}
	}

	float nominalPeriod() const;
	float minPeriod() const;
};

/*! @file
 * @brief Defines XWalkParameters, which provide configuration settings for XWalkMC
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
