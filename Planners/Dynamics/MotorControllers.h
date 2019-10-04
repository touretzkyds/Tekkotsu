//-*-c++-*-
#ifndef INCLUDED_MotorControllers_h_
#define INCLUDED_MotorControllers_h_

#include "Shared/plist.h"
#include "Motion/KinematicJoint.h"

#include <set>
#ifdef __APPLE__
#include <BulletDynamics/btBulletDynamicsCommon.h>
#else
#include <btBulletDynamicsCommon.h>
#endif

class ConstraintInterface {
public:
	virtual ~ConstraintInterface() {}
	virtual float getValue(float dt)=0;
	virtual void applyResponse(float dt, float x)=0;
};

class HingePositionInterface : public ConstraintInterface {
public:
	explicit HingePositionInterface(btHingeConstraint& hingeConstraint) : ConstraintInterface(), hinge(hingeConstraint), lastResponse(0) {}
	virtual float getValue(float /*dt*/) { return hinge.getHingeAngle(); }
	virtual void applyResponse(float dt, float x) {
		if(x==0)
			return;
		x*=dt;
		btRigidBody& bodyA = hinge.getRigidBodyA();
		btRigidBody& bodyB = hinge.getRigidBodyB();
		btVector3 torque =  bodyA.getCenterOfMassTransform().getBasis() * hinge.getAFrame().getBasis().getColumn(2) * x;
		bodyA.applyTorqueImpulse(torque);
		bodyB.applyTorqueImpulse(-torque);
		float diff = std::abs(lastResponse-x);
		if(diff>0) {
			// bullet will "deactivate" objects if they don't move for a while, thus...
			float den = std::max(std::abs(lastResponse),std::abs(x));
			if(diff/den >= .01f) { // if our torque changes significantly (here, by more than 1% than last trigger)
				// then tell bullet to reactivate the bodies (or keep them active if already)
				bodyA.activate();
				bodyB.activate();
				lastResponse=x; // only store when we actually activate, this is what we want to test against to handle slow drifts
			}
		}
	}
	btHingeConstraint& hinge;
	float lastResponse;
};

class HingeVelocityInterface : public HingePositionInterface {
public:
	explicit HingeVelocityInterface(btHingeConstraint& hingeConstraint) : HingePositionInterface(hingeConstraint), lastValue(0) {}
	float getValue(float dt) {
		if(dt==0)
			return lastValue;
		const float q = hinge.getHingeAngle();
		const float rot = normalizeAngle(q - lastValue);
		//std::cout << "rot " << rot << " from " << lastValue << " to " << q << " over " << dt << std::endl;
		lastValue=q;
		return rot / dt;
	}
	static inline float normalizeAngle(float value) {
		if ( value < -(float)M_PI ) 
			return value + 2*(float)M_PI;
		else if ( value > (float)M_PI)
			return value - 2*(float)M_PI;
		else
			return value;
	}		
	float lastValue;
};

class HingeAngularMotorInterface : public HingeVelocityInterface {
public:
	explicit HingeAngularMotorInterface(btHingeConstraint& hingeConstraint) : HingeVelocityInterface(hingeConstraint) {}
	void applyResponse(float /*dt*/, float x) {
		if(x!=lastResponse) {
			hinge.enableAngularMotor(true,x,10);
			hinge.getRigidBodyA().activate();
			hinge.getRigidBodyB().activate();
			lastResponse=x;
		}
	}
};

class GenericAngularForceInterface : public ConstraintInterface {
public:
	explicit GenericAngularForceInterface(btGeneric6DofConstraint& genericConstraint)
	: ConstraintInterface(), constraint(genericConstraint), lastResponse(0) { lastResponse=-1; applyResponse(0,0); }
	
	virtual float getValue(float /*dt*/) {
		const btRigidBody& bodyA = constraint.getRigidBodyA();
		const btRigidBody& bodyB = constraint.getRigidBodyB();
		const btVector3 refAxis0  = bodyA.getCenterOfMassTransform().getBasis() * constraint.getFrameOffsetA().getBasis().getColumn(0);
		const btVector3 refAxis1  = bodyA.getCenterOfMassTransform().getBasis() * constraint.getFrameOffsetA().getBasis().getColumn(1);
		const btVector3 swingAxis = bodyB.getCenterOfMassTransform().getBasis() * constraint.getFrameOffsetB().getBasis().getColumn(0);
		return std::atan2(swingAxis.dot(refAxis1), swingAxis.dot(refAxis0));
	}
	
	virtual void applyResponse(float dt, float x) {
		if(x==0)
			return;
		x*=dt;
		btRigidBody& bodyA = constraint.getRigidBodyA();
		btRigidBody& bodyB = constraint.getRigidBodyB();
		btVector3 torque =  bodyA.getCenterOfMassTransform().getBasis() * constraint.getFrameOffsetA().getBasis().getColumn(2) * x;
		bodyA.applyTorqueImpulse(-torque);
		bodyB.applyTorqueImpulse(torque);
		/*float diff = std::abs(lastResponse-x);
		if(diff>0) {
			// bullet will "deactivate" objects if they don't move for a while, thus...
			float den = std::max(std::abs(lastResponse),std::abs(x));
			if(diff/den >= .01f) { // if our torque changes significantly (here, by more than 1% than last trigger)
				// then tell bullet to reactivate the bodies (or keep them active if already)
				bodyA.activate();
				bodyB.activate();
				lastResponse=x; // only store when we actually activate, this is what we want to test against to handle slow drifts
			}
		}*/
		bodyA.activate();
		bodyB.activate();
		lastResponse=x;
	}
	btGeneric6DofConstraint& constraint;
	float lastResponse;
};

class GenericAngularMotorInterface : public GenericAngularForceInterface {
public:
	explicit GenericAngularMotorInterface(btGeneric6DofConstraint& genericConstraint)
	: GenericAngularForceInterface(genericConstraint) { lastResponse=-1; applyResponse(0,0); }
	
	void applyResponse(float /*dt*/, float x) {
		if(x!=lastResponse) {
			constraint.getRotationalLimitMotor(2)->m_enableMotor = true;
			constraint.getRotationalLimitMotor(2)->m_targetVelocity = -x;
			constraint.getRotationalLimitMotor(2)->m_maxMotorForce = 10;
			constraint.getRigidBodyA().activate();
			constraint.getRigidBodyB().activate();
			lastResponse=x;
		} else if(x!=0) {
			constraint.getRigidBodyA().activate();
			constraint.getRigidBodyB().activate();
		}
	}
};

class GenericAngularPositionInterface : public GenericAngularForceInterface {
public:
	explicit GenericAngularPositionInterface(btGeneric6DofConstraint& genericConstraint)
	: GenericAngularForceInterface(genericConstraint) { lastResponse=-1; applyResponse(0,0); }
	
	void applyResponse(float /*dt*/, float x) {
		constraint.setAngularLowerLimit(btVector3(0,0,-x));
		constraint.setAngularUpperLimit(btVector3(0,0,-x));
		constraint.getRigidBodyA().activate();
		constraint.getRigidBodyB().activate();
	}
};


//! Sync entries here with those in KinematicJoint::ControllerInfo
class MotorController : virtual public plist::Dictionary {
public:
	MotorController(const plist::Primitive<float>& tgt_, ConstraintInterface& c)
		: plist::Dictionary(), tgt(tgt_), constraint(c),
		velocity(false), forceControl(false)
	{
		addEntry("Velocity",velocity,"Adjusts interpretation of feedback and application of friction, false implies position control");
		addEntry("ForceControl",forceControl,"If true, simulation will use force control to move the joint rather than using position constraints.  Grippers should set this to true for more realistic object interaction.");
	}
	virtual ~MotorController() {}

	virtual void updateControllerResponse(float dt)=0;
	
	const plist::Primitive<float>& tgt;
	ConstraintInterface& constraint;
	
	plist::Primitive<bool> velocity;
	plist::Primitive<bool> forceControl; //!< If true, simulation will use force control to move the joint rather than using position constraints.  Grippers should set this to true for more realistic object interaction.
};


class LinearMotorController : virtual public plist::Dictionary, public MotorController {
public:
	LinearMotorController(const plist::Primitive<float>& tgt_, ConstraintInterface& c, float sc=1)
		: plist::Dictionary(), MotorController(tgt_,c), scale(sc)
	{
		addEntry("Scale",scale,"Scales the target domain (e.g. convert wheel rim dist/s to axle rad/s)");
	}
	virtual void updateControllerResponse(float dt) {
		constraint.applyResponse(dt, tgt*scale);
	}
	plist::Primitive<float> scale;
};

class ProportionalMotorController : virtual public plist::Dictionary, public MotorController {
public:
	ProportionalMotorController(const plist::Primitive<float>& tgt_, ConstraintInterface& c, float p_=1)
	: plist::Dictionary(), MotorController(tgt_,c), p(p_)
	{
		addEntry("P",p,"Proportional control parameter (scales I and D response as well)");
	}
	virtual void updateControllerResponse(float dt);
	plist::Primitive<float> p;
};

class PIDMotorController : virtual public plist::Dictionary, public MotorController {
public:
	PIDMotorController(const plist::Primitive<float>& tgt_, ConstraintInterface& c)
		: plist::Dictionary(), MotorController(tgt_,c),
		scale(1), p(0), i(0), d(0), derrGamma(0), linearff(0), punch(0),
		minResponse(0), maxResponse(-1), friction(0), verbose(),
		lastX(0), lastErr(0), sumErr(0), avgDerr(0)
	{
		addEntry("Scale",scale,"Scales the target domain (e.g. convert wheel rim dist/s to axle rad/s)");
		addEntry("P",p,"Proportional control parameter (scales I and D response as well)");
		addEntry("I",i,"Integrative control parameter");
		addEntry("D",d,"Derivative control parameter");
		addEntry("DerrGamma",derrGamma,"Applies exponential smoothing to derivative control, use '0' to disable smoothing");
		addEntry("LinearFF",linearff,"Linear feed-forward term, to add scaled target value directly to response (good for velocity control)");
		addEntry("Punch",punch,"Adds a constant feed-forward term if response exceeds MinResponse");
		addEntry("MinResponse",minResponse,"Subtracted from response and clips to 0, simulates static friction");
		addEntry("MaxResponse",maxResponse,"Caps maximum response, -1 to disable");
		addEntry("Friction",friction,"Subtracts a portion of constraint value or derivative (depending on Velocity) from response");
		addEntry("Verbose",verbose,"Set to non-empty string to enable debugging outputs, handy for tuning; value is prepended to outputs");
		setLoadSavePolicy(FIXED,SYNC);
		updateControllerResponse(0);
	}
	
	virtual void updateControllerResponse(float dt);
	
	plist::Primitive<float> scale;
	plist::Primitive<float> p;
	plist::Primitive<float> i;
	plist::Primitive<float> d;
	plist::Primitive<float> derrGamma;
	plist::Primitive<float> linearff;
	plist::Primitive<float> punch;
	plist::Primitive<float> minResponse;
	plist::Primitive<float> maxResponse;
	plist::Primitive<float> friction;
	plist::Primitive<std::string> verbose;
	
	float lastX;
	float lastErr;
	float sumErr;
	float avgDerr;
};

/*! @file
 * @brief 
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
