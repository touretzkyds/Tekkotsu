//-*-c++-*-
#ifndef INCLUDED_IKSolver_h
#define INCLUDED_IKSolver_h

#include "Motion/KinematicJoint.h"
#include "Shared/fmat.h"
#include "Shared/FamilyFactory.h"
#include <stdexcept>

//! Provides an abstract interface to inverse kinematic solvers
class IKSolver {
public:
	//! Indicates the type of progress made by the step() family
	enum StepResult_t {
		SUCCESS, //!< goal was reached
		PROGRESS, //!< step moved closer
		LIMITS, //!< not able to progress due to joint limits
		RANGE //!< not able to progress due to target out of reach
	};
	
	struct Point;
	struct Rotation;
	
	//!@name Position Constraints
	//@{

	//! Abstract base class for position information
	/*! Subclasses may allow for some freedom in solutions -- not every solution has to be to a single point! */
	struct Position {
		virtual ~Position() {}
		//! return vector pointing in direction of decreasing error at point @a p and rotation @a r
		virtual void computeErrorGradient(const Point& p, const Rotation& r, fmat::Column<3>& de) const = 0;
	};

	//! Points allow 0 freedoms
	struct Point : public Position, public fmat::Column<3> {
		Point() : Position(), fmat::Column<3>(0.f), x(data[0]), y(data[1]), z(data[2]) {}
		Point(float xi, float yi, float zi) : Position(), fmat::Column<3>(fmat::pack(xi,yi,zi)), x(data[0]), y(data[1]), z(data[2]) {}
		Point(const Point& p) : Position(), fmat::Column<3>(p), x(data[0]), y(data[1]), z(data[2]) {}
		Point(const fmat::Column<3>& p) : Position(), fmat::Column<3>(p), x(data[0]), y(data[1]), z(data[2]) {}
		Point(const fmat::SubVector<3,const fmat::fmatReal>& p) : Position(), fmat::Column<3>(p), x(data[0]), y(data[1]), z(data[2]) {}
		Point& operator=(const Point& p) { fmat::Column<3>::operator=(p); return *this; }
		using fmat::Column<3>::operator=;
		virtual void computeErrorGradient(const Point& p, const Rotation&, fmat::Column<3>& de) const {
			de[0]=x-p.x; de[1]=y-p.y; de[2]=z-p.z;
		}
		fmat::fmatReal& x;
		fmat::fmatReal& y;
		fmat::fmatReal& z;
	};
	
	//! Lines allow 1 freedom, slide along line
	struct Line : public Position {
		//virtual void computeErrorGradient(const Point& p, const Rotation&, fmat::Column<3>& de) const {}
		float nx;
		float ny;
		float nz;
		float dx;
		float dy;
		float dz;
	};
	
	//! Planes allow 2 freedoms, move in plane
	struct Plane : public Position {
		virtual void computeErrorGradient(const Point& p, const Rotation&, fmat::Column<3>& de) const {
			float t = -( p.x*x + p.y*y + p.z*z + d );
			de[0] = t*x;
			de[1] = t*y;
			de[2] = t*z;
		}
		float x;
		float y;
		float z;
		float d;
	};
	
	//@}

	//!@name Orientation Constraints
  //@{

	//! Abstract base class for orientation information
	/*! Subclasses may allow for some freedom in solutions -- not every solution has to be to a single rotation! */
	struct Orientation {
		virtual ~Orientation() {}
		//! return quaternion indicating direction of decreasing error at point @a p and rotation @a r
		virtual void computeErrorGradient(const Point& p, const Rotation& r, fmat::Quaternion& de) const = 0;
	};
	
	//! Rotation  (here specified as a quaternion) allows 0 freedoms
	struct Rotation : public Orientation, public fmat::Quaternion {
		Rotation() : Orientation(), fmat::Quaternion() {}
		Rotation(float wi, float xi, float yi, float zi) : Orientation(), fmat::Quaternion(wi,xi,yi,zi) {}
		Rotation(const fmat::Quaternion& q) : Orientation(), fmat::Quaternion(q) {}
		
		virtual void computeErrorGradient(const Point&, const Rotation& r, fmat::Quaternion& de) const {
			de = fmat::crossProduct(r,*this);
		}

		virtual bool operator==(const Rotation &other) const { return fmat::Quaternion::operator==(other); }
	};
	
	//! Parallel allows 1 freedom (roll along z vector)
	struct Parallel : public Orientation {
		Parallel(float xi, float yi, float zi) : Orientation(), x(xi), y(yi), z(zi) {}
		virtual void computeErrorGradient(const Point&, const Rotation& r, fmat::Quaternion& de) const {
			//std::cout << "Current rotation: " << r << std::endl;
			fmat::Column<3> clv = r * Point(0,0,1);
			fmat::Column<3> res = fmat::crossProduct(clv,fmat::SubVector<3,const float>(&x));
			float ang = std::asin(res.norm());
			de = fmat::Quaternion::fromAxisAngle(res,ang);
			//std::cout << "Error rotation: " << de << std::endl;
		}
		bool operator==(const Parallel &other) const { return x==other.x && y==other.y && z==other.z; }
		float x; // the "target" unit z vector, in base coordinates
		float y;
		float z;
	};
	
	//! Cone allows 1-2 freedoms (roll, trade off pitch vs. yaw)
	struct Cone : public Orientation {
		float lx;
		float ly;
		float lz;
		float tx;
		float ty;
		float tz;
		float a;
	};
	
	//@}

	//! destructor
	virtual ~IKSolver() {}
	
	//! Solve to get an 'effector' point @a pEff (relative to link following @a j) to a solution of @a pTgt (or at least a local minimum)
  /*! @param pEff The point on the effector that should be brought to the target.
    Typically [0,0,0] if the effector is GripperFrame.
    @param j The kinematic joint at the end of the chain (e.g., GripperFrame)
    @param pTgt The Position constraint that pEff should satisfy, i.e., the target location
  */
	virtual bool solve(const Point& pEff, KinematicJoint& j, const Position& pTgt) const {
		Rotation curOri(j.getQuaternion());
		return solve(pEff,curOri,j,pTgt,1,curOri,0);
	}
	//! Solve to get an 'effector' orientation @a oriEff (relative to link following @a j) to a solution of @a oriTgt (or at least a local minimum)
  /*! @param oriEff Rotation of the effector's reference frame; used to align some effector axis with the target's Orientation axes. For example, if @a oriTgt is a Parallel constraint, the effector will be free to rotate about the target's z-axis. If we want the effector to rotate about its y-axis, we use @a oriEff to rotate the effector y-axis to the target's z-axis.
    @param j The kinematic joint at the end of the chain (e.g., GripperFrame)
    @param oriTgt The Orientation constraint that pEff should satisfy at the target location
  */
	virtual bool solve(const Rotation& oriEff, KinematicJoint& j, const Orientation& oriTgt) const {
		Point curPos(j.getPosition());
		return solve(curPos,oriEff,j,curPos,0,oriTgt,1);
	}
	
	//! Solve to get an 'effector' (@a pEff, @a oriEff, relative to link following @a j) to a solution of @a pTgt, @a oriTgt (or at least a local minimum)
	/*! @param pEff The point on the effector that should be brought to the target.
	  Typically [0,0,0] if the effector is GripperFrame.
	  @param oriEff Rotation of the effector's reference frame; used to align some effector axis with the target's Orientation axes. For example, if @a oriTgt is a Parallel constraint, the effector will be free to rotate about the target's z-axis. If we want the effector to rotate about its y-axis, we use @a oriEff to rotate the effector y-axis to the target's z-axis.
	  @param j The kinematic joint at the end of the chain (e.g., GripperFrame)
	  @param pTgt The Position constraint that pEff should satisfy, i.e., the target location
	  @param oriTgt The Orientation constraint that pEff should satisfy at the target location
	  @param posPri,oriPri Relative priorities (weightings) of position and orientation solutions in case they are mutually exclusive
	*/
	virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const=0;
	
	//! Move an 'effector' point @a pEff (relative to link following @a j) towards a solution of @a pTgt (or at least a local minimum)
	/*! @a pDist specifies the maximum distance to move */
	virtual StepResult_t step(const Point& pEff, KinematicJoint& j, const Position& pTgt, float pDist) const {
		Rotation curOri(j.getQuaternion());
		return step(pEff,curOri,j,pTgt,1,pDist,curOri,0,0);
	}
	//! Move an 'effector' orientation @a oriEff (relative to link following @a j) towards a solution of @a oriTgt (or at least a local minimum)
	/*! @a oriDist specifies the maximum distance to move in radians */
	virtual StepResult_t step(const Rotation& oriEff, KinematicJoint& j, const Orientation& oriTgt, float oriDist) const {
		Point curPos(j.getPosition());
		return step(curPos,oriEff,j,curPos,0,0,oriTgt,1,oriDist);
	}
	
	//! Move an 'effector' (@a pEff, @a oriEff, relative to link following @a j) towards a solution of @a pTgt, @a oriTgt (or at least a local minimum)
	/*! @a pDist and @a oriDist specifies the maximum distance to move towards each solution;
	  *  @a posPri and @a oriPri specify relative weighting of each solution in case they are mutually exclusive */
	virtual StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const=0;
	
	//! shorthand for type registry to allow dynamically instantiating IKSolvers based on the name found in configuration files
	typedef FamilyFactory<IKSolver,std::string,Factory0Arg<IKSolver> > registry_t;
	static registry_t& getRegistry() { static registry_t registry; return registry; }
};

/*! @file
 * @brief Defines IKSolver interface, which provides an abstract interface to inverse kinematic solvers
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
