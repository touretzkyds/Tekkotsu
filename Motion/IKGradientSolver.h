//-*-c++-*-
#ifndef INCLUDED_IKGradientSolver_h_
#define INCLUDED_IKGradientSolver_h_

#include "IKSolver.h"

//! Performs gradient descent on the joints to find a solution
/*! This is intended as a generic fallback for IK requests that cannot
 *  be handled analytically. */
class IKGradientSolver : public IKSolver {
public:
  //! constructor
  IKGradientSolver(unsigned int iter=75, float posTolerance=0.5f, float oriTolerance=.001f) :
    IKSolver(), PTOL(posTolerance), OTOL(oriTolerance), QTOL(oriTolerance/50), MAX_ITER(iter) {}

  //! constructor
  IKGradientSolver(unsigned int iter, float posTolerance, float oriTolerance, float qTolerance) : 
    IKSolver(), PTOL(posTolerance), OTOL(oriTolerance), QTOL(qTolerance), MAX_ITER(iter) {}
	
  virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, 
		     const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const;

  virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, 
         const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri, unsigned int root) const;

  using IKSolver::solve;
	
  virtual IKSolver::StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
				      const Position& pTgt, float pDist, float posPri, 
				      const Orientation& oriTgt, float oriDist, float oriPri) const {
    return step(pEff,oriEff,j,pTgt,pDist,posPri,oriTgt,oriDist,oriPri,true);
  }

  virtual IKSolver::StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
				      const Position& pTgt, float pDist, float posPri, 
				      const Orientation& oriTgt, float oriDist, float oriPri,
				      bool successInReach) const;
  virtual IKSolver::StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j,
              const Position& pTgt, float pDist, float posPri, 
              const Orientation& oriTgt, float oriDist, float oriPri,
              bool successInReach, unsigned int root) const;
  using IKSolver::step;
	
protected:
  const float PTOL; //!< position tolerance
  const float OTOL; //!< orientation tolerance
  const float QTOL; //!< joint angle tolerance
  const unsigned int MAX_ITER; //!< maximum number of iterations to attempt
private:
  //! holds the class name, set via registration with the DeviceDriver registry
  static const std::string autoRegisterIKGradientSolver;
  //! since this is the default solver, also register as "" (empty name)
  static const std::string autoRegisterDefaultIKSolver;
};

/*! @file
 * @brief Describes IKGradientSolver, which performs gradient descent on the joints to find a solution
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
