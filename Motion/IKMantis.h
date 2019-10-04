//-*-c++-*-
#ifndef INCLUDED_IKMantis_h_
#define INCLUDED_IKMantis_h_

#include "Motion/IKSolver.h"

#if defined(TGT_IS_MANTIS) && defined(TGT_HAS_HEAD)


//! Kinematics solver for the 3-DOF Mantis Head
class IKMantis : public IKSolver {
public:     
//  static const float EPSILON;
    //! constructor
    IKMantis();

    using IKSolver::solve;
    using IKSolver::step;
    virtual bool solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const;
    virtual StepResult_t step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const;               

protected:
    virtual bool solveHead(KinematicJoint& j, const Position& pTgt) const;
    virtual bool solvePosLeg(KinematicJoint& j, const Position& pTgt) const;       
    virtual bool solveFrontLeg(KinematicJoint& j, const Position& pTgt) const; 
private:
    //! holds the class name, set via registration with the DeviceDriver registry
    static const std::string autoRegisterIKMantis;
//  fmat::Transform baseToHead;
//  plist::Angle qOffset;
//  plist::Angle cameraOffset; 
};
#endif
#endif

