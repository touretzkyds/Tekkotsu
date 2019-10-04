#ifndef _CBRACKET_GRASPER_PREDICATE_
#define _CBRACKET_GRASPER_PREDICATE_

#include "Planners/Manipulation/ShapeSpacePlanner2DR.h"
#include "Motion/Kinematics.h"
#include "Motion/IKSolver.h"
#include "Shared/Measures.h"

template <size_t N>
class CBracketGrasperPredicate : public AdmissibilityPredicate<RRTNode2DR<N> > {
public:
  typedef RRTNode2DR<N> NodeType_t;
  typedef typename NodeType_t::NodeValue_t NodeValue_t;
  
  KinematicJoint *gripperFrameKJ, *tmpKJ, *joints[N];
  typename NodeType_t::CollisionChecker *cc;
  fmat::Column<3> fromPT, toPT, goalPT;
  float fromOri, toOri, dir;
  
public:
  //! constructor
  CBracketGrasperPredicate(): gripperFrameKJ(), tmpKJ(), joints(), cc(), fromPT(), toPT(), goalPT(), fromOri(), toOri(), dir() {
    gripperFrameKJ = kine->getKinematicJoint(GripperFrameOffset)->cloneBranch();
    if(gripperFrameKJ != NULL) {
      tmpKJ = gripperFrameKJ->getRoot();
      tmpKJ->buildMobileChildMap(joints, ArmOffset, N);
    }
    GET_SHAPE(worldBounds, PolygonData, VRmixin::worldShS);
    cc = new typename NodeType_t::CollisionChecker(VRmixin::worldShS, worldBounds, 10, GripperFrameOffset);
  }
  
  //! destructor
  ~CBracketGrasperPredicate() { delete gripperFrameKJ->getRoot(); delete cc; }
  
  //! admissibility check
  virtual bool admissible (NodeValue_t q, std::vector<NodeType_t>& tree, unsigned int parent) {
    const float range = 40 * float(M_PI/180);
    NodeType_t qFrom = tree[parent];
	
    fromPT = gripperPosition(qFrom.q);
    toPT = gripperPosition(q);
    NodeValue_t qOld = q;
    goalPT = toPT;
    
    fromOri = stateOrien(qFrom.q);
    toOri = stateOrien(q);
    
    float aDist, nOri;
    //dir = (forward) ? atan2(toPT[1]-fromPT[1], toPT[0]-fromPT[0]) : atan2(fromPT[1]-toPT[1], fromPT[0]-toPT[0]);
    //aDist = (forward) ? angdist(AngTwoPi(fromOri), AngTwoPi(dir)) : angdist(AngTwoPi(toOri), AngTwoPi(dir));
    dir = atan2(toPT[1]-fromPT[1], toPT[0]-fromPT[0]);
    aDist = angdist(AngTwoPi(fromOri), AngTwoPi(dir));
    
    //return aDist < range;
    // THE FOLLOWING CODE ADDS A NEW NODE WITH GOOD ORIENTATION. MAYBE USE IT.
    if( aDist > range ) {
      nOri = newOri(fromPT, goalPT, fromOri);
      bool success = gripperFrameKJ->getIK().solve(fmat::Column<3>(), IKSolver::Rotation(fmat::Quaternion()),
                                                   *gripperFrameKJ,
                                                   IKSolver::Point(fmat::pack(fromPT[0], fromPT[1], 0)), 0,
                                                   IKSolver::Rotation(fmat::Quaternion::aboutZ(nOri)), 1);
      if (!success) return false;
      for(unsigned j = 0; j < N; j++)
        q[j] = joints[j]->getQ();
      if (cc->collides(q)) return false;
    }
    NodeType_t qNode(q, parent);
    if (qNode.distance(qOld) < qFrom.distance(qOld))
      tree.push_back(qNode);
    return false;
  }
  
  //! generate new orientation
  float newOri(fmat::Column<3>& from, fmat::Column<3>& to, float fOri) {
    const float change = 5 * float(M_PI/180);
    float diff = fOri - atan2(to[1]-from[1], to[0]-from[0]);
    return AngSignPi( fOri + (fabsf(diff) < (float)M_PI ? 1 : -1) * ((diff < 0) ? 1 : -1) * change);
  }
  
  fmat::Column<3> gripperPosition(const NodeValue_t& q) {
    for(unsigned int j = 0; j < N; j++) { joints[j]->setQ(q[j]); }
    return gripperFrameKJ->getWorldPosition();
  }
  
  float stateOrien(const NodeValue_t& s) {
    float ori = 0;
    for(unsigned int j = 0; j < N; j++) { ori += s[j]; }
    return AngSignPi(ori);
  }
  
private:
  CBracketGrasperPredicate& operator=(const CBracketGrasperPredicate &mp);
  CBracketGrasperPredicate(const CBracketGrasperPredicate& mp);
};

#endif
