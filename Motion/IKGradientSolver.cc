#include "IKGradientSolver.h"
#include "Shared/fmat.h"
#include <cmath>
#include <valarray>

using namespace std;

const std::string IKGradientSolver::autoRegisterIKGradientSolver = IKSolver::getRegistry().registerType<IKGradientSolver>("IKGradientSolver");
const std::string IKGradientSolver::autoRegisterDefaultIKSolver = IKSolver::getRegistry().registerType<IKGradientSolver>("");

bool IKGradientSolver::solve(const Point& pEff, const Rotation& oriEff, 
			     KinematicJoint& j, const Position& pTgt, float posPri, 
			     const Orientation& oriTgt, float oriPri) const {
  if (posPri==0 && oriPri==0)
    return true;

  posPri=std::max(0.f,posPri);
  oriPri=std::max(0.f,oriPri);
	
  KinematicJoint* firstMobileJoint = NULL;
  for(KinematicJoint *ancestor = &j; ancestor!=NULL; ancestor = ancestor->getParent())
    if (ancestor->isMobile())
      firstMobileJoint = ancestor;
  if (firstMobileJoint==NULL)
    return false;
	
  // choose pDist by heuristic: proportional to distance between effector and base frame
  fmat::Transform Tj = j.getT(*firstMobileJoint);
  fmat::Column<3> pEffFMJ(Tj*pEff);
  fmat::fmatReal pDist = pEffFMJ.norm()/10;
	
  // rotation is scale invariant, just start with 1
  fmat::fmatReal oriDist = 1;
	
  std::valarray<fmat::fmatReal> q(j.getDepth()+1);
  for(KinematicJoint* ancestor = &j; ancestor!=NULL; ancestor = ancestor->getParent())
    q[ancestor->getDepth()] = ancestor->getQ();
  std::valarray<fmat::fmatReal> dq, ndq(j.getDepth()+1);
  unsigned int iterCnt=0, overCnt=0;
  bool solved=false;
  while(iterCnt<MAX_ITER) {
    IKSolver::StepResult_t res = step(pEff,oriEff,j,pTgt,pDist,posPri,oriTgt,oriDist,oriPri,false);
    if (res==SUCCESS) {
      solved=true;
      break;
    }
    if (res!=PROGRESS) {
      std::cout << "Result " << res << std::endl;
      break;
    }
    for(KinematicJoint * ancestor = &j; ancestor!=NULL; ancestor = ancestor->getParent()) {
      ndq[ancestor->getDepth()] = ancestor->getQ() - q[ancestor->getDepth()];
      q[ancestor->getDepth()] = ancestor->getQ();
    }
    fmat::fmatReal qa=1;
    if (dq.size()==0) {
      dq.resize(j.getDepth()+1);
      dq=ndq;
    } else {
      qa = (dq*ndq).sum();
      std::swap(dq,ndq);
    }
    // only need abs on min: if negative, it will be more so
    if ( std::max(std::abs(dq.max()), std::abs(dq.min())) < QTOL ) {
      std::cout << "Converged" << std::endl;
      break; // not actually making progress... probably out of range
    }
    if (qa<0) {
      // this step reversed direction of previous
      // step, probably overshot target, so cut
      // distance in half for next try
      pDist *= 0.75f;
      oriDist *= 0.75f;
      ++overCnt;
    }/* else {
     // more gradual convergence
     pDist*=.9;
     oriDist*=.9;
     }*/
    ++iterCnt;
  }
  cout << "Iterations: " << iterCnt << "  Overshoots: " << overCnt 
       << " solved=" << solved << endl;
  return solved;
}

IKSolver::StepResult_t IKGradientSolver::step(const Point& pEff, const Rotation& oriEff,
					      KinematicJoint& j, const Position& pTgt, float pDist, float posPri,
					      const Orientation& oriTgt, float oriDist, float oriPri, 
					      bool successInReach) const {
  if ((posPri<=0 && oriPri<=0) || (pDist<=0 && oriDist<=0))
    return SUCCESS;
	
  posPri=std::max(0.f,posPri);
  oriPri=std::max(0.f,oriPri);
  pDist=std::max(0.f,pDist);
  oriDist=std::max(0.f,oriDist);
	
  const float totPri = posPri+oriPri;
  posPri/=totPri;
  oriPri/=totPri;
	
  // if we run out of joints to move, then we'll return joint-limits-hit
  StepResult_t result = LIMITS;
	
  // only work with joints for which qmin≠qmax (i.e. mobile joints)
  std::vector<int> mobile;
  mobile.reserve(j.getDepth()+1);
  for(KinematicJoint * ancestor=&j; ancestor!=NULL; ancestor=ancestor->getParent()) {
    if (ancestor->isMobile())
      mobile.push_back(ancestor->getDepth());
    //cout << "Joint " << ancestor->getDepth() << " @ " << ancestor->getQ() << endl;
    //cout << ancestor->getFullT() << endl;
  }
  std::reverse(mobile.begin(),mobile.end());
	
  // as we hit joint limits, we'll resolve with the remaining step distance:
  fmat::fmatReal distRemain = pDist;
  fmat::fmatReal angRemain = oriDist;
	
  while(mobile.size()>0) {
    fmat::Transform Tj = j.getFullT();
    fmat::Column<3> pEffBase(Tj*pEff);
    fmat::Quaternion qj = j.getWorldQuaternion(); //cout << "WorldQ: " << qj;
    Rotation oEffBase = qj*oriEff;
		
    fmat::Column<3> pErr;
    pTgt.computeErrorGradient(pEffBase,oEffBase,pErr);
		
    fmat::Quaternion oriErr;
    oriTgt.computeErrorGradient(pEffBase,oEffBase,oriErr);
    oriErr.normalize(); // importantly, this flips axis as needed to ensure positive angle, which is assumed later
		
    fmat::fmatReal pErrMagnitude = pErr.norm();
    fmat::fmatReal oriErrMagnitude = std::abs(oriErr.angle());
    // cout << "\nCurrent: " << oEffBase << ' ' << oEffBase.angle() << ' ' << oEffBase.axis() << endl;
    // cout << "Remain: " << oriErr << ' ' << oriErrMagnitude << ' ' << angRemain << ' ' << oriErrMagnitude/angRemain << endl;
    if (pErrMagnitude/distRemain*posPri < .001 && oriErrMagnitude/angRemain*oriPri < .001) {
      //cout << "Nailed it!" << endl;
      return SUCCESS;
    }
    if (pErrMagnitude*posPri < PTOL && oriErrMagnitude*oriPri < OTOL) {
      //cout << "Close enough! " << pErrMagnitude << ' ' << posPri << ' ' << PTOL << endl;
      return SUCCESS;
    }
    if (pErrMagnitude<distRemain && oriErrMagnitude<angRemain) {
      distRemain=pErrMagnitude;
      angRemain=oriErrMagnitude;
      if (successInReach)
	     result = SUCCESS; // we'll still take a step, but we're basically there
    } else if (pErrMagnitude<distRemain) {
      distRemain=pErrMagnitude;
      if (oriPri==0 && successInReach)
	     result = SUCCESS;
    } else if (oriErrMagnitude<angRemain) {
      angRemain=oriErrMagnitude;
      if (posPri==0 && successInReach)
	     result = SUCCESS;
    }
		
    std::vector<fmat::Column<6> > J;
    j.getFullJacobian(pEffBase,J);
    //cout << "Jacobian\n"; for(unsigned int i=0; i<J.size(); ++i) std::cout << J[i] << '\n';
    std::vector<fmat::Column<6> > Jm(mobile.size());
    for(size_t i=0; i<mobile.size(); ++i)
      Jm[i] = J[mobile[i]];
    //cout << "Jacobian mobile only\n"; for(unsigned int i=0; i<Jm.size(); ++i) std::cout << Jm[i] << '\n';
		
    std::valarray<fmat::fmatReal> q(mobile.size());
		
    // Pos start
    if (posPri>0 && distRemain>0) {
      // need to find pseudo-inverse of J:
			
      // use transposed jacobian as inverse (see "virtual work" -- there is some rationale why this works!)
      // basically we're simulating a force "pulling" on the effector
			
      // find candidate q's for mobile joints regarding positional error
      // multiplying the Jm transpose by the error, aka take dot product of each column with error
      std::valarray<fmat::fmatReal> posQ(Jm.size());
      for(unsigned int i=0; i<posQ.size(); ++i) {
	     posQ[i]=fmat::dotProduct(fmat::SubVector<3>(Jm[i]),pErr);
      }
			
      // when joints are aligned, each tries to take the full step, so we overshoot
      // so normalize q's by the size of the step they would produce (according to jacobian anyway...),
      // and then scale by the size of the step we wanted to take
      fmat::Column<3> move;
      for(unsigned int i=0; i<posQ.size(); ++i)
	       move+=fmat::SubVector<3>(Jm[i])*posQ[i];
			
      fmat::fmatReal moveSize = move.norm();
      if (moveSize > std::numeric_limits<float>::epsilon())
	       posQ *= distRemain / moveSize;
			
      // if we're only moving to solve positional error, see if we're at a local minimum (not making progress)
      if (oriErrMagnitude*oriPri < OTOL) {
      	//std::cout << "err " << pErr << ' ' << pErrMagnitude << " move " << move << ' ' << moveSize << std::endl;
      	// if we're moving perpendicular to the target, we're done... (avoid twitching)
      	fmat::fmatReal moveCos = fmat::dotProduct(pErr/pErrMagnitude, move/moveSize);
      	if (moveCos < QTOL) {
      	  //std::cout << "Move cos " << moveCos << std::endl;
      	  return RANGE;
    	   }
      	//cout << "progress angle: " << moveCos << ' ' << std::acos(moveCos) << endl;
      	//cout << "progress percent: " << moveCos*distRemain / pErrMagnitude << endl;
      	fmat::fmatReal posqsum=0;
      	for(unsigned int i=0; i<posQ.size(); ++i)
      	  posqsum+=posQ[i]*posQ[i];
      	if (std::sqrt(posqsum) < std::numeric_limits<float>::epsilon()) {
      	  //std::cout << "Move cos " << moveCos << " posqsum " << posqsum << std::endl;
      	  return RANGE;
      	}
      }
			
      // combine weighted factors:
      posQ*=posPri; // posPri and oriPri are normalized at the top so we don't divide by (posPri+oriPri)
      q += posQ;
    }
		
    // Ori start
    if (oriPri>0 && angRemain>0) {
      // orientation correction movment:
      std::valarray<fmat::fmatReal> oriQ(Jm.size());
      fmat::Column<3> oriErrV = oriErr.axis();
      // std::cerr << "oriErr " << oriErr << "  " << oriErr.angle() << " " << oriErrV << std::endl;
      for(unsigned int i=0; i<oriQ.size(); ++i) {
      	oriQ[i] = fmat::dotProduct(fmat::SubVector<3>(Jm[i],3),oriErrV);
      	//std::cout << oriQ[i] << ' ';
      }
			
      // normalization for desired magnitude of orientation movement:
      fmat::fmatReal rotSize=0;
      for(unsigned int i=0; i<oriQ.size(); ++i)
	       rotSize+=oriQ[i]*oriQ[i];
      rotSize=std::sqrt(rotSize);
      /*for(unsigned int i=0; i<oriQ.size(); ++i)
	     rotSize+=std::abs(oriQ[i]);*/
      // std::cout << " * " << oriDist << " * " << angRemain << " / " << rotSize << std::endl;
      if (rotSize > std::numeric_limits<float>::epsilon())
	       oriQ *= oriDist * angRemain / rotSize;
			
      // combine weighted factors:
      oriQ*=oriPri; // posPri and oriPri are normalized at the top so we don't divide by (posPri+oriPri)
      q += oriQ; 
    }
		
    // check for range limits:
    size_t prevSize=mobile.size();
    KinematicJoint * ancestor=&j;
    for(int i=mobile.size()-1; i>=0; --i) {
      while(ancestor!=NULL && ancestor->getDepth()>static_cast<decltype(ancestor->getDepth())>(mobile[i]))
	       ancestor=ancestor->getParent();
      q[i]+=ancestor->getQ();
      if (!ancestor->validQ(q[i])) {
      	// if a joint hits the limit, fix it there (then we'll loop over again with the remaining joints in a bit)
      	// cout << "mobile " << i << " (" << mobile[i] << ") is past limit" << endl;
      	ancestor->tryQ(q[i]);
      	std::vector<int>::iterator it=mobile.begin();
      	std::advance(it,i);
      	mobile.erase(it);
      }
    }
    if (mobile.size()<prevSize) {
      // some joint(s) hit their limit, find distance remaining and we'll try again
      fmat::Column<3> newpEffBase(j.getFullT()*pEff);
      distRemain -= (newpEffBase - pEffBase).norm();
			
      fmat::Quaternion newoEffBase = j.getWorldQuaternion()*oriEff;
      fmat::Quaternion angMoved = crossProduct(newoEffBase, oEffBase);
      // std::cout << "REMAIN: " << angMoved.angle() << " of " << angRemain << ", dist " << distRemain << std::endl;
      angRemain -= std::abs(angMoved.angle());
      // std::cout << "NOW " << angRemain << std::endl;

      if (angRemain<=0 && distRemain<=0)
	     return PROGRESS;
      if (distRemain<std::numeric_limits<float>::epsilon())
	     distRemain=std::numeric_limits<float>::epsilon();
      if (angRemain<std::numeric_limits<float>::epsilon())
	     angRemain=std::numeric_limits<float>::epsilon();
    } else {
      // no joints hit the limit, so we're done with the step!
      ancestor=&j;
      for(int i=mobile.size()-1; i>=0; --i) {
	     while(ancestor!=NULL && ancestor->getDepth()>static_cast<decltype(ancestor->getDepth())>(mobile[i]))
	       ancestor=ancestor->getParent();
	     ancestor->setQ(q[i]);
      }
      return (result==LIMITS) ? PROGRESS : result;
    }
  }
  return result;
}

/*! @file
 * @brief Implements IKGradientSolver, which performs gradient descent on the joints to find a solution
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
bool IKGradientSolver::solve(const Point& pEff, const Rotation& oriEff, 
           KinematicJoint& j, const Position& pTgt, float posPri, 
           const Orientation& oriTgt, float oriPri, unsigned int root) const {
  if (posPri==0 && oriPri==0) {
    return true;
  }

  posPri=std::max(0.f,posPri);
  oriPri=std::max(0.f,oriPri);
  
  KinematicJoint* firstMobileJoint = NULL;
  for(KinematicJoint *ancestor = &j; ancestor!=NULL; ancestor = ancestor->getParent()) {
    if (ancestor->isMobile()) {
      firstMobileJoint = ancestor;
    }
    if (ancestor->outputOffset == root) {
      break;
    }
  }
  if (firstMobileJoint==NULL) {
    return false;
  }
  
  // choose pDist by heuristic: proportional to distance between effector and base frame
  fmat::Transform Tj = j.getT(*firstMobileJoint);
  fmat::Column<3> pEffFMJ(Tj*pEff);
  fmat::fmatReal pDist = pEffFMJ.norm()/10;
  
  // rotation is scale invariant, just start with 1
  fmat::fmatReal oriDist = 1;
  
  std::valarray<fmat::fmatReal> q(j.getDepth()+1);
  for(KinematicJoint * ancestor = &j; ancestor!=NULL; ancestor = ancestor->getParent()) {
    q[ancestor->getDepth()] = ancestor->getQ();
  }
    
  std::valarray<fmat::fmatReal> dq, ndq(j.getDepth()+1);
  unsigned int iterCnt=0, overCnt=0;
  bool solved=false;
  while(iterCnt<MAX_ITER) {
    IKSolver::StepResult_t res = step(pEff,oriEff,j,pTgt,pDist,posPri,oriTgt,oriDist,oriPri,false, root);
    if (res==SUCCESS) {
      solved=true;
      break;
    }
    if (res!=PROGRESS) {
      std::cout << "Result " << res << std::endl;
      break;
    }
    for(KinematicJoint * ancestor = &j; ancestor!=NULL; ancestor = ancestor->getParent()) {
      ndq[ancestor->getDepth()] = ancestor->getQ() - q[ancestor->getDepth()];
      q[ancestor->getDepth()] = ancestor->getQ();
    }
    fmat::fmatReal qa=1;
    if (dq.size()==0) {
      dq.resize(j.getDepth()+1);
      dq=ndq;
    } else {
      qa = (dq*ndq).sum();
      std::swap(dq,ndq);
    }
    // only need abs on min: if negative, it will be more so
    if ( std::max(std::abs(dq.max()), std::abs(dq.min())) < QTOL ) {
      std::cout << "Converged" << std::endl;
      break; // not actually making progress... probably out of range
    }
    if (qa<0) {
      // this step reversed direction of previous
      // step, probably overshot target, so cut
      // distance in half for next try
      pDist *= 0.75f;
      oriDist *= 0.75f;
      ++overCnt;
    }/* else {
     // more gradual convergence
     pDist*=.9;
     oriDist*=.9;
     }*/
    ++iterCnt;
  }
  //cout << "Iterations: " << iterCnt << "  Overshoots: " << overCnt 
  //     << " solved=" << solved << endl;
  return solved;
}

IKSolver::StepResult_t IKGradientSolver::step(const Point& pEff, const Rotation& oriEff,
                KinematicJoint& j, const Position& pTgt, float pDist, float posPri,
                const Orientation& oriTgt, float oriDist, float oriPri, 
                bool successInReach, unsigned int root) const {
  if ((posPri<=0 && oriPri<=0) || (pDist<=0 && oriDist<=0)) {
    return SUCCESS;
  }
  
  posPri=std::max(0.f,posPri);
  oriPri=std::max(0.f,oriPri);
  pDist=std::max(0.f,pDist);
  oriDist=std::max(0.f,oriDist);
  
  const float totPri = posPri+oriPri;
  posPri/=totPri;
  oriPri/=totPri;
  
  // if we run out of joints to move, then we'll return joint-limits-hit
  StepResult_t result = LIMITS;
  
  // only work with joints for which qmin≠qmax (i.e. mobile joints)
  std::vector<int> mobile;
  mobile.reserve(j.getDepth()+1);
  for(KinematicJoint * ancestor=&j; ancestor!=NULL; ancestor=ancestor->getParent()) {
    if (ancestor->isMobile()) {
      mobile.push_back(ancestor->getDepth());
    }
    if (ancestor->outputOffset == root) {
      break;
    }
    //cout << "Joint " << ancestor->getDepth() << " @ " << ancestor->getQ() << endl;
    //cout << ancestor->getFullT() << endl;
  }
  std::reverse(mobile.begin(),mobile.end());
  
  // as we hit joint limits, we'll resolve with the remaining step distance:
  fmat::fmatReal distRemain = pDist;
  fmat::fmatReal angRemain = oriDist;
  
  while(mobile.size()>0) {
    fmat::Transform Tj = j.getFullT();
    fmat::Column<3> pEffBase(Tj*pEff);
    fmat::Quaternion qj = j.getWorldQuaternion(); //cout << "WorldQ: " << qj;
    Rotation oEffBase = qj*oriEff;
    
    fmat::Column<3> pErr;
    pTgt.computeErrorGradient(pEffBase,oEffBase,pErr);
    
    fmat::Quaternion oriErr;
    oriTgt.computeErrorGradient(pEffBase,oEffBase,oriErr);
    oriErr.normalize(); // importantly, this flips axis as needed to ensure positive angle, which is assumed later
    
    fmat::fmatReal pErrMagnitude = pErr.norm();
    fmat::fmatReal oriErrMagnitude = std::abs(oriErr.angle());
    // cout << "\nCurrent: " << oEffBase << ' ' << oEffBase.angle() << ' ' << oEffBase.axis() << endl;
    // cout << "Remain: " << oriErr << ' ' << oriErrMagnitude << ' ' << angRemain << ' ' << oriErrMagnitude/angRemain << endl;
    if (pErrMagnitude/distRemain*posPri < .001 && oriErrMagnitude/angRemain*oriPri < .001) {
      //cout << "Nailed it!" << endl;
      return SUCCESS;
    }
    if (pErrMagnitude*posPri < PTOL && oriErrMagnitude*oriPri < OTOL) {
      //cout << "Close enough! " << pErrMagnitude << ' ' << posPri << ' ' << PTOL << endl;
      return SUCCESS;
    }
    if (pErrMagnitude<distRemain && oriErrMagnitude<angRemain) {
        distRemain=pErrMagnitude;
        angRemain=oriErrMagnitude;
        if (successInReach) {
          result = SUCCESS;
        }
    } else if (pErrMagnitude<distRemain) {
        distRemain=pErrMagnitude;
        if (oriPri==0 && successInReach) {
          result = SUCCESS;
        }
    } else if (oriErrMagnitude<angRemain) {
        angRemain=oriErrMagnitude;
        if (posPri==0 && successInReach) {
          result = SUCCESS;
        }
    }
    
    std::vector<fmat::Column<6> > J;
    j.getFullJacobian(pEffBase,J);
    //cout << "Jacobian\n"; for(unsigned int i=0; i<J.size(); ++i) std::cout << J[i] << '\n';
    std::vector<fmat::Column<6> > Jm(mobile.size());
    for(size_t i=0; i<mobile.size(); ++i)
      Jm[i] = J[mobile[i]];
    //cout << "Jacobian mobile only\n"; for(unsigned int i=0; i<Jm.size(); ++i) std::cout << Jm[i] << '\n';
    
    std::valarray<fmat::fmatReal> q(mobile.size());
    
    if (posPri>0 && distRemain>0) {
      // need to find pseudo-inverse of J:
      
      // use transposed jacobian as inverse (see "virtual work" -- there is some rationale why this works!)
      // basically we're simulating a force "pulling" on the effector
      
      // find candidate q's for mobile joints regarding positional error
      // multiplying the Jm transpose by the error, aka take dot product of each column with error
      std::valarray<fmat::fmatReal> posQ(Jm.size());
      for(unsigned int i=0; i<posQ.size(); ++i) {
       posQ[i]=fmat::dotProduct(fmat::SubVector<3>(Jm[i]),pErr);
      }
      
      // when joints are aligned, each tries to take the full step, so we overshoot
      // so normalize q's by the size of the step they would produce (according to jacobian anyway...),
      // and then scale by the size of the step we wanted to take
      fmat::Column<3> move;
      for(unsigned int i=0; i<posQ.size(); ++i)
       move+=fmat::SubVector<3>(Jm[i])*posQ[i];
      
      fmat::fmatReal moveSize = move.norm();
      if (moveSize > std::numeric_limits<float>::epsilon())
       posQ *= distRemain / moveSize;
      
      // if we're only moving to solve positional error, see if we're at a local minimum (not making progress)
      if (oriErrMagnitude*oriPri < OTOL) {
        //std::cout << "err " << pErr << ' ' << pErrMagnitude << " move " << move << ' ' << moveSize << std::endl;
        // if we're moving perpendicular to the target, we're done... (avoid twitching)
        fmat::fmatReal moveCos = fmat::dotProduct(pErr/pErrMagnitude, move/moveSize);
        if (moveCos < QTOL) {
          //std::cout << "Move cos " << moveCos << std::endl;
          return RANGE;
        }
        //cout << "progress angle: " << moveCos << ' ' << std::acos(moveCos) << endl;
        //cout << "progress percent: " << moveCos*distRemain / pErrMagnitude << endl;
        fmat::fmatReal posqsum=0;
        for(unsigned int i=0; i<posQ.size(); ++i)
          posqsum+=posQ[i]*posQ[i];
        if (std::sqrt(posqsum) < std::numeric_limits<float>::epsilon()) {
          //std::cout << "Move cos " << moveCos << " posqsum " << posqsum << std::endl;
          return RANGE;
        }
      }
      
      // combine weighted factors:
      posQ*=posPri; // posPri and oriPri are normalized at the top so we don't divide by (posPri+oriPri)
      q += posQ;
    }
    
    if (oriPri>0 && angRemain>0) {
      // orientation correction movment:
      std::valarray<fmat::fmatReal> oriQ(Jm.size());
      fmat::Column<3> oriErrV = oriErr.axis();
      // std::cerr << "oriErr " << oriErr << "  " << oriErr.angle() << " " << oriErrV << std::endl;
      for(unsigned int i=0; i<oriQ.size(); ++i) {
        oriQ[i] = fmat::dotProduct(fmat::SubVector<3>(Jm[i],3),oriErrV);
        //std::cout << oriQ[i] << ' ';
      }
      
      // normalization for desired magnitude of orientation movement:
      fmat::fmatReal rotSize=0;
      for(unsigned int i=0; i<oriQ.size(); ++i)
       rotSize+=oriQ[i]*oriQ[i];
      rotSize=std::sqrt(rotSize);
      /*for(unsigned int i=0; i<oriQ.size(); ++i)
       rotSize+=std::abs(oriQ[i]);*/
      // std::cout << " * " << oriDist << " * " << angRemain << " / " << rotSize << std::endl;
      if (rotSize > std::numeric_limits<float>::epsilon())
       oriQ *= oriDist * angRemain / rotSize;
      
      // combine weighted factors:
      oriQ*=oriPri; // posPri and oriPri are normalized at the top so we don't divide by (posPri+oriPri)
      q += oriQ;
    }
    
    // check for range limits:
    size_t prevSize=mobile.size();
    KinematicJoint * ancestor=&j;
    for(int i=mobile.size()-1; i>=0; --i) {
      while(ancestor!=NULL && ancestor->getDepth()>static_cast<decltype(ancestor->getDepth())>(mobile[i]))
         ancestor=ancestor->getParent();
      q[i]+=ancestor->getQ();
      if (!ancestor->validQ(q[i])) {
        // if a joint hits the limit, fix it there (then we'll loop over again with the remaining joints in a bit)
        // cout << "mobile " << i << " (" << mobile[i] << ") is past limit" << endl;
        ancestor->tryQ(q[i]);
        std::vector<int>::iterator it=mobile.begin();
        std::advance(it,i);
        mobile.erase(it);
      }
    }
    if (mobile.size()<prevSize) {
      // some joint(s) hit their limit, find distance remaining and we'll try again
      fmat::Column<3> newpEffBase(j.getFullT()*pEff);
      distRemain -= (newpEffBase - pEffBase).norm();
      
      fmat::Quaternion newoEffBase = j.getWorldQuaternion()*oriEff;
      fmat::Quaternion angMoved = crossProduct(newoEffBase, oEffBase);
      // std::cout << "REMAIN: " << angMoved.angle() << " of " << angRemain << ", dist " << distRemain << std::endl;
      angRemain -= std::abs(angMoved.angle());
      // std::cout << "NOW " << angRemain << std::endl;

      if (angRemain<=0 && distRemain<=0)
  return PROGRESS;
      if (distRemain<std::numeric_limits<float>::epsilon())
  distRemain=std::numeric_limits<float>::epsilon();
      if (angRemain<std::numeric_limits<float>::epsilon())
  angRemain=std::numeric_limits<float>::epsilon();
    } else {
      // no joints hit the limit, so we're done with the step!
      ancestor=&j;
      for(int i=mobile.size()-1; i>=0; --i) {
  while(ancestor!=NULL && ancestor->getDepth()>static_cast<decltype(ancestor->getDepth())>(mobile[i]))
    ancestor=ancestor->getParent();
  ancestor->setQ(q[i]);
      }
      return (result==LIMITS) ? PROGRESS : result;
    }
  }
  return result;
}

/*! @file
 * @brief Implements IKGradientSolver, which performs gradient descent on the joints to find a solution
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
