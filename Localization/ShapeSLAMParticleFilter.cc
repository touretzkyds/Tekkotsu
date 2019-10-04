//-*-c++-*-

#include <iostream>
#include <cmath>
#if defined(PLATFORM_APERIOS) && !defined(HUGE_VALF)
#  define HUGE_VALF HUGE_VAL
#endif

#include "Crew/MapBuilder.h"
#include "Motion/WalkMC.h"
#ifdef TGT_HAS_WALK
#  include "Crew/Pilot.h"
#endif
#include "DualCoding/ShapeSpace.h"
#include "DualCoding/VRmixin.h"  // for mapBuilder
#include "DualCoding/ShapeLocalizationParticle.h"
#include "ShapeSLAMParticleFilter.h"
#include "ShapeLandmarks.h"

using namespace std;

namespace DualCoding {

	ShapeSLAMParticleEvaluator::ShapeSLAMParticleEvaluator(ShapeSpace &localShS, ShapeSpace &worldShS, float addPenalty) :
	LocalShapeEvaluator(localShS,worldShS), localMobile(false), worldMobile(false), ADDITION_PENALTY(addPenalty)
    {
		for ( unsigned int indexL=0; indexL < localLms.size(); indexL++ )
			localMobile |= localLms[indexL]->mobile;
		for ( unsigned int indexW=0; indexW < worldLms.size(); indexW++ )
			worldMobile |= worldLms[indexW]->mobile;
    }
	
  void ShapeSLAMParticleEvaluator::evaluate(ShapeSLAMParticle& part) {
    unsigned int const nLocals = localLms.size();
    float particleViewX[nLocals],  particleViewY[nLocals], particleViewX2[nLocals],  particleViewY2[nLocals];
    int localMatches[nLocals];
    float localScores[nLocals];
    LocalShapeEvaluator::evaluateWorkhorse(part, nLocals, particleViewX, particleViewY, particleViewX2, particleViewY2,
					   localMatches, localScores);
    for(unsigned int indexL=0; indexL < nLocals; indexL++)
      part.addLocal[indexL] = ( localMatches[indexL] == -1 && localLms[indexL]->mobile );
    if ( localMobile )
      determineAdditions(part, nLocals, localMatches, localScores);
    if ( worldMobile )
      determineDeletions(part, nLocals, localMatches, particleViewX, particleViewY, particleViewX2, particleViewY2);
    updateWeight(part, localMatches, localScores);

    // << ": " << bestProb << endl;
  }

  void ShapeSLAMParticleEvaluator::determineAdditions
  (ShapeSLAMParticle& part, unsigned int const nLocals,  int localMatches[], float localScores[]) {
    for (unsigned int indexL = 0; indexL < nLocals; indexL++) {
      if ( localLms[indexL]->mobile  ) {
	float const randval = float(rand()) / (float(RAND_MAX)*6);
	//**** WARNING: this code assumes we're using log weights (which is true by default)
	if (randval >= localScores[indexL]) {
	  part.addLocal[indexL] = true;
	  localScores[indexL] = ADDITION_PENALTY;
	  localMatches[indexL] = -1000 -  indexL;  // any value other than -1 will count as a match
	  continue;
	}
	// if two local LMs match the same worldLM, treat the poorer match as an addition
	for (unsigned int indexL2 = (indexL+1); indexL2 < nLocals; indexL2++) {
	  if (localMatches[indexL2] != localMatches[indexL] || localMatches[indexL2] == -1)
	    continue;
	  if (localScores[indexL2] > localScores[indexL]) {
	    part.addLocal[indexL] = true;
	    localMatches[indexL] = -1000 - indexL;
	    localScores[indexL] = ADDITION_PENALTY;
	  } else {
	    part.addLocal[indexL2] = true;
	    localMatches[indexL2] = -1000 - indexL2;
	    localScores[indexL2] = ADDITION_PENALTY;
	  }
	}
      }
    }
  }

  void ShapeSLAMParticleEvaluator::determineDeletions
    (ShapeSLAMParticle& part, unsigned int const nLocals, int const localMatches[],
     float const particleViewX[], float const particleViewY[], float const particleViewX2[], float const particleViewY2[]) {
    part.deleteWorld.assign(part.deleteWorld.size(),true);
    float minXLoc = HUGE_VALF;
    float minYLoc = HUGE_VALF;
    float maxXLoc = -HUGE_VALF;
    float maxYLoc = -HUGE_VALF;
    for (unsigned int indexL = 0; indexL<nLocals; indexL++) {
      if ( localMatches[indexL] != -1 )
	part.deleteWorld[localMatches[indexL]] = false;  // don't delete world LM if it matches
      if ( particleViewX[indexL] < minXLoc )
	minXLoc = particleViewX[indexL];
      else if (particleViewX[indexL] > maxXLoc)
	maxXLoc = particleViewX[indexL];
      if (particleViewY[indexL] < minYLoc)
	minYLoc = particleViewY[indexL];
      else if (particleViewY[indexL] > maxYLoc)
	maxYLoc = particleViewY[indexL];
      if ( localLms[indexL]->type == lineDataType ) {
	if ( particleViewX2[indexL] < minXLoc )
	  minXLoc = particleViewX2[indexL];
	else if (particleViewX2[indexL] > maxXLoc)
	  maxXLoc = particleViewX2[indexL];
	if (particleViewY2[indexL] < minYLoc)
	  minYLoc = particleViewY2[indexL];
	else if (particleViewY2[indexL] > maxYLoc)
	  maxYLoc = particleViewY2[indexL];
      }
    }
	
    for (unsigned int indexW = 0; indexW<worldLms.size(); indexW++)
      if ( ! worldLms[indexW]->mobile ||
	   !( worldLms[indexW]->x >= minXLoc && worldLms[indexW]->x <= maxXLoc &&
	      worldLms[indexW]->y >= minYLoc && worldLms[indexW]->y <= maxYLoc ) )
	part.deleteWorld[indexW] = false; // don't delete world LM if not mobile, or it was outside local view
  }


  void ShapeSLAMParticleFilter::setAgent() const {
#ifdef TGT_HAS_WALK
    //    const ShapeSLAMParticleFilter::particle_type& best;
    //    VRmixin::pilot->setAgent(Point(best.x,best.y), best.theta, true);
#endif
  }

  void ShapeSLAMParticleFilter::displayParticles(float const howmany) const {
    cout << "displayParticles not available for ShapeSLAMParticleFilter at present." << endl;
    return;
    ShapeSpace &wShS = sensorModel->getWorldShS();
    wShS.deleteShapes<LocalizationParticleData>();
    NEW_SHAPE(best, LocalizationParticleData, NULL); //new LocalizationParticleData(wShS,particles,bestIndex));
    best->setColor(VRmixin::mapBuilder->getAgent()->getColor());
    if ( howmany <= 0 ) return;
    int increment;
    if ( howmany <= 1.0 )
      increment = (int)ceil(1.0/howmany);
    else
      increment = (int)ceil(particles.size()/howmany);
    for (unsigned int i=0; i<particles.size(); i+=increment)
      continue;
      //      if ( i != bestIndex ) {
      //	NEW_SHAPE(pt, LocalizationParticleData, NULL); //new LocalizationParticleData(wShS,particles,i));
      //      }
  }

  ostream& operator << (ostream& os, const ShapeSLAMParticle &p){
    os << "Particle(p=" << p.weight
       << ", dx=" << p.x
       << ", dy=" << p.y
       << ", th=" << p.theta;
	
    os << ", add=";
    for (unsigned int i = 0; i<p.addLocal.size(); i++)
      os << p.addLocal[i];
	
    os << ", del=";
    for (unsigned int i = 0; i<p.deleteWorld.size(); i++)
      os << p.deleteWorld[i];
	
    os << ")";
    return os;
  }

} // namespace
