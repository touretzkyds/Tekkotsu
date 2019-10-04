#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) && !defined(TGT_IS_AIBO) 

#include "XWalkParameters.h"
#include "Motion/Kinematics.h"

using namespace std; 

const float XWalkParameters::EPSILON=1e-5f;

float XWalkParameters::getMaxXVel() const {
	return strideLenX/minPeriod();
}
float XWalkParameters::getMaxYVel() const {
	return strideLenY/minPeriod();
}
float XWalkParameters::getMaxAVel() const {
	float maxStride=0;
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		fmat::Column<2> neutralPos = computeNeutralPos(leg);
		float radius = (neutralPos-fmat::pack(offsetX,offsetY)).norm();
		float hyp = neutralPos.norm();
		float len = (neutralPos[1]*strideLenX + neutralPos[0]*strideLenY)/hyp;
		float strideA = std::abs(len)/radius;
		if(strideA>maxStride)
			maxStride=strideA;
	}
	return maxStride/minPeriod();
}

float XWalkParameters::nominalPeriod() const {
	std::map<float,float> phases;
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		float t = legParams[leg].totalDuration() * 2 / 1000.f;
		if(t>phases[legParams[leg].flightPhase])
			phases[legParams[leg].flightPhase] = t;
	}
	float ph = 0;
	for(std::map<float,float>::iterator it=phases.begin(); it!=phases.end(); ++it)
		ph+=it->second;
	return ph;
}

float XWalkParameters::minPeriod() const {
	std::map<float,float> phases;
	for(unsigned int leg=0; leg<NumLegs; ++leg) {
		float t = (legParams[leg].flightDuration+legParams[leg].raiseDuration/2+legParams[leg].lowerDuration/2)/1000.f;
		if(t>phases[legParams[leg].flightPhase])
			phases[legParams[leg].flightPhase] = t;
	}
	float ph = 0;
	for(std::map<float,float>::iterator it=phases.begin(); it!=phases.end(); ++it)
		ph+=it->second;
	return ph;
}

fmat::Column<2> XWalkParameters::computeNeutralPos(unsigned int leg) const {
	LegParameters& legParam = legParams[leg];
    // hack! fix it later
        #if defined(TGT_IS_MANTIS)
        static const int JointsPerLeg = 4;
        #endif
    const KinematicJoint * legRoot = kine->getKinematicJoint(LegOffset+JointsPerLeg*leg);
	fmat::Column<3> fstPos = legRoot->getWorldPosition();
	fmat::Column<2> ans;
	ans[0] = fstPos[0] + legParam.strideBias;
	ans[1] = ((fstPos[1]<0) ? -*legParam.stanceWidth : *legParam.stanceWidth);
	return ans;
}	

void XWalkParameters::projectToGround(const fmat::Column<3>& groundNormal, float height, const fmat::Column<3>& gravity, fmat::Column<3>& tgt) {
	// find distance from target point to plane, so we can restate ground plane as if tgt is the origin
	//std::cout << "Pre-project: " << tgt << std::endl;
	float dist = height - fmat::dotProduct(tgt,groundNormal);
	// now treat tgt as the origin, to be projected along gravity vector to ground plane (specified by: ground · x = dist)
	// ( gravity * t ) · ground = dist, solve for t to get gravity*t to get intersection point
	/* intersect = gravity * t
	 *  intersect = gravity * ( dist / (gravity · groundNormal) )  */
	float align = fmat::dotProduct(gravity,groundNormal);
	if(align<EPSILON && align>-EPSILON) // we're trying to walk up a perfectly vertical cliff...
		align = (align>=0) ? EPSILON : -EPSILON; //pretend it's near vertical
	//std::cout << dist << ' ' << align << ' ' << gravity << std::endl;
	tgt += gravity * (dist/align); // add this origin projected to ground plane back to original target point to be in body frame
	//std::cout << "Post-project: " << tgt << std::endl;
}

void XWalkParameters::packGroundGravity(const fmat::SubVector<3>& ground, const fmat::SubVector<3> gravity) const {
	ground[0]=groundPlane[0];
	ground[1]=groundPlane[1];
	ground[2]=groundPlane[2];
	gravity[0]=groundPlane[0] + gravityVector[0];
	gravity[1]=groundPlane[1] + gravityVector[1];
	gravity[2]=groundPlane[2] + gravityVector[2];
	
	float g = gravity.norm();
	if(g==0) // invalid gravity vector, doesn't point anywhere
		gravity=ground/ground.norm(); // assume it points into ground...
	else
		gravity/=g;
}

/*! @file
 * @brief Implements XWalkParameters, which provide configuration settings for XWalkMC
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif // TGT_HAS_LEGS
