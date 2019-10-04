#include "Shared/RobotInfo.h"
#if defined(TGT_HAS_LEGS) && !defined(TGT_IS_AIBO) && (!defined(__GNUC__) || __GNUC__>=4)

#include "GaitedFootstepMC.h"

using namespace std; 

KinematicJoint* GaitedFootstepMC::kine=NULL;
KinematicJoint* GaitedFootstepMC::childMap[NumReferenceFrames];

int GaitedFootstepMC::updateOutputs() {
	if(curStep+1 >= steps.size()) {
		newStep=false;
		return 0;
	}
	
	double t = (get_time() / 1000.0) - stepStartTime; // time within step
	if(t > stepDuration) {
		t += stepStartTime;
		if(!advanceStep()) {
			// reached destination, set legs to final values
			for(unsigned int i=0; i<NumLegs; ++i) {
				if(!xp.legParams[i].usable)
					continue;
				fmat::Column<3> tgt;
				(fmat::SubVector<2>)tgt = steps[curStep].footPos[i];
				xp.projectToGround(ground, xp.groundPlane[3], gravity, tgt);
				solveIK(i,tgt);
				if(DriverMessaging::hasListener(DriverMessaging::FixedPoints::NAME)) {
					contactMsg.clear();
					contactMsg.flushOnMotionUpdate=true; // delay message to sync with corresponding joint angles
					postMessage(contactMsg);
				}
				newStep=false;
				return NumLegs * NumLegJoints;
			}
		}
		t -= stepStartTime;
	}
	
	fmat::Column<2> p = curBodyOffset(t);
	for(unsigned int i=0; i<NumLegs; ++i) {
		if(!xp.legParams[i].usable)
			continue;
		
		fmat::Column<3> tgt;
		if(support[i]) {
			// currently in contact with ground, just track body motion
			(fmat::SubVector<2>)tgt = steps[curStep].footPos[i] - steps[curStep].pos - p;
			xp.projectToGround(ground, xp.groundPlane[3], gravity, tgt);

		} else {
			// need to lift leg and place at next contact point
			double raiseDur = xp.legParams[i].raiseDuration / 1000.0;
			double flightDur = xp.legParams[i].flightDuration / 1000.0;
			double lowerDur = xp.legParams[i].lowerDuration / 1000.0;

			if(t < raiseDur) {
				// rising leg - track old contact, increase height
				fmat::fmatReal s = static_cast<fmat::fmatReal>(t / raiseDur);
				(fmat::SubVector<2>)tgt = steps[curStep].footPos[i] - steps[curStep].pos - p;
				xp.projectToGround(ground, xp.groundPlane[3], gravity, tgt);
				tgt[2] += xp.legParams[i].flightHeight * s;

			} else if(t < raiseDur + flightDur) {
				// flying leg - move to new contact
				fmat::fmatReal s = static_cast<fmat::fmatReal>( (t - raiseDur) / flightDur );
				fmat::Column<2> d = steps[curStep+1].footPos[i] - steps[curStep].footPos[i];
				(fmat::SubVector<2>)tgt = steps[curStep].footPos[i] - steps[curStep].pos - p + d*s;
				tgt[2] = xp.legParams[i].flightHeight;

			} else if(t < raiseDur + flightDur + lowerDur) {
				// lowering leg - track new contact, decrease height
				fmat::fmatReal s = static_cast<fmat::fmatReal>( (t - raiseDur - flightDur) / lowerDur );
				(fmat::SubVector<2>)tgt = steps[curStep+1].footPos[i] - steps[curStep].pos - p;
				xp.projectToGround(ground, xp.groundPlane[3], gravity, tgt);
				tgt[2] += xp.legParams[i].flightHeight * (1-s);

			} else {
				// step completed - continue support
				(fmat::SubVector<2>)tgt = steps[curStep+1].footPos[i] - steps[curStep].pos - p;
				xp.projectToGround(ground, xp.groundPlane[3], gravity, tgt);
			}
		}
		solveIK(i,tgt);
	}
	
	if(newStep) {
		// send driver message reporting leg contacts
		if(DriverMessaging::hasListener(DriverMessaging::FixedPoints::NAME)) {
			contactMsg.clear();
			for(unsigned int leg=0; leg<NumLegs; ++leg) {
				if(support[leg] && xp.legParams[leg].usable)
					contactMsg.addEntry(FootFrameOffset+leg, fmat::Column<3>());
			}
			contactMsg.flushOnMotionUpdate=true; // delay message to sync with corresponding joint angles
			postMessage(contactMsg);
		}
	}
	
	newStep=false;
	return NumLegs * NumLegJoints;
}

/*! internal function, assumes #curStep is at most next-to-last */
fmat::Column<2> GaitedFootstepMC::curBodyOffset(double t) {
	fmat::Column<2> d = steps[curStep+1].pos - steps[curStep].pos; // step displacement
	fmat::fmatReal s = static_cast<fmat::fmatReal>(t / stepDuration); // percentage of step
	return d * s;
}

/*! internal function, assumes #curStep is at most next-to-last */
bool GaitedFootstepMC::advanceStep() {
	newStep=true;
	++curStep;
	stepStartTime+=stepDuration;
	if(curStep+1 >= steps.size())
		return false;
	
	// which legs are in flight?
	//std::cout << "Step " << curStep << ": " << std::endl;
	for(unsigned int i=0; i<NumLegs; ++i) {
		support[i] = (steps[curStep].footPos[i] == steps[curStep+1].footPos[i]);
		//std::cout << "  leg " << i << " in support? " << support[i] << ' ' << steps[curStep].footPos[i] << ' ' << steps[curStep+1].footPos[i] << std::endl;
	}
	
	// calculate nominal step duration
	fmat::Column<2> d = steps[curStep+1].pos - steps[curStep].pos;
	fmat::fmatReal dist = d.norm();
	fmat::fmatReal speed = fmat::dotProduct( xySpeed, d ) / dist; // divide by dist to norm d
	stepDuration = dist / speed;
	
	// limit duration to flight time
	for(unsigned int i=0; i<NumLegs; ++i) {
		double legDuration = xp.legParams[i].totalDuration() / 1000.0;
		if(legDuration > stepDuration)
			stepDuration = legDuration;
	}
	
	return true;
}

void GaitedFootstepMC::solveIK(unsigned int leg, const IKSolver::Point& tgt) {
	KinematicJoint * eff = childMap[FootFrameOffset+leg];
	eff->getIK().solve(IKSolver::Point(), *eff, tgt);
	for(; eff!=NULL; eff=eff->getParent()) {
		if(eff->outputOffset!=plist::OutputSelector::UNUSED && eff->outputOffset<NumOutputs) {
			/*if(leg==1)
			 std::cout << eff->outputOffset.get() << " set to " << eff->getQ() << std::endl;*/
#ifndef PLATFORM_APERIOS
			if(!std::isnan(eff->getQ()))
#endif
			{
				motman->setOutput(this, eff->outputOffset, eff->getQ());
			}
		}
	}
}

				
#endif

/*! @file
 * @brief Implements GaitedFootstepMC, which executes a series of footsteps, probably generated by applying astar() to a GaitedFootsteps domain
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
