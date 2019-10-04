//-*-c++-*-
#ifndef INCLUDED_RecordMotionNode_h_
#define INCLUDED_RecordMotionNode_h_

#include "Behaviors/StateNode.h"
#include "Events/EventRouter.h"
#include "IPC/SharedObject.h"
#include "Motion/DynamicMotionSequence.h"
#include "Motion/PIDMC.h"
#include "Shared/MarkScope.h"
#include <valarray>

//! DESCRIPTION
class RecordMotionNode : public StateNode {

	// **************************** //
	// ******* CONSTRUCTORS ******* //
	// **************************** //
public:
	//! constructor, take an instance name, filename to save to, joint position resolution (max error in radians), temporal resolution (0 for every sensor update)
	RecordMotionNode(const std::string& nm="", float angularRes=0.1f, unsigned int temporalRes=0)
	: StateNode(nm), angRes(angularRes), timeRes(temporalRes), recStartTime(), relaxID(MotionManager::invalid_MC_ID), rec()
	{}
	
	
	// **************************** //
	// ********* METHODS ********** //
	// **************************** //
public:
	virtual void preStart() {
		StateNode::preStart();
		if(timeRes<FrameTime*NumFrames)
			erouter->addListener(this,EventBase::sensorEGID);
		else
			erouter->addTimer(this,0,timeRes);
		recStartTime = state->lastSensorUpdateTime;
		for(unsigned int i=0; i<NumPIDJoints; ++i) {
			lastTime[i] = startTime[i] = recStartTime;
			lastPos[i] = startPos[i] = state->outputs[PIDJointOffset+i];
			motion[i]=error[i]=0;
			time[i]=1;
		}
		relaxID = motman->addPersistentMotion(SharedObject<PIDMC>(0));
	}

	virtual void doEvent() {
		for(unsigned int i=0; i<NumPIDJoints; ++i) {
			float t = state->lastSensorUpdateTime - startTime[i];
			float curPos = state->outputs[PIDJointOffset+i];
			float predPos = motion[i]/time[i] * t + startPos[i];
			float curErr = predPos-curPos;
			const float GAMMA = 0.8f;
			error[i] = error[i]*GAMMA + curErr*(1-GAMMA);
			/*if(std::abs(motion[i]/time[i])>1e-5)
				std::cout << i << " " << t << " @ " << motion[i]/time[i]*1000 << " current " << (curPos-lastPos[i])/(state->lastSensorUpdateTime - lastTime[i])*1000 << " " << error[i] << " " << curErr << (std::abs(curErr)<angRes ? " *" : "") << std::endl;*/
			if( std::abs(error[i]) > angRes ) {
				//std::cout << "New frame " << i << std::endl;
				rec->setTime(lastTime[i]-recStartTime);
				rec->setOutputCmd(i,lastPos[i]);
				startTime[i] = lastTime[i];
				startPos[i] = lastPos[i];
				motion[i]=time[i]=error[i]=0;
			}
			float off = curPos - startPos[i];
			motion[i] += off*t;
			time[i] += t*t;
			if( std::abs(curErr) < angRes ) {
				lastPos[i] = curPos;
				lastTime[i] = state->lastSensorUpdateTime;
			}
		}
	}

	virtual void stop() {
		motman->removeMotion(relaxID);
		rec->saveFile((getName()+".mot").c_str());
		StateNode::stop(); // do this last (required)
	}

	static std::string getClassDescription() { return "Record all joint motion during activation"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	// **************************** //
	// ********* MEMBERS ********** //
	// **************************** //
protected:
	float angRes;
	unsigned int timeRes;
	unsigned int recStartTime;
	
	unsigned int startTime[NumPIDJoints];
	float startPos[NumPIDJoints];
	float lastPos[NumPIDJoints];
	unsigned int lastTime[NumPIDJoints];
	float error[NumPIDJoints];
	
	float motion[NumPIDJoints];
	float time[NumPIDJoints];
	
	MotionManager::MC_ID relaxID;
	SharedObject<DynamicMotionSequence> rec;
	

	// **************************** //
	// ********** OTHER *********** //
	// **************************** //
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	RecordMotionNode(const RecordMotionNode&); //!< don't call (copy constructor)
	RecordMotionNode& operator=(const RecordMotionNode&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines RecordMotionNode, which DESCRIPTION
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
