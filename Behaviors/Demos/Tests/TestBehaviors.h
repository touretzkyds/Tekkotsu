//-*-c++-*-
#ifndef INCLUDED_TestBehaviors_h_
#define INCLUDED_TestBehaviors_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"
#include "Motion/MotionManager.h"
#include "Motion/PostureMC.h"
#include "Shared/ProjectInterface.h"
#include "Vision/RawCameraGenerator.h"
#include "Shared/Config.h"
#include "IPC/SharedObject.h"

//! Adds a MotionCommand and then immediately removes it again
class InstantMotionTestBehavior : public BehaviorBase {
public:
	InstantMotionTestBehavior() : BehaviorBase("InstantMotionTestBehavior") {} //!< constructor
	virtual void doStart() {
		erouter->addListener(this,EventBase::motmanEGID);
		MotionManager::MC_ID mcid = motman->addPersistentMotion(SharedObject<PostureMC>());
		std::cout << "Now removing " << mcid << "..." << std::endl;
		motman->removeMotion(mcid);
		stop();
	}
	virtual void doEvent() { std::cout << event->getDescription() << std::endl; }
	static std::string getClassDescription() { return "Adds a MotionCommand and then immediately removes it again"; }
	virtual std::string getDescription() const { return getClassDescription(); }
};

//! Removes a MotionCommand and then immediately adds it again
class InstantMotionTestBehavior2 : public BehaviorBase {
public:
	InstantMotionTestBehavior2() : BehaviorBase("InstantMotionTestBehavior2"), mc(), mcid(motman->addPersistentMotion(mc)) {} //!< constructor
	~InstantMotionTestBehavior2() { motman->removeMotion(mcid); } //!< destructor
	virtual void doStart() { erouter->addTimer(this,0,500); erouter->addListener(this,EventBase::motmanEGID); }
	virtual void doEvent() {
		if(event->getGeneratorID()==EventBase::motmanEGID) {
			std::cout << event->getDescription() << std::endl;
		} else {
			std::cout << "Now removing " << mcid << "..." << std::endl;
			motman->removeMotion(mcid);
			std::cout << "Now adding back..." << std::endl;
			mcid=motman->addPersistentMotion(mc);
		}
	}
	static std::string getClassDescription() { return "Adds a MotionCommand and then immediately removes it again"; }
	virtual std::string getDescription() const { return getClassDescription(); }
protected:
	SharedObject<PostureMC> mc; //!< persistent motion to remove and immediately re-add
	MotionManager::MC_ID mcid; //!< the MC_ID from the last add
};

//! When started, busy loops for 3 seconds and then stops
class BusyLoopTestBehavior : public BehaviorBase {
public:
	BusyLoopTestBehavior() : BehaviorBase("BusyLoopTestBehavior") {} //!< constructor
	virtual void doStart() { unsigned int t=get_time(); while(get_time()-t<3000) {} stop(); }
	static std::string getClassDescription() { return "When started, busy loops for 3 seconds and then stops"; }
	virtual std::string getDescription() const { return getClassDescription(); }
};

//! Adds a MotionCommand which will busy loop for 3 seconds on its first update, and then autoprune
class BusyMCTestBehavior : public BehaviorBase {
	//! on first updateOutputs, blocks for 3 seconds, then autoprunes on second update
	class BusyMC : public MotionCommand {
		bool hasRun;
	public:
		BusyMC() : MotionCommand(), hasRun(false) {} //!< constructor
		virtual int updateOutputs() {unsigned int t=get_time(); while(get_time()-t<3000) {} hasRun=true; return 0; }
		virtual int isDirty() { return true; }
		virtual int isAlive() { return !hasRun; }
	};
public:
	BusyMCTestBehavior() : BehaviorBase("BusyMCTestBehavior") {} //!< constructor
	virtual void doStart() { motman->addPrunableMotion(SharedObject<BusyMC>()); stop(); }
	static std::string getClassDescription() { return "Adds a MotionCommand which will busy loop for 3 seconds on its first update, and then autoprune"; }
	virtual std::string getDescription() const { return getClassDescription(); }
};

//! Stops itself after a second via timer event
class SuicidalBehavior : public BehaviorBase {
public:
	SuicidalBehavior() : BehaviorBase("SuicidalBehavior") {} //!< constructor
	~SuicidalBehavior() { std::cout << getName() << " destructed" << std::endl; }  //!< destructor
	virtual void doStart() { erouter->addTimer(this,0,1000); std::cout << "One second to live!" << std::endl; }
	virtual void doEvent() { std::cout << "I'm stopping -- refresh controller to see if it worked!" << std::endl; stop(); }
	static std::string getClassDescription() { return "Stops itself after a second via timer event"; }
	virtual std::string getDescription() const { return getClassDescription(); }
};

//! Echos any text message events received to the console (doesn't add a listener, relies on direct BehaviorSwitchControl passing)
class EchoTextBehavior : public BehaviorBase {
public:
	EchoTextBehavior() : BehaviorBase("EchoTextBehavior") {} //!< constructor
	virtual void doEvent() { if(const TextMsgEvent* tev=dynamic_cast<const TextMsgEvent*>(event)) std::cout << tev->getText() << std::endl; else std::cout << getName() << " got a non-TextMsgEvent" << std::endl; }
	static std::string getClassDescription() { return "Echos any text message events received to the console (doesn't add a listener, relies on direct BehaviorSwitchControl passing)"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	virtual void doStart() {}
};

//! Saves all images currently provided by the hardware to a series of PNG files on the memory stick
/*! Handy for examining the different channels/resolution levels provided by the system, all based on the same input */
class SaveImagePyramidBehavior : public BehaviorBase {
public:
	SaveImagePyramidBehavior() : BehaviorBase("SaveImagePyramidBehavior") {} //!< constructor
	virtual void doStart() { erouter->addListener(this,EventBase::textmsgEGID); }
	virtual void doEvent() {
		if(const TextMsgEvent* tev=dynamic_cast<const TextMsgEvent*>(event)) {
			std::string prefix=config->portPath(tev->getText().substr(0,4));
			saveImage(ProjectInterface::doubleLayer,RawCameraGenerator::CHAN_Y,prefix+"%dy.png");
			for(unsigned int layer=ProjectInterface::fullLayer; layer>=ProjectInterface::quarterLayer; layer--) {
				saveImage(layer,RawCameraGenerator::CHAN_Y,prefix+"%dy.png");
				saveImage(layer,RawCameraGenerator::CHAN_U,prefix+"%du.png");
				saveImage(layer,RawCameraGenerator::CHAN_V,prefix+"%dv.png");
				saveImage(layer,RawCameraGenerator::CHAN_Y_DX,prefix+"%ddx.png");
				saveImage(layer,RawCameraGenerator::CHAN_Y_DY,prefix+"%ddy.png");
				saveImage(layer,RawCameraGenerator::CHAN_Y_DXDY,prefix+"%ddxy.png");
			}
		} else std::cout << getName() << " got a non-TextMsgEvent" << std::endl;
	}
	static std::string getClassDescription() { return "Saves all images currently provided by the hardware to a series of PNG files on the memory stick"; }
	virtual std::string getDescription() const { return getClassDescription(); }
protected:
	void saveImage(unsigned int layer, unsigned int chan, const std::string& name);
};

/*! @file
 * @brief A collection of small test behaviors (some would call them unit tests)
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */

#endif
