#if defined(TGT_ERS2xx) || defined(TGT_ERS220)

#include "Behaviors/BehaviorBase.h"
#include "Motion/MotionPtr.h"
#include "Motion/PostureMC.h"
#include "Shared/ERS220Info.h"

//! opens or closes the head light on an ERS-220
class ToggleHeadLightBehavior : public BehaviorBase {
public:
	//! constructor
	ToggleHeadLightBehavior() : BehaviorBase("ToggleHeadLightBehavior"), light() {}

	//! opens the head light
	virtual void doStart() {
		BehaviorBase::doStart();
		if(RobotName == ERS220Info::TargetName) {
			unsigned int idx = ERS220Info::RetractableHeadLEDOffset;
			light->setOutputCmd(idx, state->outputs[idx]>0.5 ? 0 : 1);
			addMotion(light);
		}
	}

	static std::string getClassDescription() { return "Opens or closes the head light on an ERS-220"; }
	virtual std::string getDescription() const { return getClassDescription(); }

protected:
	MotionPtr<PostureMC> light; //!< the PostureMC used to control the light
};

REGISTER_BEHAVIOR_MENU_OPT(ToggleHeadLightBehavior,"Background Behaviors",BEH_NONEXCLUSIVE);

#endif

/*! @file
 * @brief Defines ToggleHeadLightBehavior, which will open or close the head light on an ERS-220
 * @author ejt (Creator)
 */
