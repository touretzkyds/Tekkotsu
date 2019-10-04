// INCLUDES
// tekkotsu library
#include "DualCoding/ShapeRoot.h"
#include "Shared/ProjectInterface.h"

// tekkodu library
#include "Kodu/KoduWorld.h"
#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Primitives/KoduConditionGot.h"

using namespace DualCoding;

namespace Kodu {

    bool KoduConditionGot::evaluate(const KoduWorld& kWorldState) {
        const DualCoding::ShapeRoot& kGripperObjectRef = kWorldState.thisAgent.gripperObject;
				std::string rcolor = objColor;
				if ( rcolor == "" ) {
					if ( objType == "apple" )
						rcolor = "red";
					else if ( objType == "tree" )
						rcolor = "green";
					else if ( objType == "rock" )
						rcolor = "blue";
				}
        bool rv = ((kGripperObjectRef.isValid())
                && (kWorldState.thisAgent.targetObjectIsInGripper)
                && (ProjectInterface::getColorName(kGripperObjectRef->getColor()) == rcolor));
        
        // if the not modifier is enabled, negate the value of the return value (rv)
        if (notModifierEnabled)
            rv = !rv;

        // 
        if ((ObjectKeeper::isValid = rv) == true) {
            ObjectKeeper::tempObject
                = ShapeRootTypeConst(kWorldState.thisAgent.gripperObject, CylinderData);
        }

        return rv;
    }

    const std::string& KoduConditionGot::getObjectColor() const {
        return objColor;
    }

    const std::string& KoduConditionGot::getObjectType() const {
        return objType;
    }

    //************* temp fix
    const DualCoding::ShapeRoot KoduConditionGot::getTargetObject() {
        return ObjectKeeper::tempObject;
    }
    //*************

    void KoduConditionGot::reinitialize() {
        KoduCondition::reinitialize();
    }
    
    void KoduConditionGot::printAttrs() const {
        KoduCondition::printAttrs();
        // not enabled?
        PRINT_ATTRS("Not enabled", notModifierEnabled);
        // object color and type
        std::cout << "Object color and type: " << objColor << " " << objType << std::endl;
    }
}
