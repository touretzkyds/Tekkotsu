// Kodu Library
#include "Kodu/KoduWorld.h"
#include "Kodu/Primitives/KoduConditionSee.h"
#include "Kodu/General/GeneralMacros.h"

namespace Kodu {

    bool KoduConditionSee::evaluate(const KoduWorld& kWorldState) {
        bool rv = false;
        DualCoding::ShapeRoot _refdObject;
        
        // get the closest object that matches what this condition is searching for
        _refdObject = getClosestObjectMatching(objColor, objType, searchLocation, kWorldState.thisAgent.gripperObject);
        
        // If there is one valid remaining, the robot will react to that object
        if (_refdObject.isValid()) {
					// std::cout << "Saw a(n) " << getObjectColor() << " " << getObjectType() << "!\n";
            ObjectKeeper::tempObject = _refdObject;
            ObjectKeeper::isValid = true;
            rv = true;
        }
        
        // check if the not modifier is enabled
        if (notModifierEnabled)
            return (!rv);
        else
            return rv;
    }

    const std::string& KoduConditionSee::getObjectColor() const {
        return objColor;
    }

    const std::string& KoduConditionSee::getObjectType() const {
        return objType;
    }

    const DualCoding::ShapeRoot KoduConditionSee::getTargetObject() {
        return refdObject;
    }

    bool KoduConditionSee::isSameTypeAs(const KoduPrimitive* kPrimitive) {
        return (dynamic_cast<const KoduConditionSee*>(kPrimitive) != NULL);
    }

    void KoduConditionSee::reinitialize() {
        KoduCondition::reinitialize();
    }
    
    void KoduConditionSee::printAttrs() const {
        KoduCondition::printAttrs();
        // not enabled?
        PRINT_ATTRS("Not enabled", notModifierEnabled);
        // object color and type
        std::cout << "Object color and type: " << objColor << " " << objType << std::endl;
        // search region
        /*
        std::cout << "Search region:";
        if (searchRegion == SR_UNRESTRICTED) {
            std::cout << " unrestricted\n";
        } else {
            if (searchRegion & SR_TO_LEFT) {
                std::cout << " to_left";
            } else if (searchRegion & SR_TO_RIGHT) {
                std::cout << " to_right";
            }

            if (searchRegion & SR_IN_FRONT) {
                std::cout << " in_front";
            } else if (searchRegion & SR_BEHIND) {
                std::cout << " behind";
            }
            std::cout << std::endl;
        }
        // search radius
        std::cout << "Search distance: ";
        switch (searchDistance) {
            case SD_UNRESTRICTED:
                std::cout << "unrestricted\n";
                break;

            case SD_CLOSE_BY:
                std::cout << "close_by\n";
                break;

            case SD_FAR_AWAY:
                std::cout << "far_away\n";
                break;
        }
        */
        // referenced object...
    }
}
