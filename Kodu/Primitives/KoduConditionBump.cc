// Kodu Library
#include "Kodu/KoduWorld.h"
#include "Kodu/Primitives/KoduConditionBump.h"
#include "Kodu/General/GeneralMacros.h"

namespace Kodu {

    const float KoduConditionBump::kMaxDistanceAwayToSenseBump = 100.0f; // millimeters
    
    bool KoduConditionBump::evaluate(const KoduWorld& kWorldState) {
        // check the following:
        // 1) the agent visually detected the bump,
        // 2) the referenced object (the object the bump condition will react to) is valid
        // 3) the agent has not moved anywhere (translated the body in the x-direction)
        //    (3) assumes that the agent's centroid measurement is correct.
        static float const kMaxDistAgentCanMoveAway = 20.0f;
        bool rv = (visuallyDetectedBump && refdObject.isValid()
            && distanceFromAgentToPoint(agentLastPosAfterDetection) <= kMaxDistAgentCanMoveAway);
        
        // if the not modifier is enabled, negate the value of the return value (rv)
        if (notModifierEnabled)
            rv = !rv;

        // 
        if ((ObjectKeeper::isValid = rv) == true)
            ObjectKeeper::tempObject = refdObject;

        return rv;
    }

    bool KoduConditionBump::agentIsNearMatchingObject(const DualCoding::ShapeRoot& kExcludedShape) {
        DualCoding::ShapeRoot obj;
        // get the closest object that matches what this condition is searching for
        obj = getClosestObjectMatching(objColor, objType, searchLocation, kExcludedShape);
        // If there is one valid remaining and it is within some distance to the agent,
        // then the robot will react to that object
				std::cout << "Bump condition:  closesObj=" << obj << "  dist="
									<< (obj.isValid() ? distanceInBetweenAgentAndObject(obj) : -1.0f) << " ... ";
        if (obj.isValid() && (distanceInBetweenAgentAndObject(obj) <= kMaxDistanceAwayToSenseBump)) {
            std::cout << "yes!\n";
            refdObject = obj;
            return true;
        } else {
            std::cout << "no bump.\n";
            refdObject = ObjectKeeper::invalidObject;
            return false;
        }
    }

    void KoduConditionBump::setVisualBumpDetection(bool bval) {
        visuallyDetectedBump = bval;
        // check if the agent visually detected the bump
        if (visuallyDetectedBump) {
            // if it did, note the last position the robot was at when it visually detected the object
            agentLastPosAfterDetection = DualCoding::VRmixin::theAgent->getCentroid();
            ObjectKeeper::tempObject = refdObject;
        }
    }

    const std::string& KoduConditionBump::getObjectColor() const {
        return objColor;
    }

    const std::string& KoduConditionBump::getObjectType() const {
        return objType;
    }

    const DualCoding::ShapeRoot KoduConditionBump::getTargetObject() {
        return refdObject;
    }

    bool KoduConditionBump::isSameTypeAs(const KoduPrimitive* kPrimitive) {
        return (dynamic_cast<const KoduConditionBump*>(kPrimitive) != NULL);
    }

    void KoduConditionBump::reinitialize() {
        KoduCondition::reinitialize();
    }
    
    void KoduConditionBump::printAttrs() const {
        KoduCondition::printAttrs();
        // not enabled?
        PRINT_ATTRS("Not enabled", notModifierEnabled);
        // object color and type
        std::cout << "Object color and type: " << objColor << " " << objType << std::endl;
        // search region
        /*
        std::cout << "Search region:";
        if (searchRegion == SRG_UNRESTRICTED) {
            std::cout << " unrestricted\n";
        } else {
            if (searchRegion & SRG_TO_LEFT) {
                std::cout << " to_left";
            } else if (searchRegion & SRG_TO_RIGHT) {
                std::cout << " to_right";
            }

            if (searchRegion & SRG_IN_FRONT) {
                std::cout << " in_front";
            } else if (searchRegion & SRG_BEHIND) {
                std::cout << " behind";
            }
            std::cout << std::endl;
        }
        */
        // referenced object...
    }
}
