#ifndef KODU_CONDITION_BUMP_H_
#define KODU_CONDITION_BUMP_H_

// Tekkodu Library
#include "Kodu/Primitives/KoduCondition.h"
#include "Kodu/Primitives/PerceptionSearch.h"
#include "Kodu/Keepers/ObjectKeeper.h"

// Tekkotsu Library
#include "DualCoding/ShapeRoot.h"

// C++ Library
#include <iostream>

namespace Kodu {

    class KoduWorld;

    class KoduConditionBump : public KoduCondition {
    public:
        //! Constructor
        KoduConditionBump(bool useNot, const std::string& kObjectType, const std::string& kObjectColor,
            SearchLocation_t locationToSearch)
          : KoduCondition("KoduConditionBump", KoduCondition::CT_BUMP),
            notModifierEnabled(useNot),
            objType(kObjectType),
            objColor(kObjectColor),
            searchLocation(locationToSearch),
            refdObject(),
            agentLastPosAfterDetection(),
            visuallyDetectedBump(false)
        { }

        //! Copy constructor
        KoduConditionBump(const KoduConditionBump& kCondition)
          : KoduCondition(kCondition),
            notModifierEnabled(kCondition.notModifierEnabled),
            objType(kCondition.objType),
            objColor(kCondition.objColor),
            searchLocation(kCondition.searchLocation),
            refdObject(kCondition.refdObject),
            agentLastPosAfterDetection(kCondition.agentLastPosAfterDetection),
            visuallyDetectedBump(kCondition.visuallyDetectedBump)
        { }

        //! Destructor
        ~KoduConditionBump() {
            // no explicit implementation
        }

        //! Assignment operator
        KoduConditionBump& operator=(const KoduConditionBump& kCondition) {
            if (this != &kCondition) {
                KoduCondition::operator=(kCondition);
                notModifierEnabled = kCondition.notModifierEnabled;
                objType = kCondition.objType;
                objColor = kCondition.objColor;
                searchLocation = kCondition.searchLocation;
                refdObject = kCondition.refdObject;
                agentLastPosAfterDetection = kCondition.agentLastPosAfterDetection;
                visuallyDetectedBump = kCondition.visuallyDetectedBump;
            }
            return *this;
        }

        //! Checks if a specified object was bumped
        virtual bool evaluate(const KoduWorld&);

        //! Returns a specified object's color
        const std::string& getObjectColor() const;

        //! Returns a specified object's type
        const std::string& getObjectType() const;

        //! Returns the target object... if none is available, return an invalid object
        const DualCoding::ShapeRoot getTargetObject();

        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive*);

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;

        //******************* temp fix
        bool agentIsNearMatchingObject(const DualCoding::ShapeRoot&);
        //*******************
        //bool agentIsNearMatchingObject();

        void setVisualBumpDetection(bool);

        //! The maximum distance the agent can be away from an object to sense a "bump"
        static const float kMaxDistanceAwayToSenseBump;
        
    private:
        bool notModifierEnabled;
        std::string objType;
        std::string objColor;
        SearchLocation_t searchLocation;
        DualCoding::ShapeRoot refdObject;
        DualCoding::Point agentLastPosAfterDetection;
        bool visuallyDetectedBump;
    };
}

#endif // KODU_CONDITION_BUMP_H_
