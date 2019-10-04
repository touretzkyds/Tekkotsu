#ifndef KODU_CONDITION_HEAR_H_
#define KODU_CONDITION_HEAR_H_

// INCLUDES
// tekkodu Library
#include "Kodu/Primitives/KoduCondition.h"

// C++ Library
#include <iostream>

namespace Kodu {

    // forward declarations
    class KoduWorld;

    class KoduConditionHear : public KoduCondition {
    public:
        //! Constructor
			KoduConditionHear(bool useNot, const std::string& kObjectType, const std::string& kObjectColor, const std::string& kSaid)
        : KoduCondition("KoduConditionHear", KoduCondition::CT_HEAR),
            notModifierEnabled(useNot),
            objType(kObjectType),
				    objColor(kObjectColor),
    				said(kSaid)
        { }

        //! Copy constructor
        KoduConditionHear(const KoduConditionHear& kCondition)
          : KoduCondition(kCondition),
            notModifierEnabled(kCondition.notModifierEnabled),
            objType(kCondition.objType),
					  objColor(kCondition.objColor),
					  said(kCondition.said)
        { }

        //! Destructor
        ~KoduConditionHear() {
            // no explicit implementation
        }

        //! Assignment operator
        KoduConditionHear& operator=(const KoduConditionHear& kCondition) {
            if (this != &kCondition) {
                KoduCondition::operator=(kCondition);
                notModifierEnabled = kCondition.notModifierEnabled;
                objType = kCondition.objType;
                objColor = kCondition.objColor;
								said = kCondition.said;
            }
            return *this;
        }

        //! Checks if a specified object was bumped
        virtual bool evaluate(const KoduWorld&);

        //! Returns a specified object's color
        const std::string& getObjectColor() const { return objColor; }

        //! Returns a specified object's type
        const std::string& getObjectType() const { return objType; }

        //! Returns the target object... if none is available, return an invalid object
        const DualCoding::ShapeRoot getTargetObject();

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;
        
    private:
        bool notModifierEnabled;
        std::string objType;
        std::string objColor;
				std::string said;
    };
}

#endif // KODU_CONDITION_HEAR_H_
