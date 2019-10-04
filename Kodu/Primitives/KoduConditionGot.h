#ifndef KODU_CONDITION_GOT_H_
#define KODU_CONDITION_GOT_H_

// INCLUDES
// tekkodu Library
#include "Kodu/Primitives/KoduCondition.h"

// Tekkotsu Library
#include "DualCoding/ShapeRoot.h"

// C++ Library
#include <iostream>

namespace Kodu {

    // forward declarations
    class KoduWorld;

    class KoduConditionGot : public KoduCondition {
    public:
        //! Constructor
        KoduConditionGot(bool useNot, const std::string& kObjectType, const std::string& kObjectColor)
          : KoduCondition("KoduConditionGot", KoduCondition::CT_GOT),
            notModifierEnabled(useNot),
            objType(kObjectType),
            objColor(kObjectColor)
        { }

        //! Copy constructor
        KoduConditionGot(const KoduConditionGot& kCondition)
          : KoduCondition(kCondition),
            notModifierEnabled(kCondition.notModifierEnabled),
            objType(kCondition.objType),
            objColor(kCondition.objColor)
        { }

        //! Destructor
        ~KoduConditionGot() {
            // no explicit implementation
        }

        //! Assignment operator
        KoduConditionGot& operator=(const KoduConditionGot& kCondition) {
            if (this != &kCondition) {
                KoduCondition::operator=(kCondition);
                notModifierEnabled = kCondition.notModifierEnabled;
                objType = kCondition.objType;
                objColor = kCondition.objColor;
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

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;
        
    private:
        bool notModifierEnabled;
        std::string objType;
        std::string objColor;
    };
}

#endif // KODU_CONDITION_GOT_H_
