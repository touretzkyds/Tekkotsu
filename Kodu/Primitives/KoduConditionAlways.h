#ifndef KODU_CONDITION_ALWAYS_H_
#define KODU_CONDITION_ALWAYS_H_

// Kodu Library
#include "Kodu/KoduWorld.h"
#include "Kodu/Primitives/KoduCondition.h"

namespace Kodu {
    
    class KoduWorld;

    //! Kodu Condition Always (derived from Kodu Condition)
    class KoduConditionAlways : public KoduCondition {
    public:
        //! Constructor
        KoduConditionAlways()
          : KoduCondition ("KoduConditionAlways", KoduCondition::CT_ALWAYS)
        { }

        //! Copy constructor
        KoduConditionAlways(const KoduConditionAlways& kCondition)
          : KoduCondition(kCondition)
        { }
        
        //! Destructor
        ~KoduConditionAlways () {
            // no explicit implementation
        }
        
        //! Assignment operator
        KoduConditionAlways& operator=(const KoduConditionAlways& kCondition) {
            if (this != &kCondition) {
                KoduCondition::operator=(kCondition);
            }
            return *this;
        }

        //! Evaluates true (always)
        virtual bool evaluate(const KoduWorld& kWorldState) {
            return true;
        }

        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive* kPrimitive) {
            return (dynamic_cast<const KoduConditionAlways*>(kPrimitive) != NULL);
        }
        
        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize() {
            KoduCondition::reinitialize();
        }
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const {
            KoduCondition::printAttrs();
        }
    };
} // end of Kodu namespace

#endif // KODU_CONDITION_ALWAYS_H_
