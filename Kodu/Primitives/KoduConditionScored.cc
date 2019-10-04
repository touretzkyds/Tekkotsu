// Tekkodu Library
#include "Kodu/KoduWorld.h"
#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Primitives/KoduConditionScored.h"

namespace Kodu {

    bool KoduConditionScored::evaluate(const KoduWorld& kWorldState) {
        // boolean return value
        bool rv = false;
        // get the latest score
        int score = KoduWorld::getScoreValue(designator);
        if (numericsWereSpecified) {
            // get a (constant/random) value from the numeric generator
            int value = numericGen.getNumericValue();
            // check the inequality type
            // equals
            if (type == CT_EQUALS && score == value) {
                rv = true;
            }
            // not equals
            else if (type == CT_NOT_EQUALS && score != value) {
                rv = true;
            }
            // greater than or equal to
            else if (type == CT_GT_EQUAL && score >= value) {
                rv = true;
            }
            // lesser than or equal to
            else if (type == CT_LT_EQUAL && score <= value) {
                rv = true;
            }
            // above
            else if (type == CT_ABOVE && score > value) {
                rv = true;
            }
            // below
            else if (type == CT_BELOW && score < value) {
                rv = true;
            }
        }
        // numerics were not specified
        else {
            if (score != lastRecordedValue) {
                lastRecordedValue = score;
                rv = true;
            }
        }
        // check if the "not" modifier is enabled
        if (notModifierEnabled)
            rv = !rv;
        return rv;
    }
    
    bool KoduConditionScored::isSameTypeAs(const KoduPrimitive* kPrimitive) {
        return (dynamic_cast<const KoduConditionScored*>(kPrimitive) != NULL);
    }

    void KoduConditionScored::reinitialize() {
        KoduCondition::reinitialize();
    }

    void KoduConditionScored::printAttrs() const {
        KoduCondition::printAttrs();
        PRINT_ATTRS("Not enabled", notModifierEnabled);
        PRINT_ATTRS("Numerics were specified", numericsWereSpecified);
        std::cout << "Inequality: ";
        switch (type) {
            case CT_EQUALS:
                std::cout << "equals";
                break;

            case CT_NOT_EQUALS:
                std::cout << "not equals";
                break;

            case CT_GT_EQUAL:
                std::cout << "greater than or equal to (>=)";
                break;
            
            case CT_LT_EQUAL:
                std::cout << "lesser than or equal to (<=)";
                break;

            case CT_ABOVE:
                std::cout << "above";
                break;

            case CT_BELOW:
                std::cout << "below";
                break;
				    case INVALID:
							std::cout << "**invliad**";
        }
        std::cout << std::endl;
        numericGen.printAttrs();
        std::cout << "Scored designator: " << designator << std::endl;
    }
} // end of Kodu namespace
