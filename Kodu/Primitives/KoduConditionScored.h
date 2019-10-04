#ifndef KODU_CONDITION_SCORED_H_
#define KODU_CONDITION_SCORED_H_

// Kodu Library
#include "Kodu/Generators/KoduGenerators.h"
#include "Kodu/Primitives/KoduCondition.h"

// TODO
// (28/03/13) implement the random modifier (needs an initial value and increment?)

/**
 * NAME:            KODU CONDITION SCORED
 * TYPE:            CONDITION
 * KODU EQUIVALENT: SCORED
 * DESCRIPTION:
 *      This class is used to monitor the global score board.
**/
    
namespace Kodu {

    class KoduWorld;

    //! Kodu Condition Scored (derived from Kodu Condition)
    class KoduConditionScored : public KoduCondition {
    public:
        //! How the target score should be compared with the current (global) score
        enum CompareType_t {
					  INVALID         = 0L,       // should never be used
            CT_EQUALS       = 1L << 0,  // default choice unless otherwise specified
            CT_NOT_EQUALS   = 1L << 1,  // evaluate true if score != target value
            CT_GT_EQUAL     = 1L << 2,  // evaluate true if score >= target value
            CT_LT_EQUAL     = 1L << 3,  // evaluate true if score <= target value
            CT_ABOVE        = 1L << 4,  // evaluate true if score > target value
            CT_BELOW        = 1L << 5   // evaluate true if score < target value
        };

        //! Constructor
        KoduConditionScored(bool useNot, CompareType_t compType, const NumericGenerator& kNumericGen, 
            const std::string& kDesignator, bool numericModWasSpecified)
          : KoduCondition ("KoduConditionScored", KoduCondition::CT_SCORED),
            notModifierEnabled(useNot),
            type(compType),
            numericGen(kNumericGen),
            designator(kDesignator),
            numericsWereSpecified(numericModWasSpecified),
            lastRecordedValue(0)
        { }
        
        //! Copy constructor
        KoduConditionScored(const KoduConditionScored& kCondition)
          : KoduCondition(kCondition),
            notModifierEnabled(kCondition.notModifierEnabled),
            type(kCondition.type),
            numericGen(kCondition.numericGen),
            designator(kCondition.designator),
            numericsWereSpecified(kCondition.numericsWereSpecified),
            lastRecordedValue(kCondition.lastRecordedValue)
        { }

        //! Destructor
        ~KoduConditionScored() {
            // no explicit implementation
        }

        //! Assignment operator
        KoduConditionScored& operator=(const KoduConditionScored& kCondition) {
            if (this != &kCondition) {
                KoduCondition::operator=(kCondition);
                notModifierEnabled = kCondition.notModifierEnabled;
                type = kCondition.type;
                numericGen = kCondition.numericGen;
                designator = kCondition.designator;
                numericsWereSpecified = kCondition.numericsWereSpecified;
                lastRecordedValue = kCondition.lastRecordedValue;
            }
            return *this;
        }
        
        //! Compares the target score against the current (global) score
        virtual bool evaluate(const KoduWorld&);
        
        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive*);

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;

    private:
        bool notModifierEnabled; //!< if true, NOT operator is applied to the evaluate function's result
        CompareType_t type;     //!< states how the target value and current score are compared
        NumericGenerator numericGen; //!< returns a constant or random numeric value (the target score)
        std::string designator; //!< the score type (the key to the Score Keeper map)
        bool numericsWereSpecified; //!< if false, this condition returns true when a certin score changes
        int lastRecordedValue;//!< used in conjunction with numericsWereSpecified to monitor score changes
    };
}

#endif // KODU_CONDITION_SCORED_H_
