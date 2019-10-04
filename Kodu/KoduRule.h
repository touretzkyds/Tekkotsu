#ifndef KODU_RULE_H_
#define KODU_RULE_H_

// Kodu Library
#include "Kodu/Primitives/KoduAction.h"
#include "Kodu/Primitives/KoduCondition.h"

namespace Kodu {
    //! Kodu Rule
    class KoduRule {
    public:
        unsigned int ruleNumber;            //!< Rule number
        unsigned int parent;                //!< Parent rule (assigned during program parsing)
        KoduCondition* condition;           //!< Condition to evaluate
        KoduAction* action;                 //!< Action to execute
        bool condLastEvalResult;            //!< Evaluation result for a condition (true|false)
        
        //! Constructor
        KoduRule(unsigned int kRuleNumber, unsigned int kParent)
            : ruleNumber(kRuleNumber),
            parent(kParent),
            condition(NULL),
            action(NULL),
            condLastEvalResult(false)
        { }
        
        //! Destructor
        ~KoduRule() {
            delete condition;
            delete action;
        }
        
        //! Get the rule number
        unsigned int getRuleNumber() const {
            return ruleNumber;
        }

        //! Get the parent number
        unsigned int getParentNumber() const {
            return parent;
        }

        //! Returns whether or not a rule is indented
        bool isIndented() const {
            return (getParentNumber() > 0);
        }

        //! Reinitializes a rule's primitives
        void reinitializePrimitives() {
            condLastEvalResult = false;
            condition->reinitialize();
            action->reinitialize();
        }

        //! Sets the condition evaluation result
        void setConditionEvalResult(bool bVal) {
            condLastEvalResult = bVal;
        }
        
    private:
        DISALLOW_COPY_ASSIGN(KoduRule);
    };
}
#endif // KODU_RULE_H_