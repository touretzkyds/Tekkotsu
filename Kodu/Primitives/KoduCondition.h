#ifndef KODU_CONDITION_H_
#define KODU_CONDITION_H_

// C++ Library
#include <string>

// Kodu Library
#include "Kodu/Primitives/KoduPrimitive.h"

#include "Kodu/General/KoduState.h"

namespace Kodu {

    class KoduWorld;

    //! Kodu Condition (derived from Kodu Primitive)
    class KoduCondition : public KoduPrimitive {
    public:
        enum ConditionTypes {
            CT_ALWAYS = 0,
            CT_BUMP,
            CT_GOT,
            CT_SCORED,
            CT_SEE,
            CT_TIMER,
            CT_GAMEPAD,
            CT_HEAR
        };

        //! Constructor
        KoduCondition(const std::string& kConditionName, ConditionTypes condType)
          : KoduPrimitive(kConditionName), conditionType(condType), state()

        { }
        
        //! Copy constructor
        KoduCondition(const KoduCondition& kCondition)
          : KoduPrimitive(kCondition),
					conditionType(kCondition.conditionType), state(kCondition.state)
        { }

        //! Destructor
        virtual ~KoduCondition() {
            // no explicit implementation
        }

        //! Assignment operator
        KoduCondition& operator=(const KoduCondition& kCondition) {
            if (this != &kCondition) {
                KoduPrimitive::operator=(kCondition);
                conditionType = kCondition.conditionType;
								state = kCondition.state;
            }
            return *this;
        }
        
        //! Evaluates the event portion of the rule (implementation in derived classes)
        virtual bool evaluate(const KoduWorld&) = 0;
        
        bool canEvaluate() const {
            return KoduPrimitive::agentCanUsePrimitive();
        }

        //! Used to reinitialize certain variables during, for example, switching to another page
        virtual void reinitialize() {
            KoduPrimitive::reinitialize();
        }

        //! Returns the condition type
        ConditionTypes getConditionType() const {
            return conditionType;
        }
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const {
            KoduPrimitive::printAttrs();
        }
        
        //Add a state give condition knowlege of state
        void setKoduState(const KoduState& curstate) {
					state = curstate;
				}

				const KoduState& getState() const { return state; }

    protected:
        ConditionTypes conditionType;       //!< the condition type
				KoduState state;
    };
} // end of Kodu namespace

#endif // KODU_CONDITION_H_
