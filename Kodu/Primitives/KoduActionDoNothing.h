#ifndef KODU_ACTION_DO_NOTHING_H_
#define KODU_ACTION_DO_NOTHING_H_

// Kodu Library
#include "Kodu/Primitives/KoduAction.h"

namespace Kodu {
    
    class KoduActionDoNothing : public KoduAction {
    public:
        //! Constructor
        KoduActionDoNothing()
          : KoduAction("KoduActionDoNothing", KoduAction::AT_DO_NOTHING, false, false)
        { }

        //! Copy constructor
        KoduActionDoNothing(const KoduActionDoNothing& kAction)
          : KoduAction(kAction)
        { }

        //! Destructor
        ~KoduActionDoNothing() {
            // no explicit implementation
        }

        //! Assignment operator
        KoduActionDoNothing& operator=(const KoduActionDoNothing& kAction) {
            if (this != &kAction) {
                KoduAction::operator=(kAction);
            }
            return *this;
        }

        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive* kPrimitive) {
            return (dynamic_cast<const KoduActionDoNothing*>(kPrimitive) != NULL);
        }

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize() {
            // nothing to do
        }
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const {
            KoduAction::printAttrs();
        }
    };
}

#endif // KODU_ACTION_DO_NOTHING_H_
