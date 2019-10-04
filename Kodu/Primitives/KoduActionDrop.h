#ifndef KODU_ACTION_DROP_H_
#define KODU_ACTION_DROP_H_

// INCLUDES
// tekkodu
#include "Kodu/Primitives/KoduAction.h"

namespace Kodu {

    //! Kodu Action Drop class (derived from Kodu Action)
    class KoduActionDrop : public KoduAction {
    public:
        //! Constructor
        KoduActionDrop()
          : KoduAction ("KoduActionDrop", KoduAction::AT_DROP, false, false)
        { }

        //! Copy constructor
        KoduActionDrop(const KoduActionDrop& kAction)
          : KoduAction(kAction)
        { }
        
        //! Destructor
        ~KoduActionDrop() {
            // no explicit implementation
        }
        
        //! Assignment operator
        KoduActionDrop& operator=(const KoduActionDrop& kAction) {
            if (this != &kAction) {
                KoduAction::operator=(kAction);
            }
            return *this;
        }

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();

        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;
    };
}

#endif // KODU_ACTION_DROP_H_
