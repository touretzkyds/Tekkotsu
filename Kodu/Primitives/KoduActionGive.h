#ifndef KODU_ACTION_GIVE_H_
#define KODU_ACTION_GIVE_H_

// INCLUDES
// tekkodu
#include "Kodu/Primitives/KoduAction.h"

namespace Kodu {

    //! Kodu Action Give class (derived from Kodu Action)
    class KoduActionGive : public KoduAction {
    public:
        //! Constructor
        KoduActionGive()
          : KoduAction ("KoduActionGive", KoduAction::AT_GIVE, false, false)
        { }

        //! Copy constructor
        KoduActionGive(const KoduActionGive& kAction)
          : KoduAction(kAction)
        { }
        
        //! Destructor
        ~KoduActionGive() {
            // no explicit implementation
        }
        
        //! Assignment operator
        KoduActionGive& operator=(const KoduActionGive& kAction) {
            if (this != &kAction) {
                KoduAction::operator=(kAction);
            }
            return *this;
        }
        
        //! Returns the target object (the object to grab)
        const DualCoding::ShapeRoot getTargetObject() const;

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();

        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;
    };
}

#endif // KODU_ACTION_GIVE_H_
