#ifndef KODU_ACTION_GRAB_H_
#define KODU_ACTION_GRAB_H_

// Tekkodu Library
#include "Kodu/Primitives/KoduAction.h"

// Tekkotsu Library
#include "DualCoding/ShapeRoot.h"

namespace Kodu {

    //! Kodu Action Grab (derived from Kodu Action)
    class KoduActionGrab : public KoduAction {
    public:
        //! Constructor
        KoduActionGrab(bool actionIsUsingIt, bool useOnce)
          : KoduAction("KoduActionGrab", KoduAction::AT_GRAB, true, useOnce),
            usingItModifier(actionIsUsingIt)
        { }

        //! Copy constructor
        KoduActionGrab(const KoduActionGrab& kAction)
          : KoduAction(kAction),
            usingItModifier(kAction.usingItModifier)
        { }

        //! Destructor
        ~KoduActionGrab() {
            // no explicit implementation
        }

        //! Assignment operator
        KoduActionGrab& operator=(const KoduActionGrab& kAction) {
            if (this != & kAction) {
                KoduAction::operator=(kAction);
                usingItModifier = kAction.usingItModifier;
            }
            return *this;
        }

        //! Returns the target object (the object to grab)
        const DualCoding::ShapeRoot getTargetObject() const;

        //! Tests if the primitive argument is the same as the clling class
        static bool isSameTypeAs(const KoduPrimitive*);

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();

        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;

    private:
        bool usingItModifier;   //!< States whether or not the "it" modifier was specified (by the user)
    };

}

#endif // end of KODU_ACTION_GRAB_H_
