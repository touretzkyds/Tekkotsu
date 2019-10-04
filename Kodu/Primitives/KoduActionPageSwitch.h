#ifndef KODU_ACTION_PAGE_SWITCH_H_
#define KODU_ACTION_PAGE_SWITCH_H_

#include <iostream>

// Kodu Library
#include "Kodu/Primitives/KoduAction.h"
#include "Kodu/Generators/KoduGenerators.h"

// TODO

namespace Kodu {

    //! Kodu Action Page Switch class (derived from Kodu Action)
    class KoduActionPageSwitch : public KoduAction {
    public:
        //! Constructor
        KoduActionPageSwitch(const NumericGenerator& pageNumber)
          : KoduAction ("KoduActionPageSwitch", KoduAction::AT_PAGE_SWITCH, false, false),
            pageNumb(pageNumber)
        { }

        //! Copy constructor
        KoduActionPageSwitch(const KoduActionPageSwitch& kAction)
          : KoduAction(kAction),
            pageNumb(kAction.pageNumb)
        { }
        
        //! KoduActionPageSwitch
        ~KoduActionPageSwitch() {
            // no explicit implementation
        }
        
        //! Assignment operator
        KoduActionPageSwitch& operator=(const KoduActionPageSwitch& kAction) {
            if (this != &kAction) {
                KoduAction::operator=(kAction);
                pageNumb = kAction.pageNumb;
            }
            return *this;
        }

        //! Returns the new requested page number
        int getPageNumber();
        
        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive*);
        
        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();

        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;

    private:
        NumericGenerator pageNumb;
    };
}

#endif // KODU_ACTION_PAGE_SWITCH_H_
