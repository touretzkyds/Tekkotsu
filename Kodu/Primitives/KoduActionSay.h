#ifndef KODU_ACTION_SAY_H_
#define KODU_ACTION_SAY_H_

// C++ Library
#include <iostream>
#include <string>

// Kodu Library
#include "Kodu/Primitives/KoduAction.h"
#include "Kodu/Generators/KoduGenerators.h"
#include "Kodu/General/GeneralMacros.h"

/**
 * NAME:            KODU ACTION SAY
 * TYPE:            ACTION
 * KODU EQUIVALENT: SAY
 * DESCRIPTION:
 *      This class is used to submit a string to speak each time this action fires.
 * 
 * DATA DESCRIPTION:
 *      - Speech strings: a vector of strings to speak.
 *      - Say request: holds the string that will be spoken when this action fires.
 *      - Order:
**/

namespace Kodu {

    //! Kodu Action Say class (derived from Kodu Speech Action)
    class KoduActionSay : public KoduAction {
    public:
        //! Constructor
        KoduActionSay(const std::string& kDesignator, const LiteralGenerator& kStringLiteral, bool useOnce)
          : KoduAction("KoduActionSay", KoduAction::AT_SAY, true, useOnce),
            designator(kDesignator),
            literalGen(kStringLiteral)//,
            // onceEnabled(useOnce)
        { }

        //! Copy constructor
        KoduActionSay(const KoduActionSay& kAction)
          : KoduAction(kAction),
            designator(kAction.designator),
            literalGen(kAction.literalGen)//,
            // onceEnabled(kAction.onceEnabled)
        { }
        
        //! Destructor
        ~KoduActionSay() {
            // no explicit implementation
        }
        
        //! Assignment operator
        KoduActionSay& operator=(const KoduActionSay& kAction) {
            if (this != &kAction) {
                KoduAction::operator=(kAction);
                designator = kAction.designator;
                literalGen = kAction.literalGen;
                // onceEnabled = kAction.onceEnabled;
            }
            return *this;
        }
        
        //! Returns a literal string to speak
        const std::string& getStringToSpeak();
        
        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive*);

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();
        
        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;

    private:
        std::string designator;
        LiteralGenerator literalGen;
        // bool onceEnabled;
    };
}

#endif // KODU_ACTION_SAY_H_
