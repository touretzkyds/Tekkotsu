#ifndef KODU_ACTION_PLAY_H_
#define KODU_ACTION_PLAY_H_

// C++ Library
#include <iostream>
#include <string>

// Kodu Library
#include "Kodu/Primitives/KoduAction.h"
#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Generators/KoduGenerators.h"

// TODO
// (25/01/13) find other modifiers for play action

namespace Kodu {

    //! Kodu Action Play class (derived from Kodu Action)
    class KoduActionPlay : public KoduAction {
    public:
        //! Constructor
        KoduActionPlay(const LiteralGenerator& kSoundFile, bool useOnce)
          : KoduAction ("KoduActionPlay", KoduAction::AT_PLAY, true, useOnce),
            soundFile(kSoundFile)//,
            // onceModifierEnabled(useOnce)
        { }
        
        //! Copy constructor
        KoduActionPlay(const KoduActionPlay& kAction)
          : KoduAction(kAction),
            soundFile(kAction.soundFile)//,
            // onceModifierEnabled(kAction.onceModifierEnabled)
        { }

        //! Destructor
        ~KoduActionPlay() {
            // no explicit implementation
        }
        
        //! Assignment operator
        KoduActionPlay& operator=(const KoduActionPlay& kAction) {
            if (this != &kAction) {
                KoduAction::operator=(kAction);
                soundFile = kAction.soundFile;
                // onceModifierEnabled = kAction.onceModifierEnabled;
            }
            return *this;
        }

        //! Returns the name of the sound file
        const std::string& getSoundFile();
        
        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive*);

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();

        //! Prints the attributes of a particular instance
        virtual void printAttrs() const;

    private:
        LiteralGenerator soundFile;     //!< Returns the name of the sound file the agent needs to play
        // bool onceModifierEnabled;       //!< States if to play a sound file once
    };
}

#endif // KODU_ACTION_PLAY_H_
