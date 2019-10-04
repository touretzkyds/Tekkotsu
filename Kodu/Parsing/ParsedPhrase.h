#ifndef PARSED_PHRASE_H_
#define PARSED_PHRASE_H_

// C++ Library
#include <vector>

// Kodu Library
#include "Kodu/General/GeneralFncs.h"
#include "Kodu/Parsing/Token.h"

namespace Kodu {
    
    //! Phrase class
    class ParsedPhrase {
    private:
        TokenBase* head;                    //!< The condition/action type that was parsed from the text file
        std::vector<TokenBase*> modifiers;  //!< The modifiers for *head
        
    public:
        //! Constructor
        ParsedPhrase()
          : head(NULL), modifiers()
        { }
        
        //! Copy constructor
        explicit ParsedPhrase(const ParsedPhrase& kPhrase)
          : head(kPhrase.head), modifiers(kPhrase.modifiers)
        { }

        //! Destructor
        ~ParsedPhrase() {
            if (head != NULL) {
                delete head;
                head = NULL;
            }
            GeneralFncs::destroyAllPtrsInVector(modifiers);
        }
        
        //! Assignment operator
        ParsedPhrase& operator=(const ParsedPhrase& kPhrase) {
            if (this != &kPhrase) {
                head = kPhrase.head;
                modifiers = kPhrase.modifiers;
            }
            return *this;
        }

        //! Returns the head of the Phrase
        TokenBase* getPhraseHead();

        //! Returns the modifiers of the Phrase
        std::vector<TokenBase*>& getPhraseModifiers();
        
        //! Sets the head
        bool setPhraseHead(TokenBase* _head);
        
        //! Sets the modifiers
        bool setPhraseModifiers(const std::vector<TokenBase*>&);
    };
}

#endif // PARSED_PHRASE_H_