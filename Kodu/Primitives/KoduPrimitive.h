#ifndef KODU_PRIMITIVE_H_
#define KODU_PRIMITIVE_H_

// Tekkodu Library
#include "Kodu/General/GeneralMacros.h"

// C++ Library
#include <iostream>
#include <string>

namespace Kodu {
    
    //! Kodu Primitive
    class KoduPrimitive {   
    public:
        //! Constructor
        KoduPrimitive(const std::string& kPrimitiveType)
          : primitiveType(kPrimitiveType),
            agentCanUseThisPrimitive(true)
        { }
        
        //! Copy constructor
        KoduPrimitive(const KoduPrimitive& kPrimitive)
          : primitiveType(kPrimitive.primitiveType),
            agentCanUseThisPrimitive(kPrimitive.agentCanUseThisPrimitive)
        { }

        //! Destructor
        virtual ~KoduPrimitive() {
            // no explicit implementation
        }
        
        //! Assignment operator
        KoduPrimitive& operator=(const KoduPrimitive& kPrimitive) {
            if (this != &kPrimitive) {
                primitiveType = kPrimitive.primitiveType;
                agentCanUseThisPrimitive = kPrimitive.agentCanUseThisPrimitive;
            }
            return *this;
        }

        //! Returns the name of the derived primitive
        const std::string& getPrimitiveType() const {
            return primitiveType;
        }
        
        //! Tests if the primitive argument is the same as the calling class
        static bool isSameTypeAs(const KoduPrimitive*);

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize() {
            agentCanUseThisPrimitive = true;
        }

        //! Prints the attributes for a particular primitive (pure virtual function)
        virtual void printAttrs() const {
            std::cout << "Primitive Type: " << primitiveType << std::endl;
            PRINT_ATTRS("Agent can use primitive", agentCanUseThisPrimitive);
        }

        bool agentCanUsePrimitive() const {
            return agentCanUseThisPrimitive;
        }

        void setAgentCanUsePrimitive(bool bval) {
            agentCanUseThisPrimitive = bval;
        }

    protected:
        std::string primitiveType;  //!< The name of the derived primitive
        bool agentCanUseThisPrimitive;
    };
} // end of Kodu namespace

#endif // KODU_PRIMITIVE_H_
