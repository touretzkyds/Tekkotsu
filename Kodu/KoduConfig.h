/**
 * @file KoduConfig.h
 *
 * @brief Kodu configuration class
 */

#ifndef INCLUDED_KoduConfig_h_
#define INCLUDED_KoduConfig_h_

#include "Shared/plist.h"
#include "Kodu/KoduDiscover.h"

class KoduConfig : public plist::Dictionary {
    private:
        plist::Primitive<string> type;
        
    public:
        KoduType::player_type kodu_type;
        plist::Primitive<string> interface;

        KoduConfig() : plist::Dictionary(false), type("kodu"),
        kodu_type(KoduType::KODU), interface("eth0")
        {
            addEntry("type",type,"The type of the kodu character");
            addEntry("interface",interface,"The network interface over which to discover other characters");
        }

        virtual unsigned int loadFile(const char* filename);
};

#endif
