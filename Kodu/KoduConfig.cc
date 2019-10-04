/**
 * @file KoduConfig.cc
 *
 * @author med
 * @author asf
 */

#include <cstring>
#include <string>
#include <unistd.h>
#include "Kodu/KoduConfig.h"
#include "Shared/Config.h"

using namespace std;

unsigned int
KoduConfig::loadFile(const char* filename) {
    int ret = plist::Dictionary::loadFile((config->getFileSystemRoot()+"/"+string(filename)).c_str());
    if (!ret) return ret;

    this->kodu_type = KoduType::INVALID;

    for (unsigned int i = 0; i < KoduType::NUM_TYPES; i++) {
        if (this->type == KoduType::strs[i]) {
            this->kodu_type = (KoduType::player_type)i;
        }
    }

    return ret;
}
