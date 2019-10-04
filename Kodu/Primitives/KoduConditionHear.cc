// INCLUDES
// tekkotsu library
#include "DualCoding/ShapeRoot.h"

// tekkodu library
#include "Kodu/KoduWorld.h"
#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Primitives/KoduConditionHear.h"
#include "Kodu/KoduDiscover.h"

namespace Kodu {

    bool KoduConditionHear::evaluate(const KoduWorld& kWorldState) {
        for (size_t i = 0; i < state.heard_utt.size(); i++) {
            KoduState::utterance utt = state.heard_utt[i];
			std::map<int,player_identity>::iterator player = KoduDiscover::players.find(utt.hostAddr);
            if (utt.phrase == said
                    && player != KoduDiscover::players.end()
                    && (objType == "" || KoduType::strs[(player->second).type] == objType)) {
                //TODO also check color
                return true;
            }
        }
        return false;
    }

    void KoduConditionHear::reinitialize() {
        KoduCondition::reinitialize();
    }

    //************* temp fix
    const DualCoding::ShapeRoot KoduConditionHear::getTargetObject() {
        return ObjectKeeper::tempObject;
    }
    //*************


    void KoduConditionHear::printAttrs() const {
        KoduCondition::printAttrs();
        // not enabled?
        PRINT_ATTRS("Not enabled", notModifierEnabled);
        // object color and type
        std::cout << "Object color and type: " << objColor << " " << objType << std::endl;
				std::cout << "Said string: '" << said << "'" << std::endl;
    }


} // namespace
