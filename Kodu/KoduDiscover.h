
#ifndef INCLUDED_KoduDiscover_h_
#define INCLUDED_KoduDiscover_h_

#include "Events/EventBase.h"
#include "Events/EventListener.h"
#include "Wireless/Wireless.h"
#include <map>
#include <string>

using namespace std;

namespace KoduType {
    typedef enum {
        INVALID = 0,
        KODU,
        CYCLE,
        OCTOPUS,
        TURTLE,
        NUM_TYPES
    } player_type;

    extern string strs[];
}

typedef struct {
    int hostAddr;
    KoduType::player_type type;
} player_identity;

typedef void (*ident_callback)(player_identity *ident, void *cb_arg);

class KoduDiscover : public EventListener {
    public:
        KoduDiscover(player_identity& ident, const string& iface);
        ~KoduDiscover();

        void setNewRobotCallback(ident_callback new_robot, void *cb_arg);

        static map<int, player_identity> players;
    private:
        static player_identity my_identity;
        static Socket *discover_sock;
        static Socket *broadcast_sock;
        static void *callback_arg;
        static ident_callback new_robot;
        static int discover_rcv(char* buf, int size);
        void processEvent(const EventBase& event);
};

#endif
