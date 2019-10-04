
#include "Events/EventBase.h"
#include "Events/EventListener.h"
#include "Events/EventRouter.h"
#include "Wireless/Wireless.h"
#include "Kodu/KoduDiscover.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

map<int, player_identity> KoduDiscover::players;
player_identity KoduDiscover::my_identity;
Socket * KoduDiscover::discover_sock;
Socket * KoduDiscover::broadcast_sock;
ident_callback KoduDiscover::new_robot;
void* KoduDiscover::callback_arg;

string KoduType::strs[] = {
    "invalid",
    "kodu",
    "cycle",
    "octopus",
    "turtle"
};

KoduDiscover::KoduDiscover(player_identity& ident, const string& iface) {
    ident.hostAddr = wireless->getIFAddress((const char*)iface.c_str());
    std::cout << "Got address " << ident.hostAddr << " for interface " << iface.c_str() << std::endl;
    my_identity = ident;
    broadcast_sock = wireless->socket(Socket::SOCK_DGRAM, 1024, 1024);
    discover_sock = wireless->socket(Socket::SOCK_DGRAM, 1024, 1024);
    wireless->setReceiver(discover_sock->sock, &discover_rcv);
    wireless->listen(discover_sock->sock, 4567); //FIXME configuration variable
    new_robot = NULL;
    callback_arg = NULL;
    //Add timer callback for sending discover messages
    erouter->addTimer(this, 1, 2000, true);
}

KoduDiscover::~KoduDiscover() {
    erouter->removeTimer(this);
    wireless->setReceiver(discover_sock->sock, (int (*)(char*,int))NULL);
    wireless->close(broadcast_sock);
    wireless->close(discover_sock);
}

void KoduDiscover::setNewRobotCallback(ident_callback new_robot_cb, void *cb_arg) {
    new_robot = new_robot_cb;
    callback_arg = cb_arg;
}

int KoduDiscover::discover_rcv(char* buf, int size) {
    if (size != sizeof(player_identity)) {
        cerr << "Badly sized discover packet" << endl;
    }
    player_identity *ident = (player_identity*)buf;
    if (players.find(ident->hostAddr) == players.end()) {
        if (new_robot != NULL && ident->hostAddr != my_identity.hostAddr) {
            new_robot(ident, callback_arg);
        }
    }
    players[ident->hostAddr] = *ident;
    cout << "Got discover packet from " << ident->hostAddr << " of type " << ident->type << endl;

    return 0;
}

void KoduDiscover::processEvent(const EventBase& event) {
    if (event.getGeneratorID() == EventBase::timerEGID) {
        wireless->connect(broadcast_sock->sock, "255.255.255.255", 4567);
        broadcast_sock->write((const byte*)&my_identity, sizeof(player_identity));

    }
}
