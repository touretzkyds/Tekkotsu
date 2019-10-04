#define TK_ENABLE_THREADING
#define TK_ENABLE_EROUTER
#define TK_ENABLE_SOUTSERR
#include "local/minisim.h"

// This could be a little simpler if you wanted to run as a 'dedicated' program
// instead of trying to run as a Behavior (adds a lot of extra dependencies)

#include "Behaviors/Demos/MirageTargetBehavior.h"
#include "Events/EventRouter.h"
#include "IPC/Thread.h"
#include "Shared/debuget.h"
#include <iostream>

class Block : public Thread {
public:
	Block() : Thread() { start(); }
protected:
	void* run() { char c; std::cin.read(&c,1); return NULL; }
};

int main(const int argc, const char* argv[]) {
	minisim::initialize();
	
	MirageTargetBehavior mirage;
	mirage.setAutoDelete(false);
	for(int i=1; i<argc; ++i) {
		if(mirage.apply(argv[i]) != 0) {
			minisim::destruct();
			return 2;
		}
	}
	
	Block b;
	mirage.start();
	while(b.isStarted() && mirage.isActive()) {
		usleep( (erouter->getNextTimer()-get_time())*1000 );
		erouter->processTimers();
	}
	mirage.stop();
	
	minisim::destruct();
	return 0;
}	

// To satisfy linkage for unused stuff without bringing in the full translation unit...
namespace ProjectInterface {
	bool displayException(const char * file, int line, const char * message, const std::exception* ex) {
		if(file!=NULL) {
			printf("Exception caught at %s:%d => ",debuget::extractFilename(file),line);
		} else {
			printf("Exception => ");
		}
		if(ex!=NULL) {
			printf("'%s'",ex->what());
		} else {
			printf("'%s'","Unknown type");
		}
		if(message!=NULL) {
			printf(" (%s)\n",message);
		} else {
			printf("\n");
		}
		printf("\tWhen running in gdb, try 'catch throw' to break where exceptions are first thrown.\n");
		return true;
	}
	bool (*uncaughtException)(const char * file, int line, const char * message, const std::exception* ex)=&displayException;
}

// to satisfy linker requirements without pulling in even more stuff... we're not really using this
#include "Motion/MotionManager.h"
unsigned short MotionManager::doAddMotion(SharedObjectBase const&, bool, float) { return static_cast<unsigned short>(-1); }
void MotionManager::removeMotion(unsigned short) {}
const float MotionManager::kStdPriority=10;
MotionManager * motman;

