#define TK_ENABLE_EROUTER
#define TK_ENABLE_WIRELESS
#include "local/minisim.h"

#include "Behaviors/Mon/EchoBehavior.h"
#include "Events/TextMsgEvent.h"

#include <iostream>
#include <string>

using namespace std;
using namespace minisim;

int main(int argc, char** argv) {
	initialize(); //minisim function for global setup
	
	{//restricting scope so behavior is destructed before minisim::destruct is called

		EchoBehavior eb;
		eb.setAutoDelete(false); //needed when using stack allocation
		eb.start();
		
		for(string in; cin; getline(cin,in)) {
			if(in.size()>0) {
				erouter->postEvent(TextMsgEvent(in));
			}
		}
		
		eb.stop();

	}//restricting scope so behavior is destructed before minisim::destruct is called
	
	destruct(); //minisim function for global teardown
	return 0;
}

