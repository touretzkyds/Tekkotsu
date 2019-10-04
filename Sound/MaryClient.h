#ifndef _MaryClient_h_
#define _MaryClient_h_
#if !defined(PLATFORM_APERIOS) && !defined(__APPLE__)

#include <string>

void launchMaryServer();

int maryQuery(std::string& result,
	      const std::string& inputText,
	      const std::string& voice = "female");

#endif
#endif
