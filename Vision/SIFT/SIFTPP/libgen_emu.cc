#include "libgen_emu.h"
#ifdef PLATFORM_APERIOS
#include <sys/syslimits.h> // limits.h doesn't have PATH_MAX
#else
#include <limits.h>
#endif
#include <string.h>

static char staticStorage[PATH_MAX];

char* libgenEmu_basename(const char* path){
	char* buffer = staticStorage;
	if (path == '\0' || path[0] == '\0'){
		// NULL pointer or empty string
		strcpy(buffer, ".");
		return buffer;
	}else if (path[1] == '\0' && path[0] == '/'){
		// path is "/"
		strcpy(buffer, "/");
		return buffer;
	}else{
		strcpy(buffer, path);
		char* ptr = buffer + strlen(buffer);
		ptr--;
		// delete all trailing '/'
		while (ptr >= buffer && *ptr == '/'){
			*ptr = '\0';
			ptr--;
		}
		// scan backwards for '/'
		while (ptr >= buffer){
			if (*ptr == '/') return ++ptr;
			ptr--;
		}
		return buffer;
	}
}
