#include "Shared/StackTrace.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#ifdef __cplusplus
using namespace stacktrace;
#endif

void trace() {
	printf("Current trace\n");
	displayCurrentStackTrace(-1U,0);
	
	const unsigned int limit=5;
	printf("limited to %d\n",limit);
	struct StackFrame * f = recordStackTrace(limit,0);
	displayStackTrace(f);
	
	printf("Record over:\n");
	struct StackFrame * rem = recordOverStackTrace(f,0);
	displayStackTrace(f);
	freeStackTrace(rem);
	freeStackTrace(f);
	
	printf("Record over (with extra):\n");
	f = allocateStackTrace(50);
	rem = recordOverStackTrace(f,0);
	displayStackTrace(f);
	freeStackTrace(rem);
	freeStackTrace(f);
}

int bar(int n) {
	printf("Bar %d\n",n);
	if(n>0)
	  return n+bar(n-1);
	trace();
	return 0;
}

void foo() {
	int total = bar(4);
	printf("Total %d\n",total);
}

#ifdef __APPLE__
void checkLeaks() {
	int pid = getpid();
	char leakscmd[50];
	snprintf(leakscmd,50,"leaks %d",pid);
	printf("Testing for leaks: %s\n",leakscmd);
	system(leakscmd);
	printf("Press any key to continue (may want to attach MallocDebug...)\n");
	getc(stdin);
}
#endif

int main(int argc, char** argv) {
	unsigned int i=0,cnt=1;
	if(argc>1)
		cnt=atoi(argv[1]);
	for(i=0; i<cnt; ++i)
		foo();
#ifdef __APPLE__
	atexit(checkLeaks);
#endif
	return 0;
}

