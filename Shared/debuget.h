#ifndef INCLUDED_debuget_h
#define INCLUDED_debuget_h

#include <stdio.h>
#include <ctype.h>
#include <iostream>

#ifdef DEBUG
#include <string.h>
#include <sstream>
#include <fstream>
#endif

//! contains some debugging functions and #ASSERT macros (although the macros don't respect the namespace scoping...)
namespace debuget {
	//! for display, just use the filename, not the whole path
	inline const char* extractFilename(const char* path) {
		const char * last=path;
		while(*path++)
			if(*path=='/')
				last=path+1;
		return last;
	}
	
	//! mostly for use with a debugger -- set a breakpoint on this function and you can catch anytime an assertion is generated
	inline void displayAssert(const char* file, unsigned int line,const char* msg) { std::cerr << "ASSERT:"<<extractFilename(file)<<'.'<<line<<':'<< msg << std::endl; }
	
#ifdef DEBUG
	
	//! if the bool b is false, std::cout the string
	#define ASSERT(b,msgstream) {if(!(b)) { std::stringstream DEBUGET_ss; DEBUGET_ss << msgstream; debuget::displayAssert(__FILE__,__LINE__,DEBUGET_ss.str().c_str()); }}
	//! if the bool b is false, std::cout the string and return
	#define ASSERTRET(b,msgstream) {if(!(b)) { std::stringstream DEBUGET_ss; DEBUGET_ss << msgstream; debuget::displayAssert(__FILE__,__LINE__,DEBUGET_ss.str().c_str()); return; }}
	//! if the bool b is false, std::cout the string and return the value
	#define ASSERTRETVAL(b,msgstream,v) {if(!(b)) { std::stringstream DEBUGET_ss; DEBUGET_ss << msgstream; debuget::displayAssert(__FILE__,__LINE__,DEBUGET_ss.str().c_str()); return v; }}
	//! if the bool b is false, std::cout the string and exit(x)
	#define ASSERTFATAL(b,msgstream,x) {if(!(b)) { std::stringstream DEBUGET_ss; DEBUGET_ss << msgstream; debuget::displayAssert(__FILE__,__LINE__,DEBUGET_ss.str().c_str()); exit(x); }}
	//! sets up an 'if' statement so the following block won't be executed if the ASSERT fails
	#define ASSERTIF(b,msgstream) if(!(b)) { std::stringstream DEBUGET_ss; DEBUGET_ss << msgstream; debuget::displayAssert(__FILE__,__LINE__,DEBUGET_ss.str().c_str()); } else
	
#else

	//! if the bool b is false, std::cout the string
	#define ASSERT(b,msgstream) {}
	//! if the bool b is false, std::cout the string and return
	#define ASSERTRET(b,msgstream) {}
	//! if the bool b is false, std::cout the string and return the value
	#define ASSERTRETVAL(b,msgstream,v) {}
	//! if the bool b is false, std::cout the string and exit(x)
	#define ASSERTFATAL(b,msgstream,x) {}
	//! sets up an 'if' statement so the following block won't be executed if the ASSERT fails
	#define ASSERTIF(b,msgstream) {}
	
#endif

	//! returns the hex char that corresponds to @a c, which should be 0-16 (returns '.' otherwise)
	inline char hexdigit(int c) {
		if(c<0)
			return '.';
		if(c<10)
			return '0'+c;
		if(c<16)
			return 'a'+(c-10);
		return ',';
	}

	//! printf's the two hex digits coresponding to a byte
	inline void charhexout(char c) {
		printf("%c%c",hexdigit((c>>4)&0x0F),hexdigit(c&0x0F));
	}

	//! charhexout's @a n bytes starting at @a p
	inline void hexout(const void* p, size_t n) {
		printf("%p:\n",p);
		const char* x=(const char*)p;
		for(unsigned int i=0; i<n;) {
			printf("%6d ",i);
			for(unsigned int k=0; k<8 && i<n; k++) {
				for(unsigned int j=0; j<4 && i<n; j++, i++) {
					charhexout(x[i]);
					//				std::cout << flush;
				}
				printf(" ");
			}
			printf("\n");
		}
	}

	//! displays hex and ascii values of @a size bytes from @a p
	inline void hexout2(const void* p, size_t size) {
		const char* buf=static_cast<const char*>(p);
		size_t prev_line=0;
		const unsigned int cols=4;
		const unsigned int n_per_col=4;
		printf("Base: %p\n",buf);
		for(unsigned int i=0; i<size; i++) {
			if(i%(cols*n_per_col)==0)
				printf("%6u  |",i);
			printf("%02hhx",buf[i]);
			if((i+1)%(cols*n_per_col)==0) {
				printf("|  ");
				for(size_t j=prev_line; j<=i; j++)
					printf("%c",isprint(buf[j])?buf[j]:'.');
				prev_line=i+1;
				printf("\n");
			} else if((i+1)%(n_per_col)==0)
				printf(" ");
		}
		for(size_t i=size; i%(cols*n_per_col)!=0; i++) {
			printf("  ");
			if((i+1)%(cols*n_per_col)==0) {
				printf("|  ");
				for(size_t j=prev_line; j<size; j++)
					printf("%c",isprint(buf[j])?buf[j]:'.');
				prev_line=i;
				printf("\n");
			} else if((i+1)%(n_per_col)==0)
				printf(" ");
		}
	}

	//! displays hex and ascii values of @a size bytes from @a p
	inline void hexout3(const char* buf, size_t size) {
		const unsigned int linelen=24;
		for(unsigned int i=0; i<size; i++) {
			printf("%.2hhx",buf[i]);
			if((i+1)%linelen==0) {
				printf("   ");
				for(unsigned int j=i+1-linelen; j<=i; j++)
					putchar(isprint(buf[j])?buf[j]:'.');
				printf("\n");
			} else if((i+1)%4==0)
				printf(" ");
		}
		//finish off the last line (still need to do human-readable version)
		for(size_t i=size; i%linelen!=0; i++) {
			printf("  ");
			if((i+1)%linelen==0) {
				printf("   ");
				for(size_t j=(size/linelen)*linelen; j<size; j++)
					putchar(isprint(buf[j])?buf[j]:'.');
				printf("\n");
			} else if((i+1)%4==0)
				printf(" ");
		}
	}
}

/*! @file
 * @brief Defines several debugging functions and macros, including ::ASSERT (and variations)
 * @author ejt (Creator)
 */

#endif
