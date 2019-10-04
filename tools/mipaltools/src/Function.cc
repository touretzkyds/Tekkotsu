// -----------------------------------------------------------------------------
// File: Function.cc
// Date: 28/8/2003
// Part of StackedIt V2.0
// By: Joel Fenwick, Mi-Pal, Griffith University
//
// This program is copyrighted to the author and released under the GPL (V2.0).
// There is no warantee associated with this code. Especially not for use
// in nuclear plants, aircraft control systems or hedgehogs. Use at your
// own risk.
// See license.txt for details
// -----------------------------------------------------------------------------

#include "Function.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>	// Not so happy about this but sscanf is just too useful

// Extract name and starting address from the line
// line must be in the following form
// 00address <name>:
Function::Function(char* line)
{
	haveretoffset=false;
	stacksize=0;
	retoffset=0;
	start=0;
	linecount=0;
	int ln=strlen(line);			// had a lot of trouble with the string manip here
	name=new char[ln-11];			// may need to be tuned
	start=strtol((line+2),0,16);
	strncpy(name,line+10,ln-13);		// remove >: from the line
	name[ln-13]='\0';	
}

Function::~Function()
{
	delete name;
}

// This test isn't rigourous at the moment. I haven't tested whether it is always possible to 
// differentiate between source and function headers anyway
bool Function::IsHeader(char* line)
{
	// first two chars on the line must be 00
	if ((line==0) || (line[0]=='\0') || (line[1]=='\0') || (line[0]!='0') || (line[1]!='0'))
	{
		return false;
	}
	return true;
}

// Return true if the line has been processed
bool Function::Process(char* line)
{
	// Assumptions based on number of lines into the function can be thrown off by source
	// inclusions
	linecount++;			// line count should be the number of lines of assembly

	// normally the stack size would occur 4 lines into the function if at all
	// however source code interleaving may cause problems here
	if ((stacksize==0) && (linecount>=4))
	{
		stacksize=GetStackSize(line);
		// Assuming there are no other stack allocations
	}
	if ((retoffset==0) && (stacksize!=0))
	{
		retoffset=GetRetOffset(line);
		if (retoffset!=0)
		{
			haveretoffset=true;
		}
	}
	return true;
}

// Attempt to find the size of the stackframe from this line
//  4000fc:       27bdffd0        addiu   sp,sp,-48
unsigned int Function::GetStackSize(char* line)
{
	unsigned int size=0;
	// This could have been done more quickly with array indexes but this is clearer
	// I'm pretty sure that the parameter of this is in deciaml not hex (since $32 is not a multiple of 4)
	if (sscanf(line,"%*s %*s addiu sp,sp,-%u",&size)==1)
	{
		return size;
	}
	return 0;
}

// attempt to find the offset where the return address is stored
// return 0 if not found.
// This code assumes that the offset is never 0
unsigned int Function::GetRetOffset(char* line)
{
	unsigned int offset=0;
	char reg[3];
	if ((sscanf(line,"%*s %*s sw ra,%u(%2s)",&offset,reg)==2) 
			&& (strcmp(reg,"sp")==0))
	{
		return offset;
	}
	return 0;
}
