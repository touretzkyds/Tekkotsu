// -----------------------------------------------------------------------------
// File: EmonLog.cc
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

#include "EmonLog.h"
#include "StackAnalyserException.h"
#include <cstdlib>
#include <cstring>

using namespace std;

EmonLog::EmonLog(istream& is)   throw (StackAnalyserException)
{
	stack=0;
	ra=0;
	ReadRegisters(is);
	if (!FindStack(is))
	{
		throw StackAnalyserException("Unable to find [stack info].");
	}
	stack=new StackDump(is);
}

EmonLog::~EmonLog()
{
	delete stack;
}

// get values for any registers we'll need later
void EmonLog::ReadRegisters(istream& is)
{
	char buffer[200];
	while (is.getline(buffer,200),cin.good())
	{
		if (buffer[0]=='\0')
		{
			// unable to find the value of ra
			break;
		}
		if (strncmp(buffer," ra:r31: 0x",11)==0)
		{
			ra=static_cast<unsigned int>(strtoul(buffer+11,0,16));
			break;
		}
	}
}

// Scan through an EMON file looking for the start of the stack
bool EmonLog::FindStack(istream& is)
{
	char buffer[200];
	while (is.getline(buffer,200),is.good())
	{
		if (strncmp(buffer,"[stack info]",11)==0)
		{
			return true;
		}
	}
	return false;
}
