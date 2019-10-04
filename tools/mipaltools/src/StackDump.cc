// -----------------------------------------------------------------------------
// File: StackDump.cc
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
// Updated 9/10/2003 - V2.0.1 fixed a problem with includes
// -----------------------------------------------------------------------------

#include "StackDump.h"
#include <vector>
#include <cstdlib>
#include <cstdio>
using namespace std;

// Not particularly happy with mixing C and C++ IO here but sscanf is too useful

// The plan for dealing with the missing (or dead lines) is to assign each value in the stack a "valid" attribute
// We can then  check to see if we actually know what was at a particular location or not. 

/* Format of stack dump:
[stack info]
 area: 0x220a0c08 - 0x220a1808, sp: 0x220a1430
[stack dump]
220a13f0: 22642658 800b4e08 23480008 800b4bac  X&d".N....H#.K..
220a1400: 00000001 00000000 00000000 00000001  ................
220a1410: 226b2dd0 226b2dd0 2791a626 bea5ee4b  .-k".-k"&..'K...
*/

// assume the stream is at the start of the area line
StackDump::StackDump(istream& is) throw (StackAnalyserException)
{
	char buffer[80];
	top=0;
	ulimit=0;
	llimit=0;
	// Now to read the stack contents
	is.getline(buffer,80);
	top=static_cast<unsigned int>(strtoul(buffer+38,0,16));
	if (top==0)
	{
		throw StackAnalyserException("Unable to determine top of stack");
	}
	is.getline(buffer,80);	// skip the [stack dump] line
	vector<char*> lines;
	ReadStack(is,lines);
	if (lines.size()==0)
	{
		throw StackAnalyserException("No valid lines in stack dump.");
	}
	else
	{
		ProcessStack(lines);
	}
}

// get a list of lines in the stack dump
void StackDump::ReadStack(istream& is, vector<char*>& lines)
{
	const int LINELENGTH=80;
	char* buffer=new char[LINELENGTH];
	while (is.getline(buffer,LINELENGTH),is.good() && buffer[0]!='\0')
	{
		if (buffer[0]!='*')
		{
			lines.push_back(buffer);
			buffer=new char[LINELENGTH];
		}
		
	}
	delete []buffer;
}

// extract data from previously read lines
void StackDump::ProcessStack(vector<char*>& lines)
{
	unsigned int v[5];

	// step 1 find the range of addresses we are dealing with
	
	llimit=(unsigned int)strtoul(lines[0],0,16);
	if (llimit==0)
	{	
		throw StackAnalyserException("Unable to determine the lower limit of the stack.");
		return;
	}
	ulimit=(unsigned int)strtoul(lines[lines.size()-1],0,16);
	if (ulimit==0)
	{
		
		throw StackAnalyserException("Unable to determine the upper limit of the stack.");
	}
	ulimit+=15;					// last address is on the end of the line
	if (ulimit<llimit)
	{
		throw StackAnalyserException("Upper limit of the stack is smaller than the lower limit.");
	}
	// Now fill in the lines
	contents=new StackEntry[ulimit-llimit];
	for (vector<char*>::iterator i=lines.begin();i!=lines.end();++i)
	{
		if (sscanf(*i,"%x: %x %x %x %x",&(v[0]),&(v[1]),&(v[2]),&(v[3]),&(v[4]))!=5)
		{
			throw StackAnalyserException("Badly formed line in stack dump.");
		}
		else
		{
			for (int j=0;j<4;j++)
			{
				contents[(v[0]-llimit)/4+j].value=v[j+1];
				contents[(v[0]-llimit)/4+j].valid=true;
			}
		}
	}
}

// Read the values from a line in the stack.
// cope with some problem lines
bool StackDump::ReadStackLine(istream& is, unsigned int vals[])
{
	char c=' ';
	is >> c;
	if (!is.good() || c=='*')
	{
		return false;
	}
	is.putback(c);
	is >> hex;
	is >> vals[4] >> c >> vals[0] >> vals[1] >> vals[2] >> vals[3];
	// clear chars at end of the line
	for (int i=0;i<18;i++)
	{
		c=is.get();
	}
	return true;
} 

// access data on the stack
const unsigned int& StackDump::operator[](unsigned int addr) throw (StackAnalyserException)
{
	if ((addr>=llimit) && (addr<=ulimit))
	{
		return contents[(addr-llimit)/4].value;
	}
	else
	{	
		throw StackAnalyserException("Stack access out of bounds");
	}
}

// Do we have a value for this location on the stack
bool StackDump::IsValid(unsigned int addr)
{
	if ((addr>=llimit) && (addr<=ulimit))
	{
		return contents[(addr-llimit)/4].valid;
	}
	else
	{	
		return false;
	}
}

void StackDump::DisplayStack(ostream& os)
{
	os << hex;
	for (unsigned int i=llimit;i<=ulimit;i+=4)
	{
		os << i << " ";
		if (IsValid(i))
		{
			os << (*this)[i];
		}
		else
		{
			os << "*";
		}
		os << endl;
	}
}



