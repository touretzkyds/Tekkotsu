// -----------------------------------------------------------------------------
// File: StackDump.h
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


// For storing the details of a stack dump
// The idea is to have a mapping from runtime address to stack contents
// Note the Upper and Lower limit are numerical bounds (ie: Lower < Top < Upper)

#ifndef STACKDUMP_H
#define STACKDUMP_H
#include <iostream>
#include <vector>
#include "StackAnalyserException.h"

class StackDump
{
public:
	StackDump(std::istream& is) throw (StackAnalyserException);
	inline unsigned int GetTop() {return top;};		// address of the top of the stack
	inline unsigned int GetULimit() {return ulimit;};
	inline unsigned int GetLLimit() {return llimit;};
	const unsigned int& operator[](unsigned int addr) throw (StackAnalyserException);
	bool IsValid(unsigned int);					// do we have a value for that stacklocation
	void DisplayStack(std::ostream& os);
private:
	struct StackEntry
	{
		StackEntry(){value=0;valid=false;}
		unsigned int value;
		bool valid;
	};
	StackEntry* contents;
	unsigned int top;
	unsigned int ulimit;
	unsigned int llimit;
	
	bool ReadStackLine(std::istream& is, unsigned int vals[]);	// read a single line
	void ReadStack(std::istream& is, std::vector<char*>& lines);	
				// get lines of text that make up the stack dump
	void ProcessStack(std::vector<char*>& lines);	// extract data from lines	
};

#endif // STACKDUMP_H
