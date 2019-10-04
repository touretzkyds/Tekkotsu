// -----------------------------------------------------------------------------
// File: Trace.h
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

#ifndef TRACE_H
#define TRACE_H

#include <list>
#include "Function.h"

#include <iostream>
// class to link an address on the stack with the function it corresponds to
class FunctionOnStack
{
public:
	FunctionOnStack(unsigned int addr, Function* fn)
		:address(addr),f(fn)
	{};
	inline unsigned int GetAddress(){return address;};
	inline Function* GetFunction(){return f;};
	int operator==(const FunctionOnStack& other){return address==other.address;}
private:	
	unsigned int address;	// 4 bytes before the start of the stack frame
	Function* f;			// note this class does not own this pointer
};

// Represents a possible stack trace
// Do not use non constant strings in this object since there is no cleanup done
class Trace
{
public:
	Trace(){reason="";}
	std::list<FunctionOnStack> fns;
	const char* reason;			// why did we stop following this trace
};

#endif
