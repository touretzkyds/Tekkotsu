// -----------------------------------------------------------------------------
// File: FunctionStore.h
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

#ifndef FUNCTION_STORE_H
#define FUNCTION_STORE_H

#include "Function.h"
#include <vector>
#include <iostream>
class FunctionStore
{
public:
	FunctionStore();
	FunctionStore(std::istream& is);	// functions must be in increasing address order
	~FunctionStore();
	Function* ContainsAddress(unsigned int addr);
	void GetAddressRange(unsigned int& lower, unsigned int& upper);
	inline int FunctionCount(){return fns.size();}
	void Dump(std::ostream& os);
	
private:
	std::vector<Function*> fns;
	Function* fp;				// current function

	int BinarySearch(int begin, int end, unsigned int val);
	void Process(char* line);		// Process this line of dissassembly
};

#endif	// FUNCTION_STORE_H
