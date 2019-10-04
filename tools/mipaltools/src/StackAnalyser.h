// ---------------------------------------------------------------------------------------------------
// File: StackAnalyser.h
// Date: 28/8/2003
// Part of StackedIt V2.0
// By: Joel Fenwick, Mi-Pal, Griffith University
//
// This program is copyrighted to the author and released under the GPL (V2.0).
// There is no warantee associated with this code. Especially not for use
// in nuclear plants, aircraft control systems or hedgehogs. Use at your
// own risk.
// See license.txt for details
// ----------------------------------------------------------------------------------------------------

#ifndef STACKANALYSER_H
#define STACKANALYSER_H

#include "StackDump.h"
#include "FunctionStore.h"
#include "StackAnalyserException.h"
#include <iostream>
#include <list>
#include "Trace.h"
#include "EmonLog.h"


class StackAnalyser
{
public:
	StackAnalyser(std::istream& emondata, std::istream& functiondata,unsigned int runtimeoffset)
			throw (StackAnalyserException);
	~StackAnalyser();
	void SimpleCheck(std::ostream& os,bool shownumbers);	// tests used in version1
	void Filtering(std::ostream& os, unsigned int crashaddress, 
			bool attemptrestart, bool shownumbers);		// test added in v2
	
private:
	EmonLog* emon;
	StackDump* stack;
	FunctionStore* fns;
	unsigned int offset;	// runtimeaddress - linktime address
	bool FindStack(std::istream& is);	// find the top of the stack dump in EMON
	void FindMatches(std::list<FunctionOnStack>& locations, std::list<FunctionOnStack>& abovestack);
							// find values in stack that match functions
	void TraceBack(Trace& tr, unsigned int stacklocation, Function* f);	// used in method2
	void SimpleCheckDriver(std::list<FunctionOnStack>& fos, std::ostream& os, 
		bool shownumbers);		// display matches from method1
	void ShowFilterTrace(std::ostream& os, Trace& tr, bool shownumbers);	
							// show traces from method2
};

#endif   // STACKANALYSER_H

