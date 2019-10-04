// -----------------------------------------------------------------------------
// File: EmonLog.h
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

#ifndef EMONLOG_H
#define EMONLOG_H
#include <iostream>
#include "StackDump.h" 

// class to encapsulate all information gathered from an EMON.LOG file
class EmonLog
{
public:
	EmonLog(std::istream& is) throw (StackAnalyserException);
	~EmonLog();
	StackDump* GetStackDump(){return stack;}
	unsigned int GetRegister_ra(){return ra;}		
private:
	StackDump* stack;
	void ReadRegisters(std::istream& is);	
	bool FindStack(std::istream& is);	// move stream pointer to start 
									//of stack dump
	unsigned int ra;	// value of the ra register
};


#endif	// EMONLOG_H
