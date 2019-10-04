// -----------------------------------------------------------------------------
// File: StackAnalyserException.h
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

#ifndef STACKANALYSEREXCEPTION_H
#define STACKANALYSEREXCEPTION_H

// Warning do not pass custom built strings into this class.
// Since as you can see it doesn't clean them up properly
class StackAnalyserException
{
public:
	StackAnalyserException(const char* m){msg=m;}
	const char* GetMsg(){return msg;}
private:
	const char* msg;
};

#endif // STACKANALYSEREXCEPTION_H
