// -----------------------------------------------------------------------------
// File: Function.h
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

#ifndef FUNCTION_H
#define FUNCTION_H

// class to encapsulate all data about a single function from a dissassembly
class Function
{
public:
	Function(char* line);		// line contains the function header
	~Function();
	bool Process(char* c);		// process another line of dissassembly
	char* name;
	unsigned int stacksize;
	unsigned int retoffset;			// location in the function where return addr is stored
	unsigned int start;		// the start address of this function
	static bool IsHeader(char* line);	// Does this line represent a function header
	bool HaveRetOffset(){return haveretoffset;}	// maybe should have a better
private:
	int linecount;
	bool haveretoffset;	// do we have a return address offset in the "frame" yet

	// return size of stack "frame" allocated by this line
	// 0 if not an allocation line
	unsigned int GetStackSize(char* line);
	
	// read the offset in the frame where the return address is stored
	// 0 if not a relevant line
	// This assumes that the offset will always be >0 
	// One day I might fix this but it seems ok for now
	unsigned int GetRetOffset(char* line);	
};


#endif //FUNCTION_H
