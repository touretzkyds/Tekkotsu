// -----------------------------------------------------------------------------
// File: StackAnalyser.cc
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

#include "StackAnalyser.h"
#include <cstring>
#include <iostream>
using namespace std;

StackAnalyser::StackAnalyser(istream& emondata, istream& functiondata, 
		unsigned int runtimeoffset) 
	throw (StackAnalyserException)
{
	emon=new EmonLog(emondata);	// throw StackAnalyserException
	stack=emon->GetStackDump();
	fns=new FunctionStore(functiondata);
	if (fns->FunctionCount()==0)
	{
		throw StackAnalyserException("No function headers found in data.");
	}
	offset=runtimeoffset;
}

StackAnalyser::~StackAnalyser()
{
	if (stack!=0)
	{
		delete stack;
	}
	if (fns!=0)
	{
		delete fns;
	}
}

// Scan through an EMON file looking for the start of the stack
bool StackAnalyser::FindStack(istream& is)
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

// return enough 0s to padd a hex number to 8 digits
const char* Padding(unsigned int i)
{
	if (i<0x10)
	{
		return "0000000";
	}
	if (i<0x100)
	{
		return "000000";
	}
	if (i<0x1000)
	{
		return "00000";
	}	
	if (i<0x10000)
	{
		return "0000";
	}
	if (i<0x100000)
	{
		return "000";
	}
	if (i<0x1000000)
	{
		return "00";
	}
	if (i<0x10000000)
	{
		return "0";
	}
	return "";
}

// populates list with addresses in the stack which store potential functions
// "locations" stores locations in the stack proper while above stack stores locations
// between llimit and top of stack
void StackAnalyser::FindMatches(list<FunctionOnStack>& locations, list<FunctionOnStack>& abovestack)
{
	unsigned int lower=0;
	unsigned int upper=0;
	fns->GetAddressRange(lower,upper);
	lower+=offset;
	upper+=offset;
	
	// check for useful data past the top of the stack
	for (unsigned int i=stack->GetLLimit();i<stack->GetTop();i+=4)
	{
		unsigned int data=(*stack)[i];
		if ((data>=lower) && (data<upper))
		{
			Function* fp=fns->ContainsAddress(data-offset);
			if (fp!=0)
			{
				FunctionOnStack fos(i,fp);
				abovestack.push_back(fos);
			}
		}
	}
	for (unsigned int i=stack->GetTop();i<=stack->GetULimit();i+=4)
	{
		unsigned int data=(*stack)[i];
		if ((data>=lower) && (data<upper))
		{
			Function* fp=fns->ContainsAddress(data-offset);
			if (fp!=0)
			{
				FunctionOnStack fos(i,fp);
				locations.push_back(fos);
			}
		}	
	}
}

// Returns the names of any addresses in the stack that match functions
// if shownumbers is false only the names of the functions are displayed
void StackAnalyser::SimpleCheck(ostream& os, bool shownumbers)
{
	list<FunctionOnStack> instack, abovestack;
	FindMatches(instack,abovestack);
	if (abovestack.size() >0)
	{
		os << "Above the stack\n";
		SimpleCheckDriver(abovestack, os,shownumbers);
		os << endl;
	}
	if (instack.size()>0)
	{
		os << "In the stack\n";
		SimpleCheckDriver(instack, os,shownumbers);
		os << endl;
	}
}

// Outputs results for SimpleCheck
void StackAnalyser::SimpleCheckDriver(list<FunctionOnStack>& fos, ostream& os, bool shownumbers)
{
	if (shownumbers)
	{
		os << "StackLoc Name\n";
	}
	else
	{
		os << "Name\n";
	}
	for (list<FunctionOnStack>::iterator i=fos.begin();i!=fos.end();++i)
	{
		if (shownumbers)
		{
			os << hex << Padding((*i).GetAddress()) << (*i).GetAddress() << ' ';	// address on stack
		}
		os << (*i).GetFunction()->name << endl;
	}
}

// start at this point in the stack for this function and trace back
void StackAnalyser::TraceBack(Trace& tr,  unsigned int stacklocation, Function* f)
{
	unsigned int retlocation=stacklocation+f->retoffset;	// where is the return address stored
	unsigned int lower,upper;
	
	fns->GetAddressRange(lower, upper);
	if (!f->HaveRetOffset())
	{
		tr.reason="Dead end. This function doesn't store a return address";
		return;
	}
	if (f->stacksize==0)
	{
		tr.reason="Dead end. This function has no stack frame";	
		return;
	}

	// sanity check the address
	if (retlocation>stack->GetULimit())
	{
		tr.reason="Frame extends beyond visible region";
		return;
	}
	if (!stack->IsValid(retlocation))
	{
		tr.reason="Return address lies in a forbidden zone";
		return;
	}
	unsigned int retaddress=(*stack)[retlocation];
	if ((retaddress<offset) || (retaddress-offset<lower) || (retaddress-offset>upper))
	{
		tr.reason="Not a valid return address";
		return;
	}
	Function* f2=fns->ContainsAddress(retaddress-offset);
	if (f2==0)
	{
		tr.reason="Unable to match function to return address";
		return;
	}
	// Add function f2 to the trace
	FunctionOnStack fos(retlocation,f2);	
	tr.fns.push_back(fos);	
	TraceBack(tr, stacklocation+f->stacksize,f2);
}

// attempt restart determines whether to try any other traces after the main one ends
void StackAnalyser::Filtering(ostream& os, unsigned int crashaddress, bool attemptrestart, bool shownumbers)
{
	Trace tr;
	Function* f=fns->ContainsAddress(crashaddress);
	if (f==0)
	{
		os << "Error: Can't find a function containing address " << hex << crashaddress << endl;
		return;
	}
	os << "Crash at " << crashaddress << " = " << crashaddress+offset << " in function ";
	os << f->name << endl;
	list<FunctionOnStack> instack, abovestack;
	FindMatches(instack,abovestack);
	tr.fns.push_back(FunctionOnStack(/*0*/stack->GetTop()-4,f));	// to make sure the frame starts at the top of the stack	
	if ((f->stacksize!=0) && (f->HaveRetOffset()))	// the crash function saved ra to then stack
	{
		TraceBack(tr,stack->GetTop(),f);
	}
	else
	{	// try to get the return address from the ra register
		if ((emon->GetRegister_ra()!=0) 	
			&& (emon->GetRegister_ra()>=offset))
		{
			Function* raf=fns->ContainsAddress(emon->GetRegister_ra()-offset);
			if (raf!=0)
			{
				tr.fns.push_back(FunctionOnStack(/*0*/stack->GetTop()-4,raf));		
				TraceBack(tr,stack->GetTop()+f->stacksize,raf);
			}
			else
			{
				// ra doesn't contain a valid address
			}
		}
		else
		{
			os << "Error: attempted to use ra register but it contained the invalid value " <<hex << emon->GetRegister_ra() <<  endl;
		}
	}
	if (!tr.fns.empty())
	{	// display the functions in the trace
		ShowFilterTrace(os,tr,shownumbers);
		for (list<FunctionOnStack>::iterator i=tr.fns.begin();i!=tr.fns.end();++i)
		{
			instack.remove((*i));
		}
	}
	os << endl;
	if (attemptrestart)
	{
		// looks at any matches on the stack that weren't picked as part of the initial trace.
		// This could be because they are wrong or because the trace was interrupted.
		// In this second case using this option may allow you to pick up the trace again
		// at the price of some false positives
		// This method isn't overly smart. It assumes that the return address is stored in the
		// last address of the stack frame. Which isn't a good assumption at all.
		
		list<Trace*> tr2;
		while (!instack.empty())
		{
			list<FunctionOnStack>::iterator i=instack.begin();
			Trace* trp=new Trace();
			trp->fns.push_back(*i);
			TraceBack(*trp,(*i).GetAddress()+4,(*i).GetFunction());
			tr2.push_back(trp);
			// Remove items that appear in the trace
			for (list<FunctionOnStack>::iterator i=trp->fns.begin();i!=trp->fns.end();++i)
			{
				instack.remove((*i));
			}
		}
		os << "Other traces:\n";
		if (tr2.empty())
		{
			os << "None\n";
		}
		else
		{
			for (list<Trace*>::iterator j=tr2.begin();j!=tr2.end();++j)
			{
				ShowFilterTrace(os, *(*j), shownumbers);
				os << "------" << endl;	
			}
		}  // !tr2.isEmpty()
	}	// attemptrestart
}

// Displays a trace generated by filter
// shownumbers determines whether data is displayed for each function or just its name
void StackAnalyser::ShowFilterTrace(ostream& os, Trace& tr, bool shownumbers)
{
	if (shownumbers)
	{
		os << "Frame    Name\n";
	}
	else
	{
		os << "Name\n";
	}
	for (list<FunctionOnStack>::iterator i=tr.fns.begin();i!=tr.fns.end();++i)
	{
		if (shownumbers)
		{
			unsigned int addr=(*i).GetAddress();
			if (addr!=0)
			{
				// +4 to show the actual start of the stack frame
				os << hex <<  Padding(addr+4) << addr+4 << ' ';
			}
			else
			{
				os << "n/a      ";
			}
		}
		os <<  (*i).GetFunction()->name << endl;
	}
	os << tr.reason << endl;
}

