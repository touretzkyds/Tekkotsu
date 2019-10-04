// -----------------------------------------------------------------------------
// File: main.cc
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

#include "StackDump.h"
#include "FunctionStore.h"
#include "StackAnalyser.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

// options from cmd line parameters
const unsigned int PAR_V1=0x00000001;
const unsigned int PAR_V2=0x00000002;
const unsigned int PAR_PERSIST=0x00000004;
const unsigned int PAR_SHOWNUM=0x000008;
const unsigned int PAR_DEF=PAR_V1|PAR_V2|PAR_PERSIST|PAR_SHOWNUM;

// programe is the name used to invoke the program
void ShowShortHelp(char* progname)
{
	cout << "Usage: " << progname << " [--help | -options] runtime linktime dissassembly EMON\n";
}

// programe is the name used to invoke the program
void ShowDetailedHelp(char* progname)
{
	cout << "StackAnalyser V2.0\n";
	cout << "This program is copyrighted to the author (Joel Fenwick) and licensed under the GPL (v2.0). ";
	cout << "There is no warantee (at all) attached to this program.\n";	
	cout << "\n";
	ShowShortHelp(progname);
	cout << "options: (the options list starts with - and contains no spaces.)\n";
	cout << "	--help	This message.\n";
	cout << "	1	Displays a trace using the method from version 1 of this \n\t\tprogram. \n\t\t(any matching numbers are mapped and displayed).\n";
	cout << "	2	Displays a trace using the return address in the stack frames \n\t\tto determine the calling function - leading to much\n\t\tfewer false positives.\n";
	cout << "	p	Persist. If using method 2 try to find other possible trace\n\t\tfragments.\n";
	cout << "	n	Only show the names of functions.\n";	
	cout << "\n";
	cout << "If no options are specified the default is to return as much information as possible. Currently this means -12p.\n";
	cout << "\nParameters:\n";
	cout << "disassembly - text file containing the dissassembly of the code that crashed.\n";
	cout << "EMON - the exception monitor log file. This must contain a register dump and a stack dump (as a minimum).\n";
	cout << "runtime - the runtime address of the crash.\n";
	cout << "linktime - the corresponding link time address of the crash.\n";
	cout << "\nNotes:\n";
	cout << " * The persist option will produce false positives in the \"other traces\" \nsection. In most cases the first trace produced by method two should be enough\nfor debugging purposes.\n";
}

// Does the char target appear in string pars
bool HasParam(char* pars,char target)
{
	while (*pars!='\0')
	{
		if (*pars==target)
		{
			return true;
		}
		pars++;
	}
	return false;
}

// Sets up the params for the analyser object.
// If non-zero is returned the program should be halted
int ValidateArgs(int argc, char** argv, unsigned int& runtime, unsigned  int& linktime, 
	std::ifstream& assemblyf, std::ifstream& emonf, int& options)
{
	int paramoffset=0;
	options=0;
	if ((argc>1) && (strcmp(argv[1],"--help")==0))
	{
		ShowDetailedHelp(argv[0]);
		return 7;
	}
	if (argc<5)
	{
		ShowShortHelp(argv[0]);
		return 1;
	}
	if (argv[1][0]=='-')	// were options specified
	{
		bool optionspresent=false;
		paramoffset=1;
		if (HasParam(argv[1],'1'))
		{
			options=options|PAR_V1;	
			optionspresent=true;
		}
		if (HasParam(argv[1],'2'))
		{
			options=options|PAR_V2;
			optionspresent=true;
		}
		if (HasParam(argv[1],'p'))
		{
			options=options|PAR_PERSIST;
			optionspresent=true;
		}
		if (!HasParam(argv[1],'n'))
		{
			options=options|PAR_SHOWNUM;
		}
		else
		{
			optionspresent=true;
		}
		if (!optionspresent)
		{
			options=PAR_DEF;
		}
	}
	else
	{
		options=PAR_DEF;
	}	
	runtime=(unsigned int)(strtoul(argv[1+paramoffset],0,16));
	if (runtime==0)
	{
		cout << argv[1+paramoffset] << " is not a valid runtime address\n";
		return 2;
	}
	linktime=(unsigned int)(strtoul(argv[2+paramoffset],0,16));
	if (linktime==0)
	{
		cout << argv[2+paramoffset] << " is not a valid linktime address\n";
		return 3;
	}
	if (runtime <linktime)
	{
		// This limitation simplifies things so 
		cout << "Runtime address must be greater than linktime address\n";
		return 4;
	}
	emonf.open(argv[4+paramoffset]);
	if (emonf.fail())
	{
		cout << "Couldn't open EMON.LOG file " << argv[4+paramoffset] << endl;
		return 5;
	}
	assemblyf.open(argv[3+paramoffset]);
	if (assemblyf.fail())
	{
		cout << "Couldn't open disassembly file " << argv[3+paramoffset] << endl;
		emonf.close();
		return 6;
	}
	return 0;
}

int main(int argc, char** argv)
{
	int options=0;
	unsigned int runtime=0;
	unsigned int linktime=0;
	unsigned int offset=0;
	ifstream assemblyf, emonf;
	int ret=ValidateArgs(argc,argv,runtime, linktime, assemblyf, emonf,options);
	if (ret!=0)
	{
		exit(ret);
	}
	offset=runtime-linktime;
	try
	{
		StackAnalyser sa(emonf, assemblyf, offset);
		if (options&PAR_V1)
		{
			cout << "Method 1 - simple matching:\n";
			sa.SimpleCheck(cout,options&PAR_SHOWNUM);
			cout << "\n";
		}
		if (options&PAR_V2)
		{
			cout << "Method 2 using return addresses:\n";
			sa.Filtering(cout, linktime,(options&PAR_PERSIST)!=0,options&PAR_SHOWNUM);
			cout << "\n";
		}
	}
	catch (StackAnalyserException& s)
	{
		cout << "Error: " << s.GetMsg() << endl;
	}
	assemblyf.close();
	emonf.close();	
	return 0;
}
