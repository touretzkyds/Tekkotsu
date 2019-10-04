About the StackedIt tool.
==========================

Author: Joel Fenwick
Mi-Pal,  Griffith University, Australia
29/7/2003
Latest updates 
(V2.0.1): 9/10/2003
(V2.0): 1/9/2003



Naming:
======
Version 2 is almost a complete rewrite of the code (from 1 file up to 13 files). At the suggestion of Nathan Lovell the name was changed from StackCheck to StackedIt. 
The main reason for this is that the new name is more amusing.

Note:
=====
In this document method 1 refers to the way V1 of this program derived its traces. 

Changes in V2.0.1
=============
Bug fix related to very long function signatures.


Changes in V2.0
============
A better method of getting traces has been added.
The output of method 1 has been modified. It now shows the location on the stack of the match and the name of the function.


What does it do?
============
	StackedIt will tell you which functions were called immediately before the crash. The accuracy of this trace depends on the method used to get the trace. The newer method is more accurate and "shouldn't" give any false positives unless it is told to. The older method is still available via command line arguments.


What doesn't it do?
==============
	StackedIt won't tell you what arguments were passed to those functions, or why the crash happened.
	
Detail:
=====
	Both methods make use of information from a dissassembly of the .nosnap.elf of the offending object. This dissassembly can be produced using the elp.pl tool or some other method.

New method:
	Starting with the function where the crash occurred the stack usage of each function is determined and the relative location of the return address is found. From this info the previous function in the trace
is determined and the process repeats. Once this "trace" is completed any values in the stack that could be return addresses can also be checked. These other checks will produce false positives in most circumstances so the output of these extra traces is clearly indicated and can be controlled with command line arguements.

Old method: 
	StackedIt will output any values in the stack dump section of EMON.LOG which fall in the range of addresses in the dissassembly of .nosnap.elf. Any values that fit this range are mapped to the name of the function that contains that address. Since in some cases the return address is stored on the stack when a call is made we can work out which function called this one. 
_Warning_: Not all values that appear on the stack are addresses so just because a value maps to a function doesn't mean that function is part of the "call trace".
  
Preparation:
=============
To use StackedIt you will need:
* A compiled version of the tool. 'make' in the top directory for these tools will do it.
* EMON.LOG for the crash.  
* A file containing the diassassembly (with interleved source) of X.nosnap where X is the object that crashed.
* The runtime address of the crash (epc in EMON.LOG).
* The corresponding linktime address of the crash (the Programmer's guide describes how to find this value or use elp).

Usage:
======
StackedIt [options] rt lt dis emon
Where:	[options] is a list of options starting with - or --help
	* 1	Use method 1 to generate trace.
	* 2	Use method 2 to generate trace.
	* p 	Attempt to follow other traces.
	* n	Only show names of functions not address data.

	rt is the runtime address of the crash (epc from EMON.LOG).
	lt is the link time address of the crash (calculated using the process described in the programmers guide).
	dis is the disassembly of the object which crashed (see programmer's guide).
	emon is your EMON.LOG file.
Note: The elp script will automate finding the above values.

Note: the addresses are expected to be in hex.
eg:
./StackedIt 2260a884 40a884 disassembly.txt EMON.LOG

would produce output like:

Method 1 - simple matching:
In the stack
StackLoc Name
22411354 GUObj::v32()
22411374 GUObj::v31()
22411394 GUObj::v30()
224113b4 GUObj::v29()
224113c0 void AlgUtil::QuickSortEx<BF_BLOB>(BF_BLOB*, int, int)
224113d4 GUObj::v28()
224113f4 GUObj::v27()
224113f8 void AlgUtil::QuickSortEx<BF_BLOB>(BF_BLOB*, int, int)
22411414 GUObj::v26()
22411434 GUObj::v25()
22411454 GUObj::v24()
22411474 GUObj::v23()
22411494 GUObj::v22()
224114a0 void AlgUtil::QuickSortEx<BF_BLOB>(BF_BLOB*, int, int)
224114b4 GUObj::v21()
224114d4 GUObj::v20()
224114f4 GUObj::v19()
22411514 GUObj::v18()
22411534 GUObj::v17()
22411554 GUObj::v16()
22411574 GUObj::v15()
22411594 GUObj::v14()
224115b4 GUObj::v13()
224115d4 GUObj::v12()
224115f4 GUObj::v11()
22411614 GUObj::v10()
22411634 GUObj::v9()
22411654 GUObj::v8()
22411674 GUObj::v6()
22411694 GUObj::v5()
224116b4 GUObj::v4()
224116d4 GUObj::v3()
224116f4 GUObj::v2()
2241170c ConsistencyEngine::ProcessInput()
22411714 GUObj::v1()
22411734 GUObj::NewImage(ONotifyEvent const&)
2241175c _Notify2
22411764 _connectstub2
224117bc _Ready0
224117d4 _notifystub2
224117dc _readystub0


Method 2 using return addresses:
Crash at 439cb8 = 22339cb8 in function GUObj::v7()
Frame    Name
22411338 GUObj::v7()
22411358 GUObj::v32()
22411378 GUObj::v31()
22411398 GUObj::v30()
224113b8 GUObj::v29()
224113d8 GUObj::v28()
224113f8 GUObj::v27()
22411418 GUObj::v26()
22411438 GUObj::v25()
22411458 GUObj::v24()
22411478 GUObj::v23()
22411498 GUObj::v22()
224114b8 GUObj::v21()
224114d8 GUObj::v20()
224114f8 GUObj::v19()
22411518 GUObj::v18()
22411538 GUObj::v17()
22411558 GUObj::v16()
22411578 GUObj::v15()
22411598 GUObj::v14()
224115b8 GUObj::v13()
224115d8 GUObj::v12()
224115f8 GUObj::v11()
22411618 GUObj::v10()
22411638 GUObj::v9()
22411658 GUObj::v8()
22411678 GUObj::v6()
22411698 GUObj::v5()
224116b8 GUObj::v4()
224116d8 GUObj::v3()
224116f8 GUObj::v2()
22411718 GUObj::v1()
22411738 GUObj::NewImage(ONotifyEvent const&)
22411760 _Notify2
224117d8 _notifystub2
Dead end. This function doesn't store a return address

Other traces:
Frame    Name
224113c4 void AlgUtil::QuickSortEx<BF_BLOB>(BF_BLOB*, int, int)
224113f8 GUObj::v27()
Not a valid return address
------
Frame    Name
224113fc void AlgUtil::QuickSortEx<BF_BLOB>(BF_BLOB*, int, int)
Not a valid return address
------
Frame    Name
224114a4 void AlgUtil::QuickSortEx<BF_BLOB>(BF_BLOB*, int, int)
224114d8 GUObj::v20()
Not a valid return address
------
Frame    Name
22411710 ConsistencyEngine::ProcessInput()
22411738 GUObj::NewImage(ONotifyEvent const&)
22411760 _Notify2
224117d8 _notifystub2
Dead end. This function doesn't store a return address
------
Frame    Name
22411768 _connectstub2
Dead end. This function doesn't store a return address
------
Frame    Name
224117c0 _Ready0
224117e0 _readystub0
Dead end. This function doesn't store a return address
------
