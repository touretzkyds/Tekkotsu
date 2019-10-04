//-*-c++-*-
/*
 
 **** NOTICE ****
 THIS FILE IS NO LONGER NEEDED
 Instead, use the REGISTER_BEHAVIOR( FooBehavior ) macro in your behavior's .cc file
 Your behavior may no longer need a header file if nothing else is referring to it, just
 move all of your code into the .cc file instead (and put the registration at the end).
 
 This allows your behavior to be completely self-contained: faster compilation, easier maintenance
 
*/

#ifndef _UserBehaviors_Includes_
#define _UserBehaviors_Includes_
//----------------------------------------------------------------
//
// Part 1: add #include statements for your behaviors' .h files here:

//#include "SampleBehavior.h"

//
//----------------------------------------------------------------

#define MENUITEM(_behavior) REGISTER_BEHAVIOR(_behavior);

//----------------------------------------------------------------
//
// Part 2: add MENUITEM entries for your behaviors here:

//MENUITEM(SampleBehavior)

//
//----------------------------------------------------------------
#endif
