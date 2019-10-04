#ifndef INCLUDED_ZIGNOR_H
#define INCLUDED_ZIGNOR_H

/*! @file
*==========================================================================
*  This code is Copyright (C) 2005, Jurgen A. Doornik.
*  Permission to use this code for non-commercial purposes
*  is hereby given, provided proper reference is made to:
*		Doornik, J.A. (2005), "An Improved Ziggurat Method to Generate Normal
*          Random Samples", mimeo, Nuffield College, University of Oxford,
*			and www.doornik.com/research.
*		or the published version when available.
*	This reference is still required when using modified versions of the code.
*  This notice should be maintained in modified versions of the code.
*	No warranty is given regarding the correctness of this code.
*==========================================================================
*
* @author Jurgen A. Doornik (Creator)
*/

#ifdef __cplusplus
extern "C" {
#endif

void    RanNormalSetSeedZig(int *piSeed, int cSeed);
double  DRanNormalZig(void);
void    RanNormalSetSeedZigVec(int *piSeed, int cSeed);
double  DRanNormalZigVec(void);
void    RanNormalSetSeedZig32(int *piSeed, int cSeed);
double  DRanNormalZig32(void);
void    RanNormalSetSeedZig32Vec(int *piSeed, int cSeed);
double  DRanNormalZig32Vec(void);

double  DRanQuanNormalZig(void);
double  DRanQuanNormalZigVec(void);
double  DRanQuanNormalZig32(void);
double  DRanQuanNormalZig32Vec(void);

#ifdef __cplusplus
} //extern "C"
#endif

#endif
