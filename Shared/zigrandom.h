#ifndef INCLUDED_ZIGRANDOM_H
#define INCLUDED_ZIGRANDOM_H

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
	
#ifdef __LP64__
	typedef unsigned long UINT64;
	typedef          long INT64;
	#define LIT_UINT64(c) (c##ul)
	#define LIT_INT64(c)  (c##l)
#elif defined(_MSC_VER)
	typedef unsigned __int64 UINT64;
	typedef          __int64 INT64;
	#define LIT_UINT64(c) (c##ui64)
	#define LIT_INT64(c)  (c##i64)
#else 
	typedef unsigned long long UINT64;
	typedef          long long INT64;
	#define LIT_UINT64(c) (c##ull)
	#define LIT_INT64(c)  (c##ll)
#endif

#define M_RAN_INVM30	9.31322574615478515625e-010			  /* 1.0 / 2^30 */
#define M_RAN_INVM32	2.32830643653869628906e-010			  /* 1.0 / 2^32 */
#define M_RAN_INVM48	3.55271367880050092936e-015			  /* 1.0 / 2^48 */
#define M_RAN_INVM52	2.22044604925031308085e-016			  /* 1.0 / 2^52 */
#define M_RAN_INVM64	5.42101086242752217004e-020			  /* 1.0 / 2^64 */

#define RANDBL_32old(iRan1)			      \
    ((unsigned int)(iRan1) * M_RAN_INVM32)
#define RANDBL_48old(iRan1, iRan2)		  \
    ((unsigned int)(iRan1) + (unsigned int)((iRan2) << 16) \
		* M_RAN_INVM32) * M_RAN_INVM32
#define RANDBL_52old(iRan1, iRan2)		  \
    ((unsigned int)(iRan1) + (unsigned int)((iRan2) << 12) \
		* M_RAN_INVM32) * M_RAN_INVM32

#define RANDBL_32new(iRan1)                   \
    ((int)(iRan1) * M_RAN_INVM32 + (0.5 + M_RAN_INVM32 / 2))
#define RANDBL_48new(iRan1, iRan2)            \
    ((int)(iRan1) * M_RAN_INVM32 + (0.5 + M_RAN_INVM48 / 2) + \
        (int)((iRan2) & 0x0000FFFF) * M_RAN_INVM48)
#define RANDBL_52new(iRan1, iRan2)            \
    ((int)(iRan1) * M_RAN_INVM32 + (0.5 + M_RAN_INVM52 / 2) + \
        (int)((iRan2) & 0x000FFFFF) * M_RAN_INVM52)

void 	GetInitialSeeds(unsigned int auiSeed[], int cSeed,
	unsigned int uiSeed, unsigned int uiMin);

/* MWC8222 George Marsaglia */
void RanSetSeed_MWC8222(int *piSeed, int cSeed);
unsigned int IRan_MWC8222(void);
double DRan_MWC8222(void);
void VecIRan_MWC8222(unsigned int *auiRan, int cRan);
void VecDRan_MWC8222(double *adRan, int cRan);

/* plug-in RNG */
typedef double 		( * DRANFUN)(void);
typedef unsigned int( * IRANFUN)(void);
typedef void   		( * IVECRANFUN)(unsigned int *, int);
typedef void   		( * DVECRANFUN)(double *, int);
typedef void   		( * RANSETSEEDFUN)(int *, int);

void    RanSetRan(const char *sRan);
void    RanSetRanExt(DRANFUN DRanFun, IRANFUN IRanFun, IVECRANFUN IVecRanFun,
	DVECRANFUN DVecRanFun, RANSETSEEDFUN RanSetSeedFun);
double  DRanU(void);
unsigned int IRanU(void);
void    RanVecIntU(unsigned int *auiRan, int cRan);
void    RanVecU(double *adRan, int cRan);
void    RanSetSeed(int *piSeed, int cSeed);

/* normal probabilities */
double  DProbNormal(double x);

/* polar standard normal RNG */
double  FRanNormalPolar(void);
double  DRanNormalPolar(void);
double  FRanQuanNormal(void);
double  DRanQuanNormal(void);

#ifdef __cplusplus
} //extern "C"
#endif

#endif /* INCLUDED_ZIGRANDOM_H */
