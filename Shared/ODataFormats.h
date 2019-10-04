//-*-c++-*-
#ifdef PLATFORM_APERIOS
#  include <OPENR/ODataFormats.h>
#else
#  ifndef INCLUDED_ODataFormats_h_
#  define INCLUDED_ODataFormats_h_

#include <stdlib.h>

//! provides compatability with the OPEN-R type of the same name
struct ODataVectorInfo {
	size_t numData;
};

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int longword;
typedef word OSoundFormat;
const OSoundFormat osoundformatUNDEF     = 0;
const OSoundFormat osoundformatPCM       = 1;
const OSoundFormat osoundformatUSB_MIDI  = 2;
const OSoundFormat osoundformatSMF       = 3; // STANDARD MIDI FILE


typedef word OSoundChannel;
const OSoundChannel osoundchannelUNDEF  = 0;
const OSoundChannel osoundchannelMONO   = 1;
const OSoundChannel osoundchannelSTEREO = 2;

//! provides compatability with the OPEN-R type of the same name
struct OSoundInfo {		                 // 64 bytes (total)
	typedef int longword;
	typedef short word;

    longword             frameNumber;    //  4 bytes
    size_t               frameSize;      //  4 bytes
    size_t               dataOffset;	 //  4 bytes
    size_t               maxDataSize;	 //  4 bytes
    size_t               dataSize;       //  4 bytes
    OSoundFormat         format;         //  2 bytes
    OSoundChannel        channel;        //  2 bytes
    word                 samplingRate;   //  2 bytes
    word                 bitsPerSample;  //  2 bytes
    size_t               actualDataSize; //  4 bytes
    longword             padding[6];	 // 24 bytes
};

//! provides compatability with the OPEN-R type of the same name
struct OSoundVectorData {
    ODataVectorInfo  vectorInfo;
    OSoundInfo       info[1];

    void SetNumData(size_t ndata)  { vectorInfo.numData = ndata; }
    OSoundInfo* GetInfo(int index) { return &info[index];        }
    byte*       GetData(int index) {
        return ((byte*)&vectorInfo + info[index].dataOffset);
    }
};

//! provides compatability with the OPEN-R type of the same name
struct OFbkImageInfo {          // 32 bytes (total)
    longword      frameNumber;  //  4 bytes
    size_t        dataOffset;   //  4 bytes
    size_t        dataSize;     //  4 bytes
    size_t        width;        //  4 bytes
    size_t        height;       //  4 bytes
    size_t        padding[1];   //  4 bytes
};

//! provides compatability with the OPEN-R type of the same name
struct OFbkImageVectorData {
    ODataVectorInfo  vectorInfo;
    OFbkImageInfo    info[1];

    OFbkImageInfo* GetInfo(int index) { return &info[index]; }
    byte*          GetData(int index) {
        return ((byte*)&vectorInfo + info[index].dataOffset);
    }
};

typedef int OFbkImageLayer;
const OFbkImageLayer ofbkimageLAYER_H = 0;
const OFbkImageLayer ofbkimageLAYER_M = 1;
const OFbkImageLayer ofbkimageLAYER_L = 2;
const OFbkImageLayer ofbkimageLAYER_C = 3;

const int ocdtNUM_CHANNELS  =  8;
typedef longword OCdtChannel;
const OCdtChannel ocdtCHANNEL_UNDEF = 0x00;
const OCdtChannel ocdtCHANNEL0      = 0x01;
const OCdtChannel ocdtCHANNEL1      = 0x02;
const OCdtChannel ocdtCHANNEL2      = 0x04;
const OCdtChannel ocdtCHANNEL3      = 0x08;
const OCdtChannel ocdtCHANNEL4      = 0x10;
const OCdtChannel ocdtCHANNEL5      = 0x20;
const OCdtChannel ocdtCHANNEL6      = 0x40;
const OCdtChannel ocdtCHANNEL7      = 0x80;

const int ocdtMAX_Y_SEGMENT    = 32;
const longword ocdtINIT        = 0x80808080;
const longword ocdtCr_MAX_MASK = 0x0000ff00;
const longword ocdtCr_MIN_MASK = 0x000000ff;
const longword ocdtCb_MAX_MASK = 0xff000000;
const longword ocdtCb_MIN_MASK = 0x00ff0000;

struct OCdtInfo {		                    // 144 bytes
    OCdtChannel   channel;		            //   4 bytes
    longword      table[ocdtMAX_Y_SEGMENT]; // 128 bytes
    longword      padding;		            //   4 bytes
    
    void Init(OCdtChannel chan) {
        channel     = chan;
        for (int i = 0; i < ocdtMAX_Y_SEGMENT; i++) table[i] = ocdtINIT;
    }
    void Set(int y_segment,byte cr_max, byte cr_min, byte cb_max, byte cb_min) {
        longword crMax = (longword)cr_max;
        longword crMin = (longword)cr_min;
        longword cbMax = (longword)cb_max;
        longword cbMin = (longword)cb_min;
        crMax = (crMax <<  8) & ocdtCr_MAX_MASK;
        crMin = (crMin      ) & ocdtCr_MIN_MASK;
        cbMax = (cbMax << 24) & ocdtCb_MAX_MASK;
        cbMin = (cbMin << 16) & ocdtCb_MIN_MASK;
        table[y_segment] =  crMax | crMin | cbMax | cbMin;
    }
};

struct OCdtVectorData{
    ODataVectorInfo  vectorInfo;
    OCdtInfo         info[ocdtNUM_CHANNELS];

    void SetNumData(size_t ndata) { vectorInfo.numData = ndata; }
    OCdtInfo* GetInfo(int index)  { return &info[index];        }
};

/*! @file
 * @brief Defines several data structures for compatability with OPEN-R
 * @author ejt (Creator)
 */

#  endif
#endif
