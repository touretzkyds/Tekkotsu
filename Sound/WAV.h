/*! @file
 * @brief Allows you to load WAV files from the memory stick
 * @author Sony (Creator)
 *
 * This file is from the SoundPlay example from the Sony sample code, with a few if any modifications.
 * Here's the license Sony provided with it:
 *
 * Copyright 2002,2003 Sony Corporation 
 *
 * Permission to use, copy, modify, and redistribute this software for
 * non-commercial use is hereby granted.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 */

#ifndef WAV_h_DEFINED
#define WAV_h_DEFINED

#include "Shared/ODataFormats.h"

enum WAVError {
    WAV_SUCCESS,
    WAV_FAIL,
    WAV_NOT_RIFF,
    WAV_NOT_WAV,
    WAV_FORMAT_NOT_SUPPORTED,
    WAV_CHANNEL_NOT_SUPPORTED,
    WAV_SAMPLINGRATE_NOT_SUPPORTED,
    WAV_BITSPERSAMPLE_NOT_SUPPORTED,
    WAV_SIZE_NOT_ENOUGH,
};

class WAV {
public:
    WAV();
    WAV(byte* addr);
    ~WAV() {}

    WAVError Set(byte *addr);
    WAVError CopyTo(OSoundVectorData* data);
    WAVError Rewind();

    unsigned int    GetSamplingRate()  { return soundInfo.samplingRate;  }
    unsigned int    GetBitsPerSample() { return soundInfo.bitsPerSample; }
    size_t GetSoundUnitSize() { return soundUnitSize;           }
		
		byte*  GetDataStart()     { return dataStart;               }
		byte*  GetDataEnd()       { return dataEnd;                 }

private:
    longword get_longword(byte* addr);
    word     get_word(byte* addr);

    // 8KHz 8bits MONO (8 * 1 * 1 * 32ms = 256)
    static const size_t MONO8K8B_UNIT_SIZE  = 256;

    // 16KHz 16bits MONO (16 * 2 * 1 * 32ms = 1024)
    static const size_t MONO16K16B_UNIT_SIZE  = 1024;

    static const size_t FMTSIZE_WITHOUT_EXTINFO = 16;

    OSoundInfo soundInfo;
    size_t     soundUnitSize;
    byte*      dataStart;
    byte*      dataEnd;
    byte*      dataCurrent;

		WAV(const WAV&); //!< don't call
		WAV& operator=(const WAV&); //!< don't call
};

#endif // WAV_h_DEFINED
