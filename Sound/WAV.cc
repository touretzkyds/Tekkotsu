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

#ifdef PLATFORM_APERIOS
#  include <OPENR/OPENR.h>
#  include <OPENR/OSyslog.h>
#endif
#include "WAV.h"
#include <cstring>
#include <stdio.h>

WAV::WAV() : soundInfo(), soundUnitSize(0), dataStart(0), dataEnd(0), dataCurrent(0)
{
}

WAV::WAV(byte* addr) : soundInfo(), soundUnitSize(0), dataStart(0), dataEnd(0), dataCurrent(0)
{
    Set(addr);
}

WAVError
WAV::Set(byte *addr)
{
    //
    // Check Wav Header
    //
    if (strncmp((char *)addr, "RIFF", 4)) return WAV_NOT_RIFF;
    addr += 4;

    longword length = get_longword(addr);
    addr += sizeof(longword);
		//    OSYSDEBUG(( "length = %x\n", length));

    if (strncmp((char *)addr, "WAVE", 4)) return WAV_NOT_WAV;
    length -= 4;
    addr += 4;

    //
    // Check Chunk
    //
    while (length > 8) {

        size_t chunksize;
        char *buf = (char *)addr;
    
        addr += 4;

        chunksize = get_longword(addr);
        addr += sizeof(longword);
        length -= chunksize + 8;

        if (!strncmp(buf, "fmt ", 4)) {

            //
            // Format Chunk
            //

            //
            // Check WAV Type
            //
            soundInfo.format = (OSoundFormat)get_word(addr);
            addr += sizeof(word);
            if (soundInfo.format != osoundformatPCM) {
                printf(("WAV_FORMAT_NOT_SUPPORTED\n"));
                return WAV_FORMAT_NOT_SUPPORTED;
            }

            //
            // Channel
            //
            soundInfo.channel = (OSoundChannel)get_word(addr);
            addr += sizeof(word);
            if (soundInfo.channel != osoundchannelMONO) {
                printf(("WAV_CHANNEL_NOT_SUPPORTED\n"));
                return WAV_CHANNEL_NOT_SUPPORTED;
            }

            //
            // Sampling Rate
            //
            longword frq = get_longword(addr);
            addr += sizeof(longword);
            soundInfo.samplingRate = (word)frq;
            if (soundInfo.samplingRate != 8000 &&
                soundInfo.samplingRate != 16000) {
                printf(("WAV_SAMPLINGRATE_NOT_SUPPORTED\n"));
                return WAV_SAMPLINGRATE_NOT_SUPPORTED;
            }

            //
            // DataSize Per sec
            //
            addr += sizeof(longword);

            //
            // Block Size
            //
            addr += sizeof(word);

            //
            // Bits Of Sample
            //
            soundInfo.bitsPerSample = get_word(addr);
            addr += sizeof(word);
            soundInfo.bitsPerSample *= soundInfo.channel;
            if (soundInfo.bitsPerSample != 8 &&
                soundInfo.bitsPerSample != 16) {
                printf(("WAV_BITSPERSAMPLE_NOT_SUPPORTED\n"));
                return WAV_BITSPERSAMPLE_NOT_SUPPORTED;
            }

            //
            // Skip Extentded Infomation
            //
            addr += chunksize - FMTSIZE_WITHOUT_EXTINFO;
            
						//            OSYSDEBUG(( "fmt chunksize = %d\n", chunksize));
						//            OSYSDEBUG(( "samplingRate  = %d\n", soundInfo.samplingRate));
						//            OSYSDEBUG(( "bitsPerSample = %d\n", soundInfo.bitsPerSample));
            
        } else if (!strncmp(buf, "data", 4)) {

            //
            // Data Chunk
            //
					//            OSYSDEBUG(( "data chunksize = %d\n", chunksize));
            soundInfo.dataSize = chunksize;
            dataStart = dataCurrent = addr;
            dataEnd = dataStart + soundInfo.dataSize;
            break;

        } else {

            //
            // Fact Chunk
            //
            addr += chunksize;
        }
    }

    int rate = soundInfo.samplingRate;
    int bits = soundInfo.bitsPerSample;
    if (rate == 8000 && bits == 8) {
        soundUnitSize = MONO8K8B_UNIT_SIZE;
    } else if (rate == 16000 && bits == 16) {
        soundUnitSize = MONO16K16B_UNIT_SIZE;
    }
    
    return WAV_SUCCESS;
}

WAVError
WAV::CopyTo(OSoundVectorData* data)
{
    if (dataCurrent >= dataEnd) return WAV_FAIL;
    
    OSoundInfo* sinfo = data->GetInfo(0);
    if (soundUnitSize > sinfo->maxDataSize) {
        printf(("WAV_SIZE_NOT_ENOUGH "));
        return WAV_SIZE_NOT_ENOUGH;
    }

    sinfo->dataSize      = soundUnitSize;
    sinfo->format        = soundInfo.format;
    sinfo->channel       = soundInfo.channel;
    sinfo->samplingRate  = soundInfo.samplingRate;
    sinfo->bitsPerSample = soundInfo.bitsPerSample;

    byte* src  = dataCurrent;
    byte* dest = data->GetData(0);
    byte* end;
    int num = (int)(dataEnd - dataCurrent);

    if (soundUnitSize <= (unsigned int)num) {
    
        end = dest + soundUnitSize;
        if (soundUnitSize == MONO8K8B_UNIT_SIZE) {
            while (dest < end) {
                *dest++ = *src++ ^ 0x80; // offset binary -> signed char
            }
        } else { // MONO16K16B_UNIT_SIZE
            while (dest < end) {
                *dest++ = *src++;
            }
        }
        dataCurrent += soundUnitSize;

    } else {

        end = dest + num;
        if (soundUnitSize == MONO8K8B_UNIT_SIZE) {
            while (dest < end) {
                *dest++ = *src++ ^ 0x80; // offset binary -> signed char
            }
        } else { // MONO16K16B_UNIT_SIZE
            while (dest < end) {
                *dest++ = *src++;
            }
        }
        memset(dest, 0x0, soundUnitSize - num);
        dataCurrent = dataEnd;

    }

    return WAV_SUCCESS;
}

WAVError
WAV::Rewind()
{
    dataCurrent = dataStart;
    return WAV_SUCCESS;
}

longword
WAV::get_longword(byte* ptr)
{
    longword lw0 = (longword)ptr[0];
    longword lw1 = (longword)ptr[1] << 8;
    longword lw2 = (longword)ptr[2] << 16;
    longword lw3 = (longword)ptr[3] << 24;
    return lw0 + lw1 + lw2 + lw3;
}

word
WAV::get_word(byte* ptr)
{
    word w0 = (word)ptr[0];
    word w1 = (word)ptr[1] << 8;
    return w0 + w1;
}
