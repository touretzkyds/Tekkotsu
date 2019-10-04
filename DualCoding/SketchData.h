//-*-c++-*-

#ifndef INCLUDED_SketchData_h
#define INCLUDED_SketchData_h

#include <valarray>
#include <iostream>
#include <stdexcept>
#include <limits> // needed for findMinPlus
#include <map>

#include "Shared/debuget.h" // needed for ASSERT macros

#include "SketchTypes.h"
#include "SketchDataRoot.h"
#include "SketchSpace.h"

namespace DualCoding {

class SketchSpace;
template<typename T> class SketchPool;
template<typename T> class Sketch;

//! Holds the pixels for an individual sketch.

/*! SketchData<T> holds the pixels for an individual sketch of type T, using a valarray<T>.  A collection of
    SketchData<T> objects is maintained in a SketchPool<T>.  Basic functions such as indexed access and empty test
    that are not implemented as Sketch operators are implemented as SketchData<T> member functions.  Sketch<T>
    overrides the -> operator to provide "smart pointer" access to these SketchData<T> functions.   */

template<class T>
class SketchData : public SketchDataRoot {
  //! the valarray which actually stores the image
  std::valarray<T> pixels;

  friend class Sketch<T>;
  friend class SketchPool<T>;

public:
  //! Constructor.  Don't call this.  SketchData objects should only be created and managed by their SketchSpace
  SketchData(SketchSpace *_space);
  ~SketchData();

  //! The type of this sketch.
  virtual SketchType_t getType() const;  // must go here, not in SketchDataRoot, due to templating

  //! Address of the memory area containing the actual pixel data.
  T* getRawPixels() { return &(pixels[0]); }

  //! Address of the memory area containing the actual pixel data.
  const T* getRawPixels() const { return &(pixels[0]); }

  //@{
  //! Indexed access, with bounds checking
  T& at(size_t x);

  //! Subscripted (x,y) access, with bounds checking
  T& at(size_t x, size_t y);

  T& operator[] (size_t idx) { return pixels[idx]; };
  const T& operator[] (size_t idx) const { return pixels[idx]; };
	
  T& operator() (size_t x, size_t y) { return pixels[y*space->getWidth() + x]; };
  const T& operator() (size_t x, size_t y) const { return pixels[y*space->getWidth() + x]; };
	
  //@}

  //! Returns true if all pixels are zero.
  bool empty() const;

  //!@name Sum/Max/Min
  //@{

  //! Sum of pixels
  T sum() const;

  //! Max of pixel values
  T max() const;

  //! Index of first maximum-value pixel
  int findMax() const;

  //! Min of pixel values
  T min() const;

  //! Index of first minimum-value pixel
  int findMin() const;

  //! Min of non-zero pixel values
  T minPlus() const;

  //! Index of first minimum non-zero pixel, or -1 if none
  int findMinPlus() const;

  //! Index of first non-zero pixel, or -1 if none
  int findTrue() const;

  //! Mode (most common) pixel value
  T mode() const;

  //! Mode (most common) non-zero pixel value
  T modePlus() const;

  //@}

  virtual size_t savePixels(char buf[], size_t avail) const; //!< handle copying pixels to buffer

private:
  SketchData (const SketchData& other); //!< never call this
};

template<class T>
SketchData<T>::SketchData(SketchSpace *_space) :
  SketchDataRoot(_space), pixels(_space->getNumPixels()+1) {
  if  ( getType() == sketchUsint || getType() == sketchUint || getType() == sketchFloat )
    colormap = jetMapScaled;
  else
    colormap = segMap;
}

template<class T> SketchData<T>::~SketchData() {}

template <class T>
SketchData<T>::SketchData (const SketchData<T> &other)
  : SketchDataRoot(other.space), pixels(other.pixels) {}

template<>
inline SketchType_t SketchData<bool>:: getType() const { return sketchBool; }
template<>
inline SketchType_t SketchData<unsigned char>::getType() const { return sketchUchar; }
template<>
inline SketchType_t SketchData<unsigned short int>::getType() const { return sketchUsint; }
template<>
inline SketchType_t SketchData<unsigned int>::getType() const { return sketchUint; }
template<>
inline SketchType_t SketchData<float>::getType() const { return sketchFloat; }
template<>
inline SketchType_t SketchData<yuv>::getType() const { return sketchYUV; }

template<class T>
T& SketchData<T>::at(size_t x) {
  if ( x < space->getWidth()*space->getHeight() )
    return pixels[x];
  else
    throw std::out_of_range("Sketch subscript out of bounds");
}

template <class T>
T& SketchData<T>::at(size_t x, size_t y) {
  if ( x < space->getWidth() && y < space->getHeight() )
    return pixels[y*space->getWidth()+x];
  else
    throw std::out_of_range("Sketch subscript out of bounds");
}

template <class T>
bool SketchData<T>::empty() const {
  for ( size_t i=0; i < pixels.size(); i++ )
    if ( pixels[i] != 0 )
      return false;
  return true;
}

template <class T>
T SketchData<T>::sum() const {
	return pixels.sum();
}

template <class T>
T SketchData<T>::max() const {
	return pixels.max();
}

template <class T>
int SketchData<T>::findMax() const {
  T maxval = pixels[0];
  int maxidx = -1;
  const unsigned int length = pixels.size();
  for (unsigned int i = 1; i<length; i++)
    if ( pixels[i] > maxval ) {
      maxidx = i;
      maxval = pixels[i];
    }
  return maxidx;
}

template <class T>
T SketchData<T>::min() const {
	return pixels.min();
}

template <class T>
int SketchData<T>::findMin() const {
  T minval = pixels[0];
  int minidx = 0;
  const unsigned int length = pixels.size();
  for (unsigned int i = 1; i<length; i++)
    if ( pixels[i] < minval ) {
      minidx = i;
      minval = pixels[minidx];
    }
  return minidx;
}

template <class T>
T SketchData<T>::minPlus() const {
  T result = 0;
  unsigned int i = 0;
  const unsigned int length = pixels.size();
  // find first positive value
  for (; i<length; i++)
    if ( pixels[i] > 0 ) {
      result = pixels[i];
      break;
    }
  // now check for values less than this
  for (; i<length; i++)
    if ( pixels[i] > 0 )
      if ( result < pixels[i] )
	result = pixels[i];
  return result;
}

template <class T>
int SketchData<T>::findMinPlus() const {
  T minval=std::numeric_limits<T>::max();
  int minidx = -1;
  const unsigned int length = pixels.size();
  // count backwards so we return the earliest value even if it's std::numeric_limits<T>::max();
  for (unsigned int i=length; i!=0; ) {
    --i;
    if ( pixels[i] != 0 && pixels[i] <= minval ) {
      minidx = i;
      minval = pixels[i];
    }
  }
  return minidx;
}

template <class T>
int SketchData<T>::findTrue() const {
  const unsigned int length = pixels.size();
  for (unsigned int i=0; i<length; i++)
    if ( pixels[i] != 0 )
      return i;
  return -1;
}

template <class T>
T SketchData<T>::mode() const {
	std::map<T,size_t> hist;
	const T* p=&pixels[0], *end=&pixels[pixels.size()];
	while(p!=end)
		hist[*p++]++;
	T maxval=T();
	size_t maxcnt=0;
	for(typename std::map<T,size_t>::const_iterator it=hist.begin(); it!=hist.end(); ++it) {
		if(maxcnt<=it->second) {
			maxval=it->first;
			maxcnt=it->second;
		}
	}
	return maxval;
}

template <class T>
T SketchData<T>::modePlus() const {
	std::map<T,size_t> hist;
	for(const T* p=&pixels[0], *end=&pixels[pixels.size()]; p!=end; ++p)
		if(*p!=0)
			hist[*p]++;
	T maxval=T();
	size_t maxcnt=0;
	for(typename std::map<T,size_t>::const_iterator it=hist.begin(); it!=hist.end(); ++it) {
		if(maxcnt<=it->second) {
			maxval=it->first;
			maxcnt=it->second;
		}
	}
	return maxval;
}


// ================================================================

template <class T>
size_t SketchData<T>::savePixels(char buf[], size_t avail) const
{
  const size_t imglength  = getWidth() * getHeight() * sizeof(T);
  ASSERTRETVAL(imglength<=avail,"Insufficient buffer space for image",0);
  memcpy(buf,(const unsigned char*)&(pixels[0]),imglength);
  return imglength;
}

template<>
inline size_t SketchData<float>::savePixels(char buf[], size_t avail) const
{
  const size_t imglength  = getWidth() * getHeight() * 4;
  ASSERTRETVAL(imglength<avail,"Insufficient buffer space for image",0);
  float temp;
  const char* tptr = (const char*)(&temp);
  char *bufptr = buf;
  const unsigned int np = getNumPixels();
  for(unsigned int i=0; i<np; i++) {
    //temp = std::min(int(fmax(0.f, pixels[i])), 0x7fffff);
    temp = pixels[i];
    *bufptr++ = tptr[0];
    *bufptr++ = tptr[1];
    *bufptr++ = tptr[2];
    *bufptr++ = tptr[3];
  }
  return imglength;
}

#ifdef __POWERPC__
//bool can be 4 bytes on PowerPC systems
template<>
inline unsigned int SketchData<bool>::savePixels(char buf[], unsigned int avail) const
{
  const unsigned int imglength  = getWidth() * getHeight() * sizeof(char);
  ASSERTRETVAL(imglength<avail,"Insufficient buffer space for image",0);
  if(sizeof(bool)==sizeof(char))
    memcpy(buf,(const unsigned char*)&(pixels[0]),imglength);
  else
    for(unsigned int i=0; i<imglength; ++i)
      buf[i]=pixels[i]; //do manual copy to ensure one byte per pixel
  return imglength;
}
#endif

} // namespace

/*! @file
 * @brief A resource which holds the image date for a Sketch, managed collectively by a SketchSpace
 * @author neilh (Creator)
 */

#endif
