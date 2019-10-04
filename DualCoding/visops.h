//-*-c++-*-
#ifndef _VISOPS_H_
#define _VISOPS_H_

#include "SketchTypes.h"
#include "Sketch.h"
#include "SketchSpace.h"

#include "ShapeLine.h"

namespace DualCoding {
  class SketchIndices;
}

//! Visual routines operators, used in DualCoding.
namespace visops {

  using namespace DualCoding;
  using DualCoding::uchar;
  using DualCoding::uint;

  //! Connectivity used by oldlabelcc and neighborsum.
  enum Connectivity_t { FourWayConnect, EightWayConnect };

  //!@name Sketch creation
  //@{

  //! Returns an all-zero Sketch<bool> in the specified sketch space
  Sketch<bool> zeros(SketchSpace& space);

  //! Returns an all-zero Sketch<bool> of same size as parent @a sketch
  Sketch<bool> zeros(const SketchRoot& sketch);

  //! Returns a deep copy of the sketch: actually copies the pixels
  template<class T>
  Sketch<T> copy(const Sketch<T>& other) {
    Sketch<T> result("copy("+other->getName()+")", other);  // will inherit from parent
    *result.pixels = *other.pixels;  // valarray assignment
    return result;
  }

  //@}

  //!@name Min/max functions
  //@{

  //! Max of each pixel with a constant
  template<class T>
  Sketch<T> max(const Sketch<T>& src, const T value) {
    Sketch<T> result("max(const)",src);
    for ( unsigned int i = 0; i < src.pixels->size()-1; i++ ) 
      (*result.pixels)[i] = std::max((*src.pixels)[i],value);
    return result;
  }

  //! Max of each pixel with a constant
  template<class T>
  Sketch<T> max(const Sketch<T>& src, const int value) {
    return visops::max(src, (T)(value));
  }


  //! Pixel-wise max of two sketches
  template<class T>
  Sketch<T> max(const Sketch<T>& arg1, const Sketch<T>& arg2) {
    Sketch<T> result("max("+arg1->getName()+","+arg2->getName()+")",arg1);
    for ( unsigned int i = 0; i < arg1.pixels->size()-1; i++ ) 
      (*result.pixels)[i] = std::max((*arg1.pixels)[i],(*arg2.pixels)[i]);
    return result;
  }

  //! Min of each pixel with a constant
  template<class T>
  Sketch<T> min(const Sketch<T>& src, const T value) {
    Sketch<T> result("min(const)",src);
    for ( unsigned int i = 0; i < src.pixels->size()-1; i++ ) 
      (*result.pixels)[i] = std::min((*src.pixels)[i],value);
    return result;
  }

  //! Min of each pixel with a constant
  template<class T>
  Sketch<T> min(const Sketch<T>& src, const int value) {
    return visops::min(src, (T)(value));
  }

  //! Pixel-wise min of two sketches
  template<class T>
  Sketch<T> min(const Sketch<T>& arg1, const Sketch<T>& arg2) {
    Sketch<T> result("min("+arg1->getName()+","+arg2->getName()+")",arg1);
    for ( unsigned int i = 0; i < arg1.pixels->size()-1; i++ ) 
      (*result.pixels)[i] = std::min((*arg1.pixels)[i],(*arg2.pixels)[i]);
    return result;
  }

  //@}

  //!@name Region filling
  //@{

  //! Fills a region bounded by borders, starting at position given by index
  Sketch<bool> seedfill(const Sketch<bool>& borders, size_t index);

  //! Fills the exterior of region bounded by borders, starting from the edges of the sketch; border pixels are not filled
  Sketch<bool> fillExterior(const Sketch<bool>& borders);

  //! Fills the interior of region bounded by borders, i.e., pixels not reachable from the edges of the sketch; border pixels are not filled
  Sketch<bool> fillInterior(const Sketch<bool>& borders);

  //@}

  //!@name Miscellaneous functions
  //@{

  //! Returns all the pixels of the named color.
  Sketch<bool> colormask(const Sketch<uchar>& src, const std::string &colorname);

  //! Returns all the pixels with the specified color index.
  Sketch<bool> colormask(const Sketch<uchar>& src, color_index cindex);

  //! For each pixel, calculate the sum of its neighbors.
  /*! @param im Sketch to use as input.
   *  @param connectivity the type of neighbor connectivity to use */
  Sketch<uchar> neighborSum(const Sketch<bool> &im, Connectivity_t connectivity=EightWayConnect);
  
  //! Produces a filled in image based on the Sketch, using 8-way connectivity.
  /*! @param im The sketch to which to apply the function.
   *  @param iter Number of times to perform the fillin operation.
   *  @param min_thresh Fill in pixel if has at least this many neighbors.
   *  @param max_thresh Fill in pixel if has fewer than this many neighbors.
   *  @param remove_only Set to true if you know you will only be deleting pixels for a speedup */
  Sketch<bool> fillin(const Sketch<bool> &im, int iter, 
		      uchar min_thresh, uchar max_thresh,
		      bool remove_only=false);
  
  //@}

  //!@name Wavefront algorithms: distance, connected components
  //@{

  /*! @brief Calculates the distance from each pixel in the image to the closest
    true pixel in destination @a dest, using the wavefront algorithm.
    Obstacles indicated by true values in pixels of @a obst.  Note: use maxdist=width+height
    if you want the result to be viewable with the jetMapScaled colormap.
  */
  Sketch<uint> bdist(const Sketch<bool> &dest, const Sketch<bool> &obst, 
		      const uint maxdist=(uint)-1);
		      
  typedef std::pair<int,int> xyPair;
  std::vector<xyPair> boundaryPoints(const Sketch<uint>& dest, uint& mindist, const uint maxval=(uint)-1);
  bool radiate(const xyPair center, Sketch<uint>& dest, const Sketch<bool>& obst, const uint maxval=(uint)-1);
  Sketch<uint> ebdist(const Sketch<bool> &dest, const Sketch<bool> &obst, 
		      const uint maxdist=(uint)-1, const uint time=3);

  //! Manhattan distance to the nearest true pixel in @a dest
  /*! Should calculate the Manhattan distance from each pixel in the image
   *  to the closest true pixel in dest, using a linear-time algorithm.
   *  Currently calculates Manhattan distance, which is good enough.
   *  Should be used instead of bdist if not concerned about obstacles. */
  Sketch<uint> mdist(const Sketch<bool> &dest);
	
	//! Euclidean distance to the nearest true pixel in @a dest
	/*! Should calculate the Euclidean distance from each pixel in the image
	 *  to the closest true pixel in dest, using a linear-time algorithm.
	 *  Currently calculates Manhattan distance, which is good enough.
	 *  Should be used instead of bdist if not concerned about obstacles. */
	Sketch<float> edist(const Sketch<bool> &dest);
  
  //! Connected components labeling using CMVision.  Components numbered sequentially from 1.
  Sketch<uint> labelcc(const Sketch<bool>& source, int minarea=1);
  
  //! Old connected-components code written using pure sketch primitives.
  /*! Returns a connected-components labeling of the foreground.
      Each different foreground region will contain a unique positive integer. 
      No guarantees on the integer values. */
  Sketch<uint> oldlabelcc(const Sketch<bool>& source, 
		      Connectivity_t connectivity=EightWayConnect);

  //! Each pixel of the result is the area of that connected component.
  Sketch<uint> areacc(const Sketch<bool>& source, Connectivity_t connectivity=EightWayConnect);

  //! Each pixel of the result is the area of that connected component.
  Sketch<uint> areacc(const Sketch<uint>& labels);

  //! Low-pass filter by eliminating small regions
  Sketch<bool> minArea(const Sketch<bool>& sketch, int minval=5);

  //@}

  //! @name Masking and conditional assignment
  //@{
  //! Returns pixels of @a A masked by bool sketch @a B
  template<typename T>
  Sketch<T> mask(const Sketch<T> &A, const Sketch<bool> &B) {
    Sketch<T> result("mask("+A->getName()+","+B->getName()+")", A);
    T* Aptr = &(*A.pixels)[0];
    T* Rptr = &(*result.pixels)[0];
    T* Rend = &(*result.pixels)[result->getNumPixels()];
    unsigned int idx = 0;
    while ( Rptr != Rend )
      *Rptr++ = B[idx++] * (*Aptr++);
    return result;
  }

  //! Result holds non-zero pixels of @a A, with zero pixels filled in by @a B.
  /*! Equivalent to writing maskedAssign(A,A==0,B) */
  template<typename T>
  Sketch<T> ifNot(const Sketch<T> &A, const Sketch<T> &B) {
    Sketch<T> result("ifNot("+A->getName()+","+B->getName()+")", A);
    T* Aptr = &(*A.pixels)[0];
    T* Bptr = &(*B.pixels)[0];
    T* Rptr = &(*result.pixels)[0];
    T* Rend = &(*result.pixels)[result->getNumPixels()];
    while ( Rptr != Rend ) {
      *Rptr++ = ( *Aptr != 0 ) ? *Aptr : * Bptr;
      *Aptr++; Bptr++;
    }
    return result;
  }

  //! Returns a result where pixels of @a sketch for which @a mask is true have been replaced by @a value.
  template<typename T, typename Tv>
  Sketch<T> maskedAssign(const Sketch<T> &sketch, const Sketch<bool> &mask, const Tv value) {
    Sketch<T> result("maskedAssign("+sketch->getName()+")",sketch);
    T* Psrc = &(*sketch.pixels)[0];
    T* Pdest = &(*result.pixels)[0];
    T* Edest = &(*result.pixels)[sketch->getNumPixels()];
    bool* Pmask =&(*mask.pixels)[0];
    const T val = (T)value;
    while ( Pdest != Edest ) {
      *Pdest++ = *Pmask++ ? val : *Psrc;
      Psrc++;
    }
    return result;
  }

  //! Returns a result where pixels of @a sketch for which @a mask is true have been replaced by corresponding pixels of @a value.
  template<typename T>
  Sketch<T> maskedAssign(const Sketch<T> &sketch, const Sketch<bool> &mask, const Sketch<T> &value) {
    Sketch<T> result("maskedAssign("+sketch->getName()+")",sketch);
    T* Psrc = &(*sketch.pixels)[0];
    T* Pdest = &(*result.pixels)[0];
    T* Edest = &(*result.pixels)[sketch->getNumPixels()];
    bool* Pmask = &(*mask.pixels)[0];
    T* Pval = &(*value.pixels)[0];
    while ( Pdest != Edest ) {
      *Pdest++ = *Pmask++ ? *Pval : *Psrc;
      Pval++;
      Psrc++;
    }
    return result;
  }
  //@}

  //!@name Edge and symmetry detection
  //@{

  //! Simple edge finding.  Use SUSAN for more sophisticated edge detection.
  /*! This edge-finding algorithm is inefficient, and produces offset results
   *  for top and left edges.  Should replace it with something better. */
  Sketch<bool> edge(const Sketch<bool> &im); 
  
  //! Horizontal symmetry points.
  /*! @brief Returns non-zero values along points of horizontal symmetry, with
   *  each of these values equal to the distance to the symmetric points.
   *  @param sketch The sketch to which to apply the function.
   *  @param minskip The min accepted distance between pixels for symmetry.
   *  @param maxskip The max accepted distance between pixels for symmetry. */
  Sketch<bool> horsym(const Sketch<bool>& sketch, 
		     size_t minskip=3, size_t maxskip=80);

  //! Vertical symmetry points.
  /*! @brief Returns non-zero values along points of vertical symmetry, with
   *  each of these values equal to the distance to the symmetric points.
   *  @param sketch The sketch to which to apply the function.
   *  @param minskip The min accepted distance between pixels for symmetry.
   *  @param maxskip The max accepted distance between pixels for symmetry. */
  Sketch<bool> versym(const Sketch<bool>& sketch, 
		     size_t minskip=3, size_t maxskip=80);
  
  /*! @brief returns a skeleton of @a sketch, with pixel values corresponding to 
   *  distance of symmetry */
  Sketch<bool> skel(const Sketch<bool>& sketch); 

  //@}
  
  //!@name Sketch dissection
  //@{
  //! Half-plane functions fill in the half plane on one side of a line.
  //@{
  Sketch<bool> leftHalfPlane(const Shape<LineData> &ln);

  Sketch<bool> rightHalfPlane(const Shape<LineData> &ln);

  Sketch<bool> topHalfPlane(const Shape<LineData> &ln);

  Sketch<bool> bottomHalfPlane(const Shape<LineData> &ln);
  //@}

  //! Returns a copy of im except that its pixels within offset from boundaries are removed
  Sketch<bool> non_bounds(const Sketch<bool>& im, int offset);
  //@}

  //!@name Image manipulation primitives
  //@{
  //! Runs the SUSAN edge detector on a grayscale image
  Sketch<uchar> susan_edges(const Sketch<uchar>& im, int brightness);

  //! Returns a Sketch<bool> indicating edge points found by SUSAN
  Sketch<bool> susan_edge_points(const Sketch<uchar>& im, int brightness);
  
  //! Convolves a kernel with an image.
  Sketch<uint> convolve(const Sketch<uchar> &sketch, Sketch<uchar> &kernel, 
			 int i, int j, int width, int height);

  //! Convolves a kernel with an image, normalizing the kernel to zero mean.
  Sketch<uint> templateMatch(const Sketch<uchar> &sketch, Sketch<uchar> &kernel, 
			 int i, int j, int width, int height);

  //@}

} // namespace

#endif
