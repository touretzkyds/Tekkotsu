//-*-c++-*-
#ifndef INCLUDED_SketchSpace_h
#define INCLUDED_SketchSpace_h

#include "Shared/fmatSpatial.h"
#include "Shared/BoundingBox.h"
#include "Vision/colors.h"  // for rgb and yuv structs

#include "SketchTypes.h"
#include "ShapeTypes.h"
#include "SketchPool.h"
#include "SketchIndices.h"

namespace DualCoding {

class ShapeSpace;
template<typename T> class Sketch;
class ViewerConnection;

//! Holds a collection of sketches of various types
/*! All the sketches in a SketchSpace have the same dimensions (width and height).
   They are organized into pools, managed by SketchPool<T> instances.
   Each SketchSpace has a dual, called a ShapeSpace.  Several standard
   SketchSpace/ShapeSpace pairs are built in to VisualRoutinesBehavior.
   The most basic is @a camSkS, the camera sketch space.   */

class SketchSpace {
public:
  std::string name;    //!< name of this SketchSpace

private:
  unsigned int width;	  //!< pixels along x axis
  unsigned int height;  //!< pixels along y axis
  unsigned int numPixels; //!< total pixels = width * height
  fmat::Transform Tmat; //!< transformation matrix for rendering shapes
  fmat::Transform Tmatinv; //!< inverse transformation matrix for extracting shapes
    
  int idCounter; //!< Incremented with each new Sketch, to guarantee a unique ID.
  int refreshCounter; //!< Incremented each time SketchGUI refreshes the sketch/shape list
  
  ShapeSpace* dualSpace; //!< Pointer to the ShapeSpace associated with this SketchSpace
  
  //! Pool for one of the SketchData<T> classes
  //@{
  SketchPool<bool>  boolPool;
  SketchPool<uchar> ucharPool;
  SketchPool<usint> usintPool;
  SketchPool<uint> uintPool;
  SketchPool<float> floatPool; 
  SketchPool<yuv> yuvPool; 
  //@}
  
public:
  //! The value assigned to out-of-bounds indices: @a numPixels, i.e., one beyond the last image pixel.
  int dummy;

  //!@name Pre-generated indices for different directions
  //@{
  Sketch<skindex> *idx;
  Sketch<skindex> *idxN;
  Sketch<skindex> *idxS;
  Sketch<skindex> *idxE;
  Sketch<skindex> *idxW;
  Sketch<skindex> *idxNE;
  Sketch<skindex> *idxNW;
  Sketch<skindex> *idxSE;
  Sketch<skindex> *idxSW;
  //@}
  
  SketchSpace(std::string const _name, ReferenceFrameType_t _refFrameType,
	      int const init_id, size_t const _width, size_t const _height);
  
  ~SketchSpace();
  
  ShapeSpace& getDualSpace() const { return *dualSpace; }
  
  //! dumps contents of sketch space
  void dumpSpace() const;
  
  //! Clears out viewable Sketches, and also retained sketches if argument is true
  void clear(bool clearRetained=true);

  //! returns the width of contained images, in pixels
  unsigned int getWidth() const { return width; }
  //! returns the height of contained images, in pixels
  unsigned int getHeight() const { return height; }
  //! returns the number of pixels of images in this space
  unsigned int getNumPixels() const { return numPixels; }
  
  int getRefreshCounter() const { return refreshCounter; }
  void bumpRefreshCounter() { ++refreshCounter; }

  //! creates #idx if needed
  void requireIdx();
	
  //! creates #idxN, #idxS, #idxE, and #idxW if needed
  void requireIdx4way();

  //! creates #idxNE, #idxNW, #idxSE, and #idxSW, plus NSEW cases via requireIdx4way(), if needed
  void requireIdx8way();
	
  //! frees the idx members
  void freeIndexes();

  //! change the size of sketches in this sketch space (discards all existing sketches)
  void resize(const size_t new_width, const size_t new_height);

  //! return the ShapeSpace-to-SketchSpace coordinate transformation matrix
  fmat::Transform& getTmat() { return Tmat; }

  //! return the SketchSpace-to-ShapeSpace coordinate transformation matrix
  fmat::Transform& getTmatinv() { return Tmatinv; }

  //! set the ShapeSpace-to-SketchSpace coordinate transformation matrix
  void setTmat(const fmat::Transform &mat);

  //! Scale and then translate shape space to sketch space coordinates
  void setTmat(float scale=1, float tx=0, float ty=0);

  void setTmat(const BoundingBox2D &b);

  //! apply the ShapeSpace-to-SketchSpace coordinate transformation to a vector
  void applyTmat(fmat::Column<3> &vec);

  //! apply the SketchSpace-to-ShapeSpace coordinate transformation to a vector
  void applyTmatinv(fmat::Column<3> &vec);

  //! Returns the SketchPool of appropriate type for the calling Sketch
  //@{
  SketchPool<bool>&  get_pool(const Sketch<bool>&)  { return boolPool; }
  SketchPool<uchar>& get_pool(const Sketch<uchar>&) { return ucharPool; }
  SketchPool<usint>& get_pool(const Sketch<usint>&){ return usintPool; }
  SketchPool<uint>&  get_pool(const Sketch<uint>&)  { return uintPool; }
  SketchPool<float>& get_pool(const Sketch<float>&) { return floatPool; }
  SketchPool<yuv>& get_pool(const Sketch<yuv>&) { return yuvPool; }
  
  //@}
  
  //!@name SketchGUI interface
  //@{
  
  //! Socket and port info for communication with a sketch viewer GUI.
  ViewerConnection *viewer;

  //! Returns a string describing the shape-to-sketch transformation matrix.
  std::string getTmatForGUI();

  //! Returns a string containing a list of all the sketches and their attributes
  std::string getSketchListForGUI();

  //! Returns a pointer to the sketch with specified ID number; null if not found
  SketchDataRoot* retrieveSketch(int const id);
  //@}
  
protected:
  //! helper function to ensure indices of idx Sketches are proper
  void setIdx(Sketch<skindex>& indices, int const x, int const y, int const indexval);
  
  // We don't want clients accidentally copying or assigning SketchSpace.
  SketchSpace(const SketchSpace&); //!< never call this
  SketchSpace& operator= (const SketchSpace&); //!< never call this
};

} // namespace

#endif
