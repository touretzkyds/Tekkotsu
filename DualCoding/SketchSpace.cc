#include <iostream>
#include <sstream> // for ostringstream

#include "SketchSpace.h"
#include "ShapeRoot.h"
#include "ShapeSpace.h"
#include "Sketch.h"
#include "BaseData.h"
#include "ViewerConnection.h"
#include "Vision/colors.h" // for rgb and yuv structs

using namespace std;

namespace DualCoding {

SketchSpace::SketchSpace(std::string const _name,  ReferenceFrameType_t _refFrameType,
			 int const init_id, size_t const _width, size_t const _height) :
  name(_name), width(_width), height(_height), numPixels(_width*_height),
  Tmat(fmat::Transform()),  Tmatinv(fmat::Transform()),
  idCounter(1), refreshCounter(1),
  dualSpace(new ShapeSpace(this,init_id,_name,_refFrameType)),
  boolPool(this,"bool"), ucharPool(this,"uchar"), 
  usintPool(this,"usint"), uintPool(this,"uint"), floatPool(this,"float"), yuvPool(this,"yuv"),
  dummy(numPixels), idx(NULL),
  idxN(NULL), idxS(NULL), idxE(NULL), idxW(NULL), 
  idxNE(NULL), idxNW(NULL), idxSE(NULL), idxSW(NULL),
  viewer(new ViewerConnection)
{
}

void SketchSpace::requireIdx() {
  if ( idx == NULL ) {
    idx = new Sketch<skindex>(*this, "idx");
    uint i = 0;
    for (size_t y = 0; y < height; y++)
      for (size_t x = 0; x < width; x++)
	setIdx(*idx, x, y, i++);
  }
}

void SketchSpace::requireIdx4way() {
  if ( idxN == NULL) {
    idxN = new Sketch<skindex>(*this,"idN");
    idxS = new Sketch<skindex>(*this,"idS");
    idxE = new Sketch<skindex>(*this,"idE");
    idxW = new Sketch<skindex>(*this,"idW");
    int i = 0;
    for (size_t y = 0; y < height; y++) {
      for (size_t x = 0; x < width; x++) {
      setIdx(*idxN, x, y, i-width);
      setIdx(*idxS, x, y, i+width);
      setIdx(*idxW, x, y, i-1);
      setIdx(*idxE, x, y, i+1);
      i++;
      }
    }
  }
}

void SketchSpace::requireIdx8way() {
  requireIdx4way();
  if ( idxNE == NULL) {
    idxNE = new Sketch<skindex>(*this,"idNE");
    idxNW = new Sketch<skindex>(*this,"idNW");
    idxSE = new Sketch<skindex>(*this,"idSE");
    idxSW = new Sketch<skindex>(*this,"idSW");
    int i = 0;
    for (size_t y = 0; y < height; y++) {
      for (size_t x = 0; x < width; x++) {
      setIdx(*idxNE, x, y, i-width+1);
      setIdx(*idxNW, x, y, i-width-1);
      setIdx(*idxSE, x, y, i+width+1);
      setIdx(*idxSW, x, y, i+width-1);
      i++;
      }
    }
  }
}

void SketchSpace::freeIndexes() {
  delete idx;
  idx=NULL;
  delete idxN; delete idxS; delete idxE; delete idxW; 
  idxN=idxS=idxE=idxW=NULL;
  delete idxNE; delete idxNW; delete idxSE; delete idxSW;
  idxNE=idxNW=idxSE=idxSW=NULL;
}


void SketchSpace::resize(const size_t new_width, const size_t new_height) {
	// delete all the old stuff
	freeIndexes();
  boolPool.deleteElements();
  ucharPool.deleteElements();
  usintPool.deleteElements();
  uintPool.deleteElements();
  floatPool.deleteElements();
  yuvPool.deleteElements();
  // now set the new dimensions
  width = new_width;
  height = new_height;
  numPixels = new_width * new_height;
  dummy = numPixels;
}

SketchSpace::~SketchSpace() { 
  delete dualSpace;
  //printf("Destroying SketchSpace %s at 0x%p\n",name.c_str(),this);
  freeIndexes();
  delete viewer;
}


void SketchSpace::dumpSpace() const {
  boolPool.dumpPool();
  ucharPool.dumpPool();
  usintPool.dumpPool();
  uintPool.dumpPool();
  floatPool.dumpPool();
  yuvPool.dumpPool();
}

void SketchSpace::clear(bool clearRetained) {
  boolPool.clear(clearRetained);
  ucharPool.clear(clearRetained);
  usintPool.clear(clearRetained);
  uintPool.clear(clearRetained);
  floatPool.clear(clearRetained);
  yuvPool.clear(clearRetained);
}

void SketchSpace::setTmat(const fmat::Transform &mat) {
  Tmat = mat;
  Tmatinv = mat.inverse();
  // Drop all cached renderings since they're no longer valid.  Note that
  // any user-created rendering sketches will still exist.
  for ( std::vector<ShapeRoot>::iterator it = dualSpace->allShapes().begin();
  	it != dualSpace->allShapes().end(); it++ )
    (*it)->deleteRendering();
}

void SketchSpace::setTmat(float scale, float tx, float ty) {
  // scale must not be 0 (fmat::Transform doesn't store scale factor, would have to use full Matrix<4,4>)
  setTmat(fmat::Transform(fmat::Matrix<3,3>::identity() * scale, fmat::pack(tx,ty,0.f)));
}

void SketchSpace::setTmat(const BoundingBox2D &b) {
  float const scale = b.getDimensions().min();
  setTmat(scale, -b.min[0], -b.min[1]);
}    

void SketchSpace::applyTmat(fmat::Column<3> &vec) {
  vec = Tmat * vec;
}

void SketchSpace::applyTmatinv(fmat::Column<3> &vec) {
  vec = Tmatinv * vec;
}

void SketchSpace::setIdx(Sketch<skindex>& indices, int const x, int const y, int const indexval) {
  int const indexval_x = indexval % width;
  int const indexval_y = indexval / width;
  indices(x,y) = (indexval_x < 0 || indexval_y < 0 || indexval_x >= (int)width || indexval_y >= (int)height
		  || abs(indexval_x-x)+1 == (int)width) // loop-around check
    ? dummy : indexval;
}

std::string SketchSpace::getTmatForGUI() {
  std::ostringstream tmat_stream;
  tmat_stream << "tmat" << endl;
  for (int i=0; i<3; i++)
    for (int j=0; j<4; j++)
      tmat_stream << " " << Tmat(i,j);
  tmat_stream << endl;
  return tmat_stream.str();
}

std::string SketchSpace::getSketchListForGUI() {
	bumpRefreshCounter();
	std::string sketchlist;
	sketchlist += boolPool.getSketchListForGUI();
	sketchlist += ucharPool.getSketchListForGUI();
	sketchlist += usintPool.getSketchListForGUI();
	sketchlist += uintPool.getSketchListForGUI();
	sketchlist += floatPool.getSketchListForGUI();
	sketchlist += yuvPool.getSketchListForGUI();
	return sketchlist;	
}

SketchDataRoot* SketchSpace::retrieveSketch(int const id) {
  SketchDataRoot* sketchp;
  // Try each pool in turn until we find it.
  sketchp = boolPool.retrieveSketch(id);
  if ( sketchp ) return sketchp;
  sketchp = ucharPool.retrieveSketch(id);
  if ( sketchp ) return sketchp;
  sketchp = usintPool.retrieveSketch(id);
  if ( sketchp ) return sketchp;
  sketchp = uintPool.retrieveSketch(id);
  if ( sketchp ) return sketchp;
  sketchp = floatPool.retrieveSketch(id);
  if ( sketchp ) return sketchp;
  sketchp = yuvPool.retrieveSketch(id);
  return sketchp;
}

} // namespace
