#include "SketchSpace.h"
#include "ShapeSpace.h"
#include "ShapeTypes.h"
#include "ShapeRoot.h"
#include "BaseData.h"

using namespace std;

namespace DualCoding {

ShapeRoot::ShapeRoot(BaseData *p) : id(p->getId()), data(p) {
  ++data->refcount;
}

ShapeRoot::ShapeRoot(const ShapeRoot &other) 
  : id(other.id), data(other.data) {
  if ( data != NULL )
    ++data->refcount;
};

ShapeRoot& ShapeRoot::operator=(const ShapeRoot& src) {
  if ( data != NULL && --data->refcount == 0 && data != src.data )
    delete data;
  id = src.id;
  data = src.data;
  if ( data != NULL )
    ++data->refcount;
  return *this;
}

ShapeRoot::~ShapeRoot(void) { 
  if ( data != NULL && --data->refcount == 0 )
	 delete data;
}

void ShapeRoot::deleteShape(void) { 
  if ( isValid() )
	 data->space->deleteShape(*this); 
}

void ShapeRoot::sanity_check(void) const {
  if ( !isValid() ) {
    cerr << "ERROR: Reference to invalid shape at " << (const void*)this
			<< "  id=" << id << "  data=" << (const void*)data;
	 if ( data != NULL )
		cout << "  data->id=" << data->id;
	 cout << endl;
  }
}

ShapeSpace& ShapeRoot::getSpace() const {
  sanity_check(); 
  return *(data->space);
}

bool ShapeRoot::operator==(const ShapeRoot& other) const {
  if ( isValid() )
    if ( other.isValid() )
      return id == other.id;
    else
      return false;
  else
    return !other.isValid();
}      

std::ostream& operator<<(std::ostream &os, const ShapeRoot &r) {
  if ( r.isValid() ) {
    cout << r.data->getSpace().name << ":" << "Shape<" << r->getTypeName() << ">("
	 << r->getName() << ",id="
	 << r.id  /* << ", data=" << r.data */
	 << ")";
  }
  else {
	 cout << "Shape(id=" << r.getId() << ",data=" << reinterpret_cast<size_t>(r.data) << ")[invalid]";
  }
  return os;
}

} // namespace
