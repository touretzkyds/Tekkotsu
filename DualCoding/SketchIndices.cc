#include "SketchIndices.h"
#include "Sketch.h"
#include "SketchSpace.h"
#include "visops.h"
#include <algorithm>
#include <iostream>
#include <iterator>

namespace DualCoding {

const SketchIndices SketchIndices::operator[] (const Sketch<uint>& indirection) const {
  SketchIndices redir;
  for (CI it = table.begin(); it != table.end(); ++it)
    redir.table.insert(indirection[*it]);
  return redir;	
}

const SketchIndices 
SketchIndices::operator+ (const SketchIndices& other) const {
  SketchIndices result(*this);
  for (CI it = other.table.begin(); it != other.table.end(); ++it)
    result.table.insert(*it);
  return result;
}

const SketchIndices SketchIndices::operator- (const SketchIndices& other) const {
  SketchIndices result(*this);
  for (CI o = other.table.begin(); o != other.table.end(); ++o)
    result.table.erase(*o);
  return result;
}

std::ostream& operator<< (std::ostream& out, const SketchIndices &s) {
  typedef std::ostream_iterator<SketchIndices::SketchIndicesTable::value_type, char,
    std::char_traits<char> > ositer;
  std::copy(s.table.begin(), s.table.end(), ositer(std::cout," "));
  return out;
}

void SketchIndices::addIndices(const Sketch<bool>& im)
{
  size_t length = im->getNumPixels();
  for (size_t i = 0; i < length; i++)
    if ( im[i] )
      table.insert(i);
}

void SketchIndices::trimBounds(const SketchSpace &space) {
  SketchIndices result;
  for (SketchIndices::CI it = table.begin(); it != table.end(); ++it)
    if ( *it < space.getNumPixels() )
      result.table.insert(*it);
  table = result.table;
}

Sketch<bool> SketchIndices::getRendering(SketchSpace &space) const {
  Sketch<bool> result = visops::zeros(space);
  SketchIndices::CI it;
  for (it = table.begin(); it != table.end(); it++)
    result[*it] = true;
  return result;
}


} // namespace
