//-*-c++-*-
#include "ShapeLine.h"
#include "ShapeRoot.h"
#include "ShapeSpace.h"
#include "Sketch.h"
#include "SketchSpace.h"
#include "visops.h"

#include "PolygonData.h"
#include "ShapePolygon.h"

using namespace std;

namespace DualCoding {

DATASTUFF_CC(PolygonData);

PolygonData::PolygonData(const LineData& side) 
  : BaseData(*side.space,polygonDataType), edges(), vertices()
{ 
  edges.push_back(side);
	parentId = side.getId();
  updateVertices();
  mobile = POLYGON_DATA_MOBILE;
}

PolygonData::PolygonData(ShapeSpace& _space, const vector<Point>& pts, 
												 bool closed, bool end1Valid, bool end2Valid)
  : BaseData(_space, polygonDataType), edges(), vertices(pts)
{
  mobile = POLYGON_DATA_MOBILE;
  if ( closed && vertices.size() > 1)
    vertices.push_back(vertices.front());
	for ( size_t i = 0; i < pts.size(); i++ )
		if ( std::isnan(pts[i].coordX()) || std::isnan(pts[i].coordY()) )
			cout << "Warning from PolygonData constructor: bad vertex " << pts[i] << endl;
  for(vector<Point>::const_iterator vtx_it = vertices.begin();
      vtx_it < vertices.end()-1; vtx_it++)
    edges.push_back(LineData(_space, *vtx_it, *(vtx_it+1)));
	if ( ! edges.empty() ) {
		edges.front().end1Pt().setValid(end1Valid);
		edges.back().end2Pt().setValid(end2Valid);
	}
}

vector<Shape<LineData> > PolygonData::extractPolygonEdges(Sketch<bool> const& sketch,
																													Sketch<bool> const& occluder) {
  vector<Shape<LineData> > lines(LineData::extractLines(sketch, occluder, 4));
  return lines;
}

void PolygonData::updateVertices() {
  vertices.clear();
  if (edges.empty()) return;

  if (isClosed())
    vertices.push_back(end1Ln().intersectionWithLine(end2Ln()));
  else
    vertices.push_back(end1Pt());
  if (edges.size() > 1)
    for (vector<LineData>::const_iterator it = edges.begin();
	 it != edges.end()-1; it++)
      vertices.push_back(it->intersectionWithLine(*(it+1)));
  if (isClosed())
    vertices.push_back(vertices.front());
  else
    vertices.push_back(end2Pt());
}

void PolygonData::setVertices(std::vector<Point> &pts, bool closed, bool end1Valid, bool end2Valid) {
  vertices = pts;
	edges.clear();
  for(vector<Point>::const_iterator vtx_it = pts.begin();
      vtx_it < pts.end()-1; vtx_it++)
    edges.push_back(LineData(*space, *vtx_it, *(vtx_it+1)));
	if ( ! edges.empty() ) {
		edges.front().end1Pt().setValid(end1Valid);
		edges.back().end2Pt().setValid(end2Valid);
	}
  if (closed) // close polygon
    tryClosePolygon();
}

vector<ShapeRoot> PolygonData::formPolygons(const vector<LineData>& lines) {
  if (lines.empty()) return vector<ShapeRoot>();
  // sort lines from longest to shortest
  vector<LineData> allEdges = lines;
  sort(allEdges.begin(), allEdges.end(), ptr_fun(isFirstLineLonger));
  
  vector<ShapeRoot> newPolygons;
  for (vector<LineData>::iterator line_it = allEdges.begin();
			 line_it != allEdges.end(); line_it++) {
    Shape<PolygonData> temp_polygon(*line_it);
    for (vector<LineData>::iterator line_it2 = line_it+1;
				 line_it2 < allEdges.end(); line_it2++) {
      if (line_it2->getColor() == line_it->getColor()
					&& (temp_polygon->tryUpdateEdge(*line_it2)
							|| temp_polygon->tryImportNewEndline(*line_it2))) {
				allEdges.erase(line_it2);
				line_it2 = line_it;
      }
    }
    temp_polygon->setColor(line_it->getColor());
    temp_polygon->updateVertices();
    //temp_polygon->printParams();
    newPolygons.push_back(temp_polygon);
  }
  return newPolygons;
}

vector<ShapeRoot> PolygonData::rebuildPolygons(const vector<LineData>& lines, 
																							 vector<Shape<PolygonData> >& existingPolygons, 
																							 vector<Shape<PolygonData> >& deletedPolygons) {
	// If no lines were found, the set of polygons cannot change, so punt.
  if (lines.empty()) return vector<ShapeRoot>();

  vector<LineData> allEdges = lines; // Will hold all lines plus all existing polygon edges

	// Add every polygon's edges to allEdges
  for (vector<Shape<PolygonData> >::const_iterator oldPoly = existingPolygons.begin();
       oldPoly < existingPolygons.end(); oldPoly++)
    allEdges.insert(allEdges.end(), (*oldPoly)->getEdges().begin(),(*oldPoly)->getEdges().end());

	// Make new polygons from all the lines plus the edges of existing polygons
	vector<ShapeRoot> newP = formPolygons(allEdges);
  vector<Shape<PolygonData> > &newPolygons = *reinterpret_cast<vector<Shape<PolygonData> >*>(&newP);

	// If any of the newly-formed polygons exactly matches an existing
	// polygon, delete the new polygon.  Otherwise, delete the old polygon
	// because we will have re-formed it with some modification.
  for (vector<Shape<PolygonData> >::iterator oldPoly = existingPolygons.begin();
       oldPoly < existingPolygons.end(); oldPoly++) {
    vector<Shape<PolygonData> >::iterator newPoly = newPolygons.begin();
    for (; newPoly < newPolygons.end(); newPoly++)
      if ( (*newPoly)->getColor() == (*oldPoly)->getColor() &&
					 (*newPoly)->equivalentVertices(**oldPoly) ) {
				cout << "=> new poly " << *newPoly << " was match for existing polygon " << *oldPoly << endl;
				newPoly->deleteShape();
				newPolygons.erase(newPoly--);
				break;
			}
    if (newPoly == newPolygons.end()) {
      deletedPolygons.push_back(*oldPoly);
      existingPolygons.erase(oldPoly--);
    }
  }
  return newP;
}

void PolygonData::importAdjust() {
	for (vector<LineData>::iterator it = edges.begin();
       it != edges.end(); it++) {
		it->space = space;
		it->parentId = id;
	}
}

BoundingBox2D PolygonData::getBoundingBox() const {
	BoundingBox2D result;
	for (vector<Point>::const_iterator it=vertices.begin(); it!=vertices.end(); ++it)
		result.expand(it->coords);
	return result;
}

bool PolygonData::formsNewEndline(const LineData& ln, bool useEnd1Pt, bool useEnd2Pt) const {
  if (edges.empty()) return true;

  useEnd1Pt &= ln.end1Pt().isValid(); // don't use invalid endpoint
  useEnd2Pt &= ln.end2Pt().isValid(); // don't use invalid endpoint

  if (!(useEnd1Pt || useEnd2Pt))
    return false;

  if (end1Pt().isValid()) //forms vertex with end1Ln?
    if((useEnd1Pt && end1Pt().distanceFrom(ln.end1Pt()) < THRESH_DIST_VERTEX)
       || (useEnd2Pt && end1Pt().distanceFrom(ln.end2Pt()) < THRESH_DIST_VERTEX))
      return true;
  if (end2Pt().isValid()) //forms vertex with end2Ln?
    if((useEnd1Pt && end2Pt().distanceFrom(ln.end1Pt()) < THRESH_DIST_VERTEX)
       || (useEnd2Pt && end2Pt().distanceFrom(ln.end2Pt()) < THRESH_DIST_VERTEX))
      return true;

  return false;
}

bool PolygonData::tryImportNewEndline(const LineData& ln, bool useEnd1Pt, bool useEnd2Pt) {
  if (edges.empty()) {
    edges.push_back(ln);
    return true;
  }
      
  useEnd1Pt &= ln.end1Pt().isValid(); // don't use invalid endpoint
  useEnd2Pt &= ln.end2Pt().isValid(); // don't use invalid endpoint
  if (!(useEnd1Pt || useEnd2Pt))
    return false;
  
  if (end1Pt().isValid()) {
    if (useEnd1Pt && end1Pt().distanceFrom(ln.end1Pt()) < THRESH_DIST_VERTEX) {
      LineData end1ln = ln;
      end1ln.setEndPts(ln.end2Pt(),ln.end1Pt());
      edges.insert(edges.begin(), end1ln);
      return true;
    }
    else if (useEnd2Pt && end1Pt().distanceFrom(ln.end2Pt()) < THRESH_DIST_VERTEX) {
      edges.insert(edges.begin(), ln);
      return true;
    }
  }
  
  if (end2Pt().isValid()) {
    if (useEnd1Pt && end2Pt().distanceFrom(ln.end1Pt()) < THRESH_DIST_VERTEX) {
      edges.push_back(ln);
      return true;
    }
    else if (useEnd2Pt && end2Pt().distanceFrom(ln.end2Pt()) < THRESH_DIST_VERTEX) {
      LineData end2ln = ln;
      end2ln.setEndPts(ln.end2Pt(),ln.end1Pt());
      edges.push_back(end2ln);
      return true;
    }
  }
  return false;
}

bool PolygonData::tryClosePolygon() {
  if (vertices.size()<3 || 
      !end1Pt().isValid() || !end2Pt().isValid())
    return false;
  if (end1Ln().isMatchFor(end2Ln()))
    edges.erase(edges.end());
  edges.push_back(LineData(*space,end2Pt(),end1Pt()));
  edges.front().end1Pt().setValid(true);
  updateVertices();
  return true;
}

bool PolygonData::isClosed() const {
  return (edges.size() > 2 &&
	  (end1Ln().isMatchFor(end2Ln()) || 
	   ((end1Pt().distanceFrom(end2Pt()) < THRESH_DIST_VERTEX)
	    && end1Pt().isValid() && end2Pt().isValid())));
}


bool PolygonData::isMatchForEdge(const LineData& other) const {
  for(vector<LineData>::const_iterator this_it = edges.begin();
      this_it != edges.end(); this_it++)
    if (this_it->isMatchFor(other))
      return true;
  return false;
}

// remove edges whose confidence < 0
// break polygon into pieces if inner edge removed
vector<ShapeRoot> PolygonData::updateState() {
  bool breakPolygon = false;
  for (vector<LineData>::iterator it = edges.begin();
       it != edges.end(); it++) {
    if (it->getConfidence() < 0) {
      if (! (it == edges.begin() || it == edges.end()-1)) //inner edge deleted
	breakPolygon = true;
      edges.erase(it--);
    }
  }
  if (breakPolygon && !edges.empty()) { //form new polygons from the remaining edges of this polygon
    vector<LineData> lines(edges);
    edges.clear(); 
    return formPolygons(lines);
  }
  else
    return vector<ShapeRoot>();
}

bool PolygonData::tryUpdateEdge(const LineData &ln) {
  for(vector<LineData>::iterator it = edges.begin();
      it != edges.end(); it++)
    if (it->isMatchFor(ln) && it->updateParams(ln)) {
      it->increaseConfidence(ln);
      if (it->getParentId() == 0 && ln.getParentId() != 0)
				it->setParentId(ln.getParentId());
      return true;
    }
  return false;
}

bool PolygonData::isMatchFor(const ShapeRoot& other) const {
	/*  Don't match lines for polygons
  if (other->isType(lineDataType) && isSameColorAs(other)) {
    const LineData& other_ln = ShapeRootTypeConst(other,LineData).getData();
    return (isMatchForEdge(other_ln));
  }
	*/
  if (other->isType(polygonDataType) && isSameColorAs(other)) {
    const PolygonData& other_polygon = ShapeRootTypeConst(other,PolygonData).getData();
    if (other_polygon.getEdges().size() == edges.size()) {
      vector<LineData>::const_iterator other_it = other_polygon.getEdges().begin();
      vector<LineData>::const_iterator this_it = getEdges().begin();
      for (; other_it != other_polygon.getEdges().end(); other_it++, this_it++)
	if (!this_it->isMatchFor(*other_it)) break;
      if (this_it == edges.end())
	return true;
      vector<LineData>::const_reverse_iterator other_rit = other_polygon.getEdges().rbegin();
      this_it = edges.begin();
      for (; other_rit != other_polygon.getEdges().rend(); other_it++, this_it++)
				if (!this_it->isMatchFor(*other_rit)) break;
      return (this_it == edges.end());
    }
  }
  return false;
}

bool PolygonData::equivalentVertices(const PolygonData &other) const {
	if ( edges.size() != other.edges.size() )
		return false;
	unsigned int j = 0;
	// Find a vertex j that matches our vertex 0.  Now make sure all
	// vertices match.  Note that this algorithm can get confused is a
	// polygon contains multiple identical vertices (self-crossing at a
	// vertex).
	for (; j < edges.size(); j++)
		if ( vertices[0] == other.vertices[j] )
			break;
	// Now make sure all vertices match.
	for (unsigned int i = 0; i < edges.size();) {
		if ( vertices[i++] != other.vertices[j++] )
			return false;
		if ( j == edges.size() )
			j = 0;
	}
	return true;
}

int PolygonData::getConfidence() const {
  switch (edges.size()) {
  case 0:
    return -1;
  case 1:
    return edges.front().getConfidence();
  default:
    vector<LineData>::const_iterator it = edges.begin();
    int conf = (it++)->getConfidence();
    for (; it != edges.end(); it++)
      if (conf > it->getConfidence())
	conf = it->getConfidence();
    return conf;
  };
}

bool PolygonData::updateParams(const ShapeRoot& other, bool) {
  if (other->isType(lineDataType) ) { // && tryUpdateEdge(other)) {
    updateVertices();
    return true;
  }
  return false;
}

bool PolygonData::isAdmissible() const {
  for (vector<LineData>::const_iterator it = edges.begin();
       it != edges.end(); it++) {
    if (it->isAdmissible())
      return true;
  }
  return false;
}

// define a line from pt to most distant vertex from this point.
// count number of intersections with edges (except the ones 
bool PolygonData::isInside(const Point& pt) const {
  if (!isClosed()) return false;
  float mostDist = -1;
  unsigned int mostDistIndex = -1U;
  const unsigned int numVertices = vertices.size()-1; // number of unique vertices (first and last are same) 
  for (unsigned int i = 0; i < numVertices; i++) {
    float dist = pt.distanceFrom(vertices[i]);
    if (dist > mostDist) {
      mostDist = dist;
      mostDistIndex = i;
    }
  }
  //  cout << "Most distant vertex: " << vertices[mostDistIndex] << endl;
  LineData ln(this->getSpace(),pt,(vertices[mostDistIndex]));
  unsigned int intersectionCount = 0;
  // this is a check if this point falls between the two edges surrounding the most distant vertex from this point
  // if not, odd number of intersection indicates the point is inside, so increment the count by 1
  {
    float theta0 = (mostDistIndex == 0) ? 
      ((vertices.front() == vertices.back()) ? 
       atan2(vertices[vertices.size()-2].coordY()-vertices[mostDistIndex].coordY(),
	     vertices[vertices.size()-2].coordX()-vertices[mostDistIndex].coordX()) :
       atan2(vertices[vertices.size()-1].coordY()-vertices[mostDistIndex].coordY(),
	     vertices[vertices.size()-1].coordX()-vertices[mostDistIndex].coordX())) :
      atan2(vertices[mostDistIndex-1].coordY()-vertices[mostDistIndex].coordY(),
	    vertices[mostDistIndex-1].coordX()-vertices[mostDistIndex].coordX());
    float theta1 = atan2(pt.coordY()-vertices[mostDistIndex].coordY(),pt.coordX()-vertices[mostDistIndex].coordX());
    float theta2 = (mostDistIndex == vertices.size()-1) ?
      ((vertices.front() == vertices.back()) ? 
       atan2(vertices[1].coordY()-vertices[mostDistIndex].coordY(),
	     vertices[1].coordX()-vertices[mostDistIndex].coordX()) :
       atan2(vertices[0].coordY()-vertices[mostDistIndex].coordY(),
	     vertices[0].coordX()-vertices[mostDistIndex].coordX())) :
      atan2(vertices[mostDistIndex+1].coordY()-vertices[mostDistIndex].coordY(),
	    vertices[mostDistIndex+1].coordX()-vertices[mostDistIndex].coordX());
    if ((theta2>theta0 && !((theta2-theta0<M_PI && theta2>theta1 && theta1>theta0) ||
			   (2*M_PI-theta2+theta0<M_PI && (theta1>theta2 || theta0>theta1)))) 
	|| (theta0>theta2 && !((theta0-theta2<M_PI && theta0>theta1 && theta1>theta2) ||
			       (2*M_PI-theta0+theta2<M_PI && (theta1>theta0 || theta2>theta1))))) {
      //      cout << "orientataion not b/w edges\n";
      intersectionCount++;
    }
  }
    //  cout << "  " << ln.end1Pt() << " <-> " << ln.end2Pt() << endl;
  for (unsigned int i = mostDistIndex+1; i < numVertices+mostDistIndex-1; i++) {
    LineData cleanEdge(*space,vertices[i%numVertices],vertices[(i+1)%numVertices]);
    //    cout << "  " << i << ": " << cleanEdge.end1Pt() << " <-> " << cleanEdge.end2Pt();
    if (ln.getOrientation() != cleanEdge.getOrientation()
	&& ln.intersectsLine(cleanEdge)) {
      //      cout << " true\n";
      intersectionCount++;
    }
  }
  return (intersectionCount%2) == 0;
}

bool PolygonData::intersectsLine(const LineData& line)
{
  for (unsigned int i = 0; i < vertices.size() - 1; i++) {
    LineData cleanEdge(*space, vertices[i], vertices[i+1]);
    if (line.getOrientation() != cleanEdge.getOrientation()
	&& line.intersectsLine(cleanEdge)) {
      return true;
    }
  }

  return false;
}


void PolygonData::printParams() const {
	cout << "Polygon" << "  color=" << getColor() <<  endl;
  cout << " " << edges.size() << " edge(s)\n";
  for (vector<LineData>::const_iterator it = edges.begin();
       it != edges.end(); it++)
    cout << "  " << it->end1Pt() << " -> " << it->end2Pt() << endl;

  cout << " " << vertices.size () << " vertice(s):\n";
  for (vector<Point>::const_iterator it = vertices.begin();
       it != vertices.end(); it++)
    cout << "  " << (*it) << endl;
}

void PolygonData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  for (vector<LineData>::iterator it = edges.begin();
       it != edges.end(); it++)
    it->applyTransform(Tmat,newref);
  for (vector<Point>::iterator it = vertices.begin();
       it != vertices.end(); it++)
    it->applyTransform(Tmat,newref);
}

void PolygonData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
  for (vector<LineData>::iterator it = edges.begin();
       it != edges.end(); it++)
    it->projectToGround(camToBase,groundplane);
  for (vector<Point>::iterator it = vertices.begin();
       it != vertices.end(); it++)
    it->projectToGround(camToBase,groundplane);
}

Point PolygonData::getCentroid() const {
  Point pointSum = Point(0, 0, 0, getRefFrameType());
  if ( edges.empty() )
    return pointSum;
  vector<Point>::const_iterator it = vertices.begin();
  size_t numVertices = vertices.size();
  if ( isClosed() ) {
    it++;
    numVertices--;
  }
  for (; it != vertices.end(); it++)
    pointSum += *it;
  return pointSum/numVertices;
}


void PolygonData::setColor(const rgb &new_color) {
  color_rgb = new_color;
  for (vector<LineData>::iterator it = edges.begin();
       it != edges.end(); it++)
    it->setColor(color_rgb);
  deleteRendering();
}

void PolygonData::setColor(const color_index cindex) {
  setColor(ProjectInterface::getColorRGB(cindex));
}

void PolygonData::setColor(const std::string &color_name) {
  setColor(ProjectInterface::getColorRGB(color_name));
}

// helper function to sort lines from longest to shortest
bool PolygonData::isFirstLineLonger(const LineData& ln1,const LineData& ln2) { 
  return ln1.getLength() > ln2.getLength();
}

Sketch<bool>* PolygonData::render() const {
  Sketch<bool> *canvas = new Sketch<bool>(space->getDualSpace(),"");
  (*canvas) = 0;
  const int width = space->getDualSpace().getWidth();
  const int height = space->getDualSpace().getHeight();
  float x1, y1, x2, y2;
  for (vector<LineData>::const_iterator it = edges.begin();
       it != edges.end(); it++) {
    it->setDrawCoords(x1, y1, x2, y2, width, height);
    it->drawline2d(*canvas, (int)x1, (int)y1, (int)x2, (int)y2);
  }
  return canvas;
}

//================ Convex Hull ================

class convexHullPoint {
public:
  int x, y;
  float angle;

  convexHullPoint() : x(0), y(0), angle(0) {}

  convexHullPoint(int _x, int _y, float _a) : x(_x), y(_y), angle(_a) {}

  static int crossProduct(const convexHullPoint &p1, const convexHullPoint &p2, const convexHullPoint &p3) {
    return (p2.x - p1.x)*(p3.y - p1.y) - (p3.x - p1.x)*(p2.y - p1.y);
  }

  class pointCompare : public binary_function<convexHullPoint, convexHullPoint, bool> {
  private:
    const convexHullPoint &pivot;
  public:
    pointCompare(const convexHullPoint &_pivot) : pivot(_pivot) {}
    bool operator() (const convexHullPoint &p1, const convexHullPoint &p2) {
      if ( p1.angle < p2.angle )
	return true;
      else if ( p1.angle > p2.angle )
	return false;
      else {
	int const d1x = pivot.x - p1.x;
	int const d1y = pivot.y - p1.y;
	int const d2x = pivot.x - p2.x;
	int const d2y = pivot.y - p2.y;
	return (d1x*d1x+d1y*d1y) < (d2x*d2x+d2y*d2y);
      }
    }
  }
;
};

Shape<PolygonData> PolygonData::convexHull(const Sketch<bool> &sketch) {
  int const spts = sketch->sum();
  int const width = sketch.width;
  int const npix = sketch->getNumPixels();
  
  //  cout << "septs,width,npix= " << spts << ", " << width << ", " << npix << endl;

  // construct a vector of points and find the pivot point
  NEW_SKETCH_N(neighbs, uchar, visops::neighborSum(sketch));
  std::vector<convexHullPoint> points(spts);
  points.clear();
  int pivot_x = -1, pivot_y = width+1;
  for (int i=0; i<npix; i++) {
    //    cout << sketch[i] << " : " << neighbs[i] << endl;
    if ( sketch[i] && neighbs[i] < 8 ) {
      int const x = i % width;
      int const y = i / width;
      points.push_back(convexHullPoint(x,y,0));
      if ( y < pivot_y || (y == pivot_y && x > pivot_x) ) {
	pivot_x = x;
	pivot_y = y;
      };
    }
  }
  int const npts = points.size();


  // sort points by angle from the pivot, and if colinear, take closer point first
  for (int i=0; i<npts; i++)
    points[i].angle  = (float) -atan2((float) (pivot_y - points[i].y), (float) (pivot_x - points[i].x));
  std::sort(points.begin(),points.end(),convexHullPoint::pointCompare(convexHullPoint(pivot_x,pivot_y,0)));

  // push points onto a stack to form the convex hull
  vector<convexHullPoint> hull(npts);
  hull.clear();
  hull.push_back(points[0]);
  hull.push_back(points[1]);
  for ( int i=2; i<npts; i++ ) {
    int last = hull.size() - 1;
    int o = convexHullPoint::crossProduct(hull[last-1],hull[last],points[i]);
    if ( o == 0 )
      hull[last] = points[i];
    else if ( o < 0 )
      hull.push_back(points[i]);
    else {
      while ( o >= 0 && hull.size() > 2 ) {
	hull.pop_back();
	last--;
	o = convexHullPoint::crossProduct(hull[last-1],hull[last],points[i]);
      }
      hull.push_back(points[i]);
    }
  }
  int const nhull = hull.size();
  // build a polygon to represent the hull
  ShapeSpace &ShS = sketch->getDualSpace();
  vector<Point> vertices;
  const fmat::Transform &tmatinv = sketch->getSpace().getTmatinv();
  const ReferenceFrameType_t reftype = ShS.getRefFrameType();
  for (int i=0; i<nhull; i++) {
    fmat::Column<3> v = tmatinv * fmat::pack(hull[i].x, hull[i].y,  0);
    vertices.push_back(Point(v, reftype));
  }
  vertices.push_back(vertices.front());  // force closed polygon: don't trust tryClose()
  return Shape<PolygonData>(new PolygonData(ShS,vertices,true));
}


} // namespace
