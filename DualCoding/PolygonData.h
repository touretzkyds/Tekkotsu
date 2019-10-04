//-*-c++-*-
#ifndef _POLYGONDATA_H_
#define _POLYGONDATA_H_

#include <vector>
#include <iostream>
#include <string>

#include "BaseData.h"    // superclass
#include "LineData.h"    // LineData data member
#include "Shared/Measures.h"    // coordinate_t; AngPi data member
#include "Shared/fmatSpatial.h"

namespace DualCoding {

class EndPoint;
class ShapeRoot;
class ShapeSpace;
template<> class Shape<LineData>;

#define POLYGON_DATA_MOBILE false
#define THRESH_DIST_VERTEX 50

//vertices unique
//num vertices >= 1
//for wall bounds, vertices are in clockwise order
//order of vertices isn't ambiguous
//average of vertices is the centroid

class PolygonData : public BaseData {
protected:
  std::vector<LineData> edges; // edges that make up this polygon
private:
  std::vector<Point> vertices; // cache of vertices

public:
  static ShapeType_t getStaticType() { return polygonDataType; }

  DATASTUFF_H(PolygonData);
  friend class Shape<PolygonData>;

  //! Constructors
  PolygonData(const LineData&);
  PolygonData(ShapeSpace& space, const std::vector<Point>& pts, bool closed, 
	      bool end1Valid=true, bool end2Valid=true);
  PolygonData(const std::vector<LineData>& lns)
    : BaseData(lns.front()), edges(lns), vertices() { updateVertices(); }
  PolygonData(const PolygonData& other)
    : BaseData(other), edges(other.edges), vertices(other.vertices) {}
  
  static std::vector<Shape<LineData> > extractPolygonEdges(Sketch<bool> const& sketch, Sketch<bool> const& occluder); //!< extracts then-edges lines
  //! forms polygons from lines and existing polygons
  //! existing polygons may be updated or deleted for which case they are added to deleted vector
  static std::vector<ShapeRoot> rebuildPolygons(const std::vector<LineData>&, 
																						 std::vector<Shape<PolygonData> >& existing,
																						 std::vector<Shape<PolygonData> >& deleted); 
  static std::vector<ShapeRoot> formPolygons(const std::vector<LineData>&); //!< forms polygons from lines
  
  //! Edge/Vertex Access Functions
  //@{
  const LineData& end1Ln() const { return edges.front(); } //!< returns first edge of this polygon
  const LineData& end2Ln() const { return edges.back(); } //!< returns last edge of this polygon
  const EndPoint& end1Pt() const { return edges.front().end1Pt(); } //!< returns end1Pt of end1Ln
  const EndPoint& end2Pt() const { return edges.back().end2Pt(); } //!< returns end2Pt of end2Ln
  const std::vector<Point>& getVertices() const { return vertices; } //!< returns all vertices of this polygon
  const std::vector<LineData>& getEdges() const { return edges; } //!< returns all edges of this polygon
  std::vector<LineData>& getEdgesRW() { return edges; } //!< returns address of edge vector for modification

  void setVertices(std::vector<Point> &vs, bool closed, bool end1Valid=true, bool end2Valid=true);
  //@}
  
	void importAdjust();

  BoundingBox2D getBoundingBox() const;

  std::vector<ShapeRoot> updateState();

  bool isClosed() const ; //<! returns if this polygon is closed
  
  //! Convex hull using Graham's scan.
  static Shape<PolygonData> convexHull(const Sketch<bool> &sketch);

  PolygonData& operator=(const PolygonData& other) {
    if (&other == this)
      return *this;
    BaseData::operator=(other);
    edges = other.edges;
    vertices = other.vertices;
    return *this;
  }

private:
  bool tryClosePolygon();
  bool tryImportNewEndline(const LineData& line, bool useEnd1Pt=true, bool useEnd2Pt=true);
  bool tryUpdateEdge(const LineData &line);
  bool isMatchForEdge(const LineData& other) const;
  bool formsNewEndline(const LineData& ln, bool useEnd1Pt=true, bool useEnd2Pt=true) const;
  void updateVertices(); //!< called everytime polygon is changed. updates vertices by finding intersections of adjascent edges
  static bool isFirstLineLonger(const LineData& ln1,const LineData& ln2);

public:   
  //{@! functions virtual in BaseData
  virtual Point getCentroid() const;
  virtual bool isMatchFor(const ShapeRoot& other) const;
	virtual bool equivalentVertices(const PolygonData& other) const; //!< Return true if polys have the same vertices but possibly in a rotated order due to rebuilding
  virtual bool isAdmissible() const;
  virtual bool updateParams(const ShapeRoot& other, bool forceUpdate=false); //!< updates existing edges, but does not importing new edges
  virtual int getConfidence() const; //!< returns minimum confidence of all edges
  virtual bool isInside(const Point& pt) const;
  virtual bool intersectsLine(const LineData& line);
  virtual void printParams() const;
  virtual void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified);
  virtual void projectToGround(const fmat::Transform& camToBase,
															 const PlaneEquation& groundplane);
  virtual void setColor(const rgb &new_color);
  virtual void setColor(const unsigned int color_index);
  virtual void setColor(const std::string &color_name);
  virtual Sketch<bool>* render() const;
  virtual unsigned short getDimension() const { return 2; }
  //}
};

} // namespace

#endif
