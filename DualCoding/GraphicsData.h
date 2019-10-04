//-*-c++-*-
#ifndef _GRAPHICSDATA_H_
#define _GRAPHICSDATA_H_

#include <vector>

#include "BaseData.h"
#include "Shared/Measures.h" // AngPi
#include "Localization/ShapeBasedParticleFilter.h"
#include "ShapeLocalizationParticle.h"

namespace DualCoding {

  template<typename T> class Sketch;

  //! Graphical display of a collection of lines, ellipses, particles, and text objects
  class GraphicsData : public BaseData {
  public:
    typedef std::pair<float,float> xyPair;
    typedef fmat::Column<3> xyzPair;

    enum GraphicsElementType {
      lineGType,
      polygonGType,
      ellipseGType,
      textGType,
      locParticleGType,
      axisAngleGType,
      pointGType,
      boundingGType
    };

    class GraphicsElement {
    private:
      GraphicsElementType type;
      std::string name;
      rgb color;
    public:
      GraphicsElement(GraphicsElementType _type, std::string _name, rgb _color) :
	type(_type), name(_name), color(_color) {}
      virtual ~GraphicsElement() {};
      GraphicsElementType getType() const { return type; }
      const std::string& getName() const { return name; }
      void setName(const std::string &_name) { name=_name; }
      const rgb& getColor() const { return color; }
      void setColor(const rgb c) { color = c; }
      void setColor(const std::string &colorname);
    };

    class LineElement : public GraphicsElement {
    public:
      xyPair pt1, pt2;
      LineElement(const std::string &_name, const xyPair &_pt1, const xyPair &_pt2, const rgb _color=rgb(0,0,255))
	: GraphicsElement(lineGType, _name, _color), pt1(_pt1), pt2(_pt2) {}
      virtual ~LineElement() {}
    };
  
    class PolygonElement : public GraphicsElement {
    public:
      std::vector<xyPair> vertices;
      bool closed;
    
      PolygonElement(const std::string &_name, const std::vector<xyPair> &pts, bool _closed, rgb _color=rgb(255,0,0))
	: GraphicsElement(polygonGType, _name, _color), vertices(pts.size()>1 ? pts : std::vector<xyPair>()), closed(_closed) {}
      virtual ~PolygonElement() {}
    };

    class BoundingBoxElement : public GraphicsElement {
    public:
      fmat::Quaternion q;
      xyzPair centroid;
      float l, w, h;
      std::vector<std::vector<fmat::Column<3> > > vertices;
      BoundingBoxElement(const std::string &_name, const fmat::Quaternion &_q, 
			 const xyzPair & _centroid, float _l, float _w, float _h, rgb _color=rgb(255,0,0)) :
	GraphicsElement(boundingGType, _name, _color), q(_q), centroid(_centroid),
	l(_l), w(_w), h(_h), vertices(std::vector<std::vector<fmat::Column<3> > >()){}
      BoundingBoxElement(const std::string &_name, const std::vector<std::vector<fmat::Column<3> > > &_vertices,
			 rgb _color = rgb(255,0,0)) : 
	GraphicsElement(boundingGType, _name, _color), q(fmat::Quaternion()), 
	centroid(xyzPair()), l(0), w(0), h(0), vertices(_vertices) {}
      virtual ~BoundingBoxElement() {}
    };

    class EllipseElement : public GraphicsElement {
    public:
      xyPair center;
      float semimajor, semiminor;
      AngPi orientation;
      bool filled;
      EllipseElement(const std::string &_name, const xyPair &_center, float _semimajor, float _semiminor,
		     AngTwoPi _orientation, bool _filled, const rgb _color=rgb(255,0,0)) :
	GraphicsElement(ellipseGType, _name, _color),
	center(_center), semimajor(_semimajor), semiminor(_semiminor),
	orientation(_orientation), filled(_filled) {}
      virtual ~EllipseElement() {}
    };
  
    class TextElement : public GraphicsElement {
    public:
      xyPair startpt;
      std::string msg;
    
      TextElement(const std::string &_name, const xyPair &_startpt, std::string const &_msg, const rgb _color=rgb(0,0,0)) :
	GraphicsElement(textGType, _name, _color), startpt(_startpt), msg(_msg) {}
      virtual ~TextElement() {}
    };

    class LocalizationParticleElement : public GraphicsElement {
    public:
      const ShapeBasedParticleFilter::particle_collection *particles;
      ShapeBasedParticleFilter::index_t index;
    
      LocalizationParticleElement(const std::string &_name, 
				  const ParticleFilter<LocalizationParticle>::particle_collection &_particles,
				  ParticleFilter<LocalizationParticle>::index_t _index, const rgb _color=rgb(0,0,0)) :
	GraphicsElement(locParticleGType, _name, _color), particles(&_particles), index(_index) {}
    
      LocalizationParticleElement(const LocalizationParticleElement &other) :
	GraphicsElement(other), particles(other.particles), index(other.index) {}
    
      virtual Point getCentroid() const { return Point((*particles)[index].x, (*particles)[index].y);}

      AngTwoPi getOrientation() const { return (*particles)[index].theta; }
      float getWeight() const { return (*particles)[index].weight; }
            
      LocalizationParticleElement& operator=(const LocalizationParticleElement &other) {
	if ( this != &other ) {
	  particles = other.particles;
	  index = other.index;
	}
	return *this;
      }
    
      virtual ~LocalizationParticleElement() {}
    };

    class AxisAngleElement : public GraphicsElement {
    public:
      fmat::Quaternion q;
      xyzPair centroid;
      AxisAngleElement(const std::string &_name, const fmat::Quaternion &_q, 
		       const xyzPair &_centroid, const rgb _color = rgb(0,0,0)) :
	GraphicsElement(axisAngleGType, _name, _color), q(_q), centroid(_centroid) {}
      virtual ~AxisAngleElement() {}
    };

    class PointElement : public GraphicsElement {
    public:
      xyPair center;
      PointElement(const std::string &_name, const xyPair &_center, const rgb _color=rgb(255,0,0)) :
	GraphicsElement(pointGType, _name, _color), center(_center) {}
      virtual ~PointElement() {}
    };

    class CircleElement : public EllipseElement {
    public:
      CircleElement(const std::string &_name, const xyPair &_center, float _radius, bool _filled, const rgb _color=rgb(255,0,0)) : 
	EllipseElement(_name, _center, _radius, _radius, 0, _filled, _color) {}
      virtual ~CircleElement() {}
    };

    std::vector<const GraphicsElement*> elements;

    GraphicsData(ShapeSpace &_space);

    static ShapeType_t getStaticType() { return graphicsDataType; }

    DATASTUFF_H(GraphicsData);

    virtual ~GraphicsData();

    void add(const GraphicsElement *element);

    virtual bool isMatchFor(const ShapeRoot& other) const { return false; }
    virtual bool updateParams(const ShapeRoot& other, bool force=false) { return false; }
    Point getCentroid() const { return Point(); }
    virtual void printParams() const;
    void applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref=unspecified) {}
    virtual void projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {}

    virtual unsigned short getDimension() const { return 2; }

  private:
    virtual Sketch<bool>* render() const;

  };

} // namespace

#endif

