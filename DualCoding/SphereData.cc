#include <iostream>
#include <vector>
#include <list>
#include <math.h>

#include "Motion/Kinematics.h"  // for kine

#include "SketchSpace.h"
#include "Sketch.h"
#include "ShapeRoot.h"
#include "LineData.h"
#include "Region.h"
#include "visops.h"

#include "SphereData.h"
#include "ShapeSphere.h"

using namespace std;

namespace DualCoding {

SphereData::SphereData(ShapeSpace& _space, const Point &c, float r) 
  : BaseData(_space,sphereDataType),
    centroid(c), radius(r)
  { mobile = SPHERE_DATA_MOBILE; }
  
SphereData::SphereData(const SphereData& otherData)
  : BaseData(otherData),centroid(otherData.centroid),radius(otherData.radius) 
  { mobile = otherData.mobile; }
  
DATASTUFF_CC(SphereData);

bool SphereData::isMatchFor(const ShapeRoot& other) const {
  if (!(isSameTypeAs(other) && isSameColorAs(other)))
    return false;
  const Shape<SphereData>& other_sphere = ShapeRootTypeConst(other,SphereData);
  float dist = centroid.distanceFrom(other_sphere->centerPt());
  return dist < 2*max(radius,other_sphere->radius); // *** DST hack
}

void SphereData::mergeWith(const ShapeRoot& other) {
  const Shape<SphereData>& other_sphere = ShapeRootTypeConst(other,SphereData);
  if (other_sphere->confidence <= 0)
    return;
  const int other_conf = other_sphere->confidence;
  confidence += other_conf;
  centroid = (centroid*confidence + other_sphere->centerPt()*other_conf) / (confidence+other_conf);
  radius = (radius*confidence + other_sphere->getRadius()*other_conf) / (confidence+other_conf);
}

bool SphereData::updateParams(const ShapeRoot& other, bool) {
  const Shape<SphereData>& other_sphere = *static_cast<const Shape<SphereData>*>(&other);
  centroid = (centroid*(confidence-1) + other_sphere->getCentroid())/confidence;
  radius = (radius*(confidence-1) + other_sphere->getRadius())/confidence;
  return true;
}

//! Print information about this shape. (Virtual in BaseData.)
void SphereData::printParams() const {
  cout << "Type = " << getTypeName();
  cout << "Shape ID = " << getId() << endl;
  cout << "Parent ID = " << getParentId() << endl;
  
  // Print critical points.
  cout << endl;
  cout << "center{" << centerPt().coordX() << ", " << centerPt().coordY() << "}" << endl;
  
  cout << "radius = " << getRadius() << endl;
  printf("color = %d %d %d\n",getColor().red,getColor().green,getColor().blue);
  cout << "mobile = " << getMobile() << endl;
  cout << "viewable = " << isViewable() << endl;
}


//! Transformations. (Virtual in BaseData.)
void SphereData::applyTransform(const fmat::Transform& Tmat, const ReferenceFrameType_t newref) {
  centroid.applyTransform(Tmat,newref);
}

bool SphereData::isInside(const Point& pt) const {
  float dist = pt.distanceFrom(centerPt());
  return radius>dist;
}


void SphereData::projectToGround(const fmat::Transform& camToBase, const PlaneEquation& groundplane) {
#ifdef TGT_HAS_CAMERA
  fmat::Column<3> cam_pos = kine->linkToBase(CameraFrameOffset).translation();
#else
  // shouldn't we do it this way regardless of TGT_HAS_CAMERA?
  fmat::Column<3> cam_pos = camToBase.translation();
#endif
  cout << "cam position " << cam_pos << endl;
  Point tangent_pt(centroid.coordX(),centroid.coordY()+radius, centroid.coordZ()); // pick a tangent point from cam point.
  Point cam_pt(cam_pos); // position of camera w.r.t. base
  cout << "sphere in cam frame: centroid:" << "(" << centroid.coordX() 
       << "," << centroid.coordY() << "," << centroid.coordZ() << ");  tangent_pt:" 
       << "(" << tangent_pt.coordX() << "," << tangent_pt.coordY() << "," << tangent_pt.coordZ()
       << ")" << endl;

  centroid.projectToGround(camToBase,groundplane);
  tangent_pt.projectToGround(camToBase,groundplane);
  cout << "sphere projected to ground: centroid:" << "(" << centroid.coordX() 
       << "," << centroid.coordY() << "," << centroid.coordZ() << ");  tangent_pt:" 
       << "(" << tangent_pt.coordX() << "," << tangent_pt.coordY() << "," << tangent_pt.coordZ()
       << ")" << endl;

  LineData tangent_line(getSpace(), cam_pt, tangent_pt); // tangent line from camera to sphere
  LineData cam_center(getSpace(), cam_pt, centroid); // line from camera passing through center point of sphere

  // a line perpendicular to tangent_line should cross cam_center line at the center point of the sphere if it
  // crosses tangent_line at the tangent point. Distance b/w tangent point and center point is the radius of sphere
  // which should also equal the height of the sphere (coordZ = 1/groundplane(3) + radius)
  // line from tangent_pt to centroid: z = ax + b (a known, b unkown)
  // line from camera to centroid: z = cx + d (c,d known)
  // tangent_line: z = ex + f (e,f known)
  // tangent_pt: x = (f-b)/(a-e)
  // centroid: x = (d-b)/(a-c), z = d + c(d-b)/(a-c) = 1/groundplane(3) + radius = (radius above groud level)
  // solve for b and substitute it to get centroid and radius

  vector<float> t_abc_xz = tangent_line.lineEquation_abc_xz();
  vector<float> cc_abc_xz = cam_center.lineEquation_abc_xz();
  vector<float> cc_abc_xy = cam_center.lineEquation_abc();

  const float f = cc_abc_xz[2] / cc_abc_xz[1];
  const float e = - cc_abc_xz[0] / cc_abc_xz[1];
  const float d = t_abc_xz[2] / t_abc_xz[1];
  const float c = - t_abc_xz[0] / t_abc_xz[1];
  const float a = -1.f / e; // perpendicular to e
  const float ground = 1.f/groundplane.getDisplacement();  //*** this assumes groundplane direction is [0,0,-1]
  const float DXtoR = 1.f / std::cos(std::atan(a)) / std::cos(std::atan(-cc_abc_xy[0]/cc_abc_xy[1])); // radius = dx * DXtoR where dx is b/w center pt and tangent pt
  const float b = (-DXtoR*f*a+DXtoR*f*c+DXtoR*d*a-DXtoR*d*e+ground*a*a-ground*a*c-ground*e*a+ground*e*c-d*a*a+d*e*a)/(-a*c+e*c+DXtoR*c-DXtoR*e);

  cout << "ground level: " << ground << ", DXtoR: " << DXtoR << endl;
  cout << "tangent line: z = " << e << " * x + " << f << endl;
  cout << "perpendicular line: z = " << a << " * x + " << b << endl;
  cout << "center line: z = " << c << " * x + " << d << endl;
  cout << "dx b/w tangent pt and center pt: " << ((f-b)/(a-e)-(d-b)/(a-c)) << endl;

  const float x = (d-b)/(a-c);
  const float z = d + c*(d-b)/(a-c);
  const float y = (cc_abc_xy[2]-cc_abc_xy[0]*x) / cc_abc_xy[1];

  centroid.setCoords(x,y,z);
  centroid.setRefFrameType(egocentric);
  radius = z-ground;

  cout << " => (" << x << "," << y << "," << z << ");  radius: " << radius << endl;
}

void SphereData::setRadius(float _radius) {
  radius = _radius;
  deleteRendering();
}
//}


// ==================================================
// BEGIN SKETCH MANIPULATION AND LINE EXTRACTION CODE
// ==================================================


//! Extraction.
//{
std::vector<Shape<SphereData> > SphereData::extractSpheres(const Sketch<bool>& sketch)
{
  const float AREA_TOLERANCE = 0.5f;
  const int REGION_THRESH = 25;
  NEW_SKETCH_N(labels,uint,visops::oldlabelcc(sketch,visops::EightWayConnect));
  list<Region> regionlist = Region::extractRegions(labels,REGION_THRESH);
  std::vector<Shape<SphereData> > spheres;
  
  if(regionlist.empty())
    return spheres;
  
  typedef list<Region>::iterator R_IT;
  for (R_IT it = regionlist.begin(); it != regionlist.end(); ++it) {
    float ratio = it->findSemiMajorAxisLength()/(float)(it->findSemiMinorAxisLength());
    if((ratio < 2.0) && (ratio > 1.0/(float)2.0)
       && (it->findArea() > M_PI*2.0*(it->findSemiMajorAxisLength())
	   *2.0*(it->findSemiMinorAxisLength())*AREA_TOLERANCE/4.0)) {
      Shape<SphereData> temp_sphere(*it);
      temp_sphere->inheritFrom(*sketch.operator->());
      //   temp_sphere->setParentId(sketch->getViewableId());
      // temp_sphere->setColor(sketch->getColor());
      spheres.push_back(Shape<SphereData>(temp_sphere));
    };
  }
  return spheres;
}

std::vector<Shape<SphereData> > SphereData::get_spheres(const Sketch<CMVision::uchar>& cam) {
  //! Declare all colors as valid.
  std::vector<bool> Valid_Colors;
  Valid_Colors.resize(ProjectInterface::getNumColors(),true);
  return(get_spheres(cam,Valid_Colors));
}

std::vector<Shape<SphereData> > SphereData::get_spheres(const Sketch<CMVision::uchar>& cam,
						 std::vector<bool>& Valid_Colors) {
  std::vector<Shape<SphereData> > spheres_vec;
  uchar cur_color;
  uchar num_colors = (uchar)Valid_Colors.size();
  char *pmask_name_chr = (char *)malloc(128*sizeof(char));
  
  // Loop through all valid colors.
  for(cur_color = 0; cur_color < num_colors; cur_color++) {
    
    if(Valid_Colors[cur_color] == true) {
      
      // Segment color pixels.
      NEW_SKETCH_N(pmask, bool, visops::colormask(cam,cur_color));
      sprintf(pmask_name_chr, "pmask_%d",cur_color);
      pmask->setName(pmask_name_chr);
      
      // Extract spheres.
      std::vector<Shape<SphereData> > spheresList = SphereData::extractSpheres(pmask);
      
      int num_spheres = (int)spheresList.size();
      int cur_sphere;
      
      for(cur_sphere = 0; cur_sphere < num_spheres; cur_sphere++) {
	//	spheresList[cur_sphere]->setColor(ProjectInterface::getColorRGB(cur_color));
	spheres_vec.push_back(spheresList[cur_sphere]); 
      }
      
    };
  }
  return(spheres_vec);
}


//! Render into a sketch space and return reference. (Private.)
Sketch<bool>* SphereData::render() const {
  const int cx = int(centerPt().coordX());
  const int cy = int(centerPt().coordY());
  /*  
  // Sure the sphere rendering is terribly inefficient, but it works
  float a = getRadius();
  float x_skip = atan(1/(0.5*a)); // minimum x-diff w/o gaps 
  for( float x = (cx-a); x<(cx+a); x+=x_skip) {
    float y_y0_sq = 1 - (x-cx)*(x-cx);
    if(y_y0_sq > 0) {
      int y_bot = cy + (int)(sqrt(y_y0_sq));
      int y_top = cy - (int)(sqrt(y_y0_sq));
      draw_result((int)x,y_bot) = true;
      draw_result((int)x,y_top) = true;
    }
  }
  draw_result(cx-(int)a,cy) = true; // fill in "holes" at ends
  draw_result(cx+(int)a,cy) = true;
  */  
  // Fill the sphere.

  Sketch<bool> result(space->getDualSpace(), "render("+getName()+")");
  result = 0;
  const int rad =(int) floor(getRadius()+0.5);
  const int radSq = rad*rad + rad/10; // rad/10 added to make sphere look nicer
  const int minX = (rad > cx) ? 0 : cx-rad;
  const int maxX = ((unsigned int) (rad+cx) > getSpace().getDualSpace().getWidth()-1)
    ? getSpace().getDualSpace().getWidth()-1 : (unsigned int)(cx+rad);
  for (int x = minX; x <= maxX; x++) {
    const int yRange = (int) sqrt((float) (radSq-(cx-x)*(cx-x))); 
    const int minY = (yRange > cy) ? 0 : cy-yRange;
    const int maxY = ((unsigned int) yRange+cy > getSpace().getDualSpace().getHeight()-1)
      ? getSpace().getDualSpace().getHeight()-1 : (unsigned int)(cy+yRange);
    for (int y = minY; y <= maxY; y++)
      result(x,y) = true;
  }
  return new Sketch<bool>(result);
}


} // namespace
