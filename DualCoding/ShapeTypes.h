//-*-c++-*-
#ifndef _ShapeTypes_H_
#define _ShapeTypes_H_

namespace DualCoding {

enum ReferenceFrameType_t {
  unspecified,
  camcentric,
  egocentric,
  allocentric
};

//! NOTE: If any of these type numbers are changed, the corresponding type
//! number must be changed in SketchGUI.java and possibly elsewhere!
enum ShapeType_t {
  unknownDataType = 0,
  lineDataType,
  ellipseDataType,
  pointDataType,
  agentDataType,
  sphereDataType,
  polygonDataType,
  blobDataType,
  brickDataType,
  pyramidDataType,
  localizationParticleDataType,
  targetDataType,
  markerDataType,
  cylinderDataType,
  siftDataType,
  aprilTagDataType,
  graphicsDataType,
  dominoDataType,
  naughtDataType,
  crossDataType,
  skeletonDataType,	
  // this one must always come last
  numDataTypes
};

const char* data_name(int data_type);

} // namespace

#endif
