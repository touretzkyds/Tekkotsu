//-*-c++-*-

#include "ShapeTypes.h"

namespace DualCoding {

const char* data_name(int data_type)
{
  switch(data_type) {
  case lineDataType:
    return("LineData");
    break;
  case ellipseDataType:
    return("EllipseData");
    break;
  case pointDataType:
    return("PointData");
    break;
  case agentDataType:
    return("AgentData");
    break;
  case sphereDataType:
    return("SphereData");
    break;
  case polygonDataType:
    return("PolygonData");
    break;
  case blobDataType:
    return("BlobData");
    break;
  case brickDataType:
    return("BrickData");
    break;
  case pyramidDataType:
    return("PyramidData");
    break;
  case localizationParticleDataType:
    return("LocalizationParticleData");
    break;
  case targetDataType:
    return("TargetData");
    break;
  case markerDataType:
    return("MarkerData");
    break;
  case cylinderDataType:
	return("CylinderData");
    break;
  case siftDataType:
	return("SiftData");
    break;
  case aprilTagDataType:
	return("AprilTagData");
    break;
  case graphicsDataType:
	return("GraphicsData");
    break;
  case dominoDataType:
	return("DominoData");
    break;
  case naughtDataType:
	return("NaughtData");
    break;
  case crossDataType:
	return("CrossData");
    break;
  case skeletonDataType:
	return("SkeletonData");
    break;
  case unknownDataType:
  default:
    return("*Unknown*");
    break;
  }
}

} // namespace
