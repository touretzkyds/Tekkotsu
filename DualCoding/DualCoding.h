//-*-c++-*-
// Master include file list to be used by user behaviors.

#ifndef LOADED_DualCoding_h_
#define LOADED_DualCoding_h_

//! Dual coding vision representations (Sketches and Shapes)
/*! For tutorial, see http://www.cs.cmu.edu/~dst/Tekkotsu/Tutorial/vr-intro.shtml */
namespace DualCoding {}

#include "DualCoding/ShapeTypes.h"
#include "DualCoding/SketchTypes.h"

#include "DualCoding/SketchSpace.h"
#include "DualCoding/ShapeSpace.h"
#include "DualCoding/ShapeFuns.h"

#include "DualCoding/Sketch.h"
#include "DualCoding/SketchIndices.h"
#include "DualCoding/Region.h"
#include "DualCoding/visops.h"

#include "DualCoding/ShapeRoot.h"
#include "DualCoding/ShapeLine.h"
#include "DualCoding/ShapeEllipse.h"
#include "DualCoding/ShapePoint.h"
#include "DualCoding/ShapeAgent.h"
#include "DualCoding/ShapeSphere.h"
#include "DualCoding/ShapePolygon.h"
#include "DualCoding/ShapeBlob.h"
#include "DualCoding/ShapeBrick.h"
#include "DualCoding/ShapePyramid.h"
#include "DualCoding/ShapeLocalizationParticle.h"
#include "DualCoding/ShapeMarker.h"
#include "DualCoding/ShapeCylinder.h"
#include "DualCoding/ShapeSift.h"
#include "DualCoding/ShapeAprilTag.h"
#include "DualCoding/ShapeGraphics.h"
#include "DualCoding/ShapeDomino.h"
#include "DualCoding/ShapeNaught.h"
#include "DualCoding/ShapeCross.h"
#include "DualCoding/ShapeSkeleton.h"

#include "DualCoding/BiColorMarkerData.h"   // no Shape because this is a subclass of MarkerData

#include "DualCoding/VisualRoutinesStateNode.h"
#include "DualCoding/VisualRoutinesBehavior.h" // this must precede mapbuilders, pilot and lookout

#include "Crew/Lookout.h"
#include "Crew/MapBuilder.h"

#endif
