//-*-c++-*-
#ifndef _LookForTapia3Markers_h_
#define _LookForTapia3Markers_h_

#include "Behaviors/Demos/Tapia/Tapia3MarkerData.h"
#include "DualCoding/DualCoding.h"
#include "Sound/SoundManager.h"

using namespace DualCoding;

class LookForTapia3Markers : public VisualRoutinesBehavior {
public:
  LookForTapia3Markers() : VisualRoutinesBehavior("LookForTapia3Markers") {}

  virtual void doStart() {
    MapBuilderRequest req(MapBuilderRequest::localMap);
    req.maxDist = 100000;
    req.addObjectColor(markerDataType, "orange");
    req.addObjectColor(markerDataType, "pink");
    req.addObjectColor(markerDataType, "blue");
    req.markerTypes.insert(Tapia3MarkerData::tapia3MarkerType);
    
    unsigned int mapreq_id = mapBuilder.executeRequest(req);
    erouter->addListener(this, EventBase::mapbuilderEGID, mapreq_id, EventBase::statusETID);
  }
  
  virtual void processEvent(const EventBase &event) {
    
    cout << "Found " << camShS.allShapes().size() << " cam shapes." << endl;
    SHAPEROOTVEC_ITERATE(camShS, s)
      cout << "   " << s << endl;
    END_ITERATE;
    
    // hide the blobs imported into local space so they don't mess up the SketchGUI display
    NEW_SHAPEVEC(localBlobs, BlobData, select_type<BlobData>(localShS));
    SHAPEVEC_ITERATE(localBlobs, BlobData, b)
      b->setViewable(false);
    END_ITERATE;

    NEW_SHAPEVEC(tmarkers, Tapia3MarkerData, select_type<Tapia3MarkerData>(localShS));
    cout << "Created " << tmarkers.size() << " Tapia3Markers in local space." << endl;
    SHAPEVEC_ITERATE(tmarkers, Tapia3MarkerData, s)
      cout << "   " << s << endl;
    END_ITERATE;

    if ( tmarkers.size() > 0 ) {
      Shape<Tapia3MarkerData> &firstmarker = tmarkers[0];
      string const topcol = ProjectInterface::getColorName(firstmarker->topColor);
      string const midcol = ProjectInterface::getColorName(firstmarker->middleColor);
      string const botcol = ProjectInterface::getColorName(firstmarker->bottomColor);
      cout << "Top marker color is " << topcol << endl
	   << "Middle marker color is " << midcol << endl
	   << "Bottom marker color is " << botcol << endl;
      sndman->speak("Saw a "+topcol+" "+midcol+" and "+botcol+" marker");
    }
  }

};

#endif
