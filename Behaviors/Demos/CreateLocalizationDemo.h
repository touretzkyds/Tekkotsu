//-*-c++-*-
#ifndef _LocalizationDemo_h_
#define _LocalizationDemo_h_

#include "DualCoding/DualCoding.h"
#include "Behaviors/Demos/Tapia/TapiaMarkerData.h"
#include "Localization/ShapeBasedParticleFilter.h"
#include "Localization/CreateMotionModel.h"

using namespace DualCoding;
using namespace std;

class CreateLocalizationDemo : public VisualRoutinesBehavior {
private:
  ShapeBasedParticleFilter filter;
  CreateMotionModel<LocalizationParticle> motion;

  CreateMotionModel<LocalizationParticle>::particle_collection theparticle;

public:
  CreateLocalizationDemo() : VisualRoutinesBehavior("CreateLocalizationDemo"),
			     filter(new CreateMotionModel<LocalizationParticle>(), camShS, worldShS),
			     motion(0.1, 0.1, 0, 0, 0, 0), theparticle()
  {
    theparticle.push_back(LocalizationParticle());
  }
  
  virtual void DoStart()
  {
    cout << "Starting localization demo." << endl;

    buildMap();

    cout << "Setting up particle filter..." << endl;

    filter.setPosition(0,0,0);

    filter.displayParticles(worldShS);

    erouter->addTimer(this, 0, 1000);
  }
  
  virtual void processEvent(const EventBase& event)
  {
    if (event.getGeneratorID() == EventBase::timerEGID) {
      // update the dead reckoning motion
      LocalizationParticle old = theparticle[0];
      motion.updateMotion(theparticle);
      if (motion.wasUpdated) {
	NEW_SHAPE(odomPath, LineData, new LineData(worldShS, Point(old.x, old.y), Point(theparticle[0].x, theparticle[0].y)));
      }

      if (motion.wasUpdated) {
	LocalizationParticle pre = filter.getBestParticle();
	
	// find markers in image
	camShS.clear();
	vector<Shape<MarkerData> > markers = MarkerData::extractMarkers(VRmixin::sketchFromSeg(), TapiaMarkerData::tapiaMarkerType);
	
	// do a single sensor update
	filter.update();
	filter.displayParticles(worldShS);

	LocalizationParticle post = filter.getBestParticle();
	NEW_SHAPE(bestPath, LineData, new LineData(worldShS, Point(pre.x, pre.y),
						   Point(post.x, post.y)));
	bestPath->setColor(rgb(100,100,255));
      }
    }
  }

  // adds the map shapes to shape space
  virtual void buildMap()
  {
    cout << "Building map..." << endl;
    
    
    NEW_SHAPE(marker1, TapiaMarkerData,
	      new TapiaMarkerData(worldShS, Point(1500,0,203.2),
				  ProjectInterface::getColorRGB("blue"),
				  ProjectInterface::getColorRGB("pink")));
    NEW_SHAPE(marker2, TapiaMarkerData,
	      new TapiaMarkerData(worldShS, Point(1500,-2000,203.2),
				  ProjectInterface::getColorRGB("orange"),
				  ProjectInterface::getColorRGB("blue")));
    NEW_SHAPE(marker3, TapiaMarkerData,
	      new TapiaMarkerData(worldShS, Point(0,-2000,203.2),
				  ProjectInterface::getColorRGB("pink"),
				  ProjectInterface::getColorRGB("orange")));
  }
};

#endif
