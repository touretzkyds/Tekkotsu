// INCLUDES
// c++
#include <iostream>
#include <vector>

// tekkodu
#include "Kodu/KoduWorld.h"
#include "Kodu/PerceptualTasks/PerceptualTaskBase.h"
#include "Kodu/PerceptualTasks/VisualLocalizationTask.h"
#include "Kodu/Primitives/PerceptionSearch.h"

// tekkotsu
#include "Crew/MapBuilderRequest.h"
#include "DualCoding/PolygonData.h"
#include "DualCoding/VRmixin.h"

namespace Kodu {

    unsigned int VisualLocalizationTask::idCount = 50000;
    const int VisualLocalizationTask::kMinStarsRequiredToLocalize = 2;

    bool VisualLocalizationTask::canExecute(const KoduWorld& kWorldState) {
        return (!kWorldState.thisAgent.isExecutingMotionAction() && (localizationPoints.size() >= 2));
    }
    
    const DualCoding::PilotRequest VisualLocalizationTask::getPilotRequest() {
        std::cout << "[Visual Localization Task]\n";
        // construct the mapbuilder request
				PilotRequest pilotreq;
        DualCoding::MapBuilderRequest* mreq = new DualCoding::MapBuilderRequest(DualCoding::MapBuilderRequest::localMap);
        mreq->removePts = false;
        
        if (!localizationPoints.empty()) {
            // the vector that will contain the position of the stars the robot should look at
            std::vector<DualCoding::Point> starLocs;
            starLocs.reserve(localizationPoints.size());
            std::cout << "creating the localization point vector for " << localizationPoints.size()
                << " points.\n";

            // a count of the number of stars added to the vector
            int numbOfStarsInVector = 0;
            // iterate over all the stars in the map
            for (std::map<int, DualCoding::Point>::iterator it = localizationPoints.begin();
                it != localizationPoints.end(); ++it)
            {
                AngSignPi dtheta = bearingFromAgentToPoint(it->second);
                dtheta += AngSignPi(M_PI / 2.0f);
                if (dtheta > 0.0f) {
                    std::cout << "adding tag #" << it->first << " @ " << it->second << " to vector.\n";
                    NEW_SHAPE(star, DualCoding::AprilTagData,
                        new DualCoding::AprilTagData(DualCoding::VRmixin::worldShS,
                            AprilTags::TagDetection(it->first), DualCoding::Point(it->second)));
                    DualCoding::Point starCen(star->getCentroid());
                    //******** to fix... make the robot only look a little back when stars are to the side
                    starLocs.push_back(starCen);
                    starLocs.push_back(DualCoding::Point(starCen.coordX() - 200.0f, starCen.coordY(),
                        starCen.coordZ(), starCen.getRefFrameType()));
                    //**********
                    // if there is enough stars in the vector, exit
                    if ((++numbOfStarsInVector) == maxStarsRequested) break;
                }
            }
            if (numbOfStarsInVector < maxStarsRequested) {
                std::cout << "VisualLocalizationTask: WARNING---you requested " << maxStarsRequested
                    << " stars, but there were only " << numbOfStarsInVector << " 'visible'.\n";
            }

            std::cout << "generating localization polygon\n";
            NEW_SHAPE(localizePolygon, DualCoding::PolygonData,
                new DualCoding::PolygonData(DualCoding::VRmixin::localShS, starLocs, false));
            localizePolygon->setObstacle(false);
            localizePolygon->setViewable(true);

            std::cout << "creating mapbuilder request\n";
            mreq->setAprilTagFamily();
            mreq->searchArea = localizePolygon;
        }
        // this should never happen because if the localization points map has less than 2 stars, then
        // the request will never execute.
        else {
            std::cout << "THIS SHOULD HAVE NEVER EXECUTED!!! WHAT'S HAPPENING!!!\n";
            taskStatus = TS_FAILURE;
            return pilotreq;
        }

        // create the pilot request
        std::cout << "creating pilot request\n";
        pilotreq = DualCoding::PilotRequest(DualCoding::PilotTypes::localize);
        pilotreq.landmarkExtractor = mreq;
        
        // return the pilot request
        taskStatus = TS_COMPLETE;
        std::cout << "done!\n";
        return pilotreq;
    }
}
