// INCLUDES
// c++
#include <cmath>
#include <sstream>

// tekkodu
#include "Kodu/KoduWorld.h"
#include "Kodu/PerceptualTasks/PerceptualTaskBase.h"
#include "Kodu/PerceptualTasks/VisualNavErrMonTask.h"
#include "Kodu/Primitives/PerceptionSearch.h"

//******** temp fix until base object is done
#include "Kodu/Primitives/KoduConditionBump.h"

// tekkotsu
#include "DualCoding/VRmixin.h"

namespace Kodu {

    //unsigned int const VisualNavErrMonTask::kMaxErrorOccurences = 1;
    unsigned int VisualNavErrMonTask::idCount = 30000;

    bool VisualNavErrMonTask::canExecute(const KoduWorld& kWorldState) {
        return (kWorldState.thisAgent.bodyIsInMotion()
            && kWorldState.thisAgent.isExecutingMotionAction()
            && (distanceInBetweenAgentAndObject(targets[0]) > KoduConditionBump::kMaxDistanceAwayToSenseBump)
            && !IsBehindAgent()(targets[0]));
    }

    void VisualNavErrMonTask::examineTaskResults() {
        // get all the objects in the local shape space
        std::vector<DualCoding::ShapeRoot> lclShapes(DualCoding::VRmixin::localShS);
        // import the target shape into the local shape space
        DualCoding::ShapeRoot lclTarget = DualCoding::VRmixin::mapBuilder->importWorldToLocal(targets[0]);
        // iterate over all the shapes in the local shape space for a match
        size_t const kNumbOfShapes = lclShapes.size();
        std::cout << "VisualNavErrorMon check target...";
        for (size_t i = 0; i < kNumbOfShapes; i++) {
            if (lclShapes[i]->isMatchFor(lclTarget)) {
                std::cout << "found a match! xy-dist = ";
                float dist = lclShapes[i]->getCentroid().xyDistanceFrom(lclTarget->getCentroid());
                std::cout << dist << "mm\n";
                //errorCount = 0;
                return;
            }
        }

        // ***NOTE: execution of the remaining portion of this function means a match was not found
        // check if the robot turned; sometimes that can cause an error (no match)
        const float kTurningError = 3.0f * M_PI / 180.0f;
        float agentCurrentOrientation = DualCoding::VRmixin::theAgent->getOrientation();
        if (std::fabs(AngSignPi(agentLastOrientation - agentCurrentOrientation)) > kTurningError) {
            std::cout << "Turning may have caused a matching error... ignoring error\n";
            return;
        }

        // increment the error count since the robot did not correctly identify the object
        //errorCount++;
        //std::stringstream stream;
        //stream << "task #" << id << ": recording (error) strike #" << errorCount << ". ";
        //if (errorCount == kMaxErrorOccurences) {
        //    taskStatus = TS_FAILURE;
        //    stream << " Task failed!";
        //}
        //std::cout << stream.str() << std::endl;
        std::cout << "task #" << id << ": navigation task failed.\n";
        // taskStatus = TS_FAILURE;  // *** HACK TO MAKE DEMO WORK FOR NOW
    }

    const DualCoding::MapBuilderRequest& VisualNavErrMonTask::getMapBuilderRequest() {

        // save the agent's current orientation
        agentLastOrientation = DualCoding::VRmixin::theAgent->getOrientation();

        mapreq = DualCoding::MapBuilderRequest(DualCoding::MapBuilderRequest::localMap);
        const DualCoding::Point& kTargetPt = targets[0]->getCentroid();
        const DualCoding::Point& kAgentPt = DualCoding::VRmixin::theAgent->getCentroid();
        //const float kAgentOrientation = DualCoding::VRmixin::theAgent->getOrientation();
        mapreq.addObjectColor(targets[0]->getType(), targets[0]->getColor());
				cout << "getMapBuilderRequest using target " << targets[0] << endl;
        float dtheta = bearingFromAgentToPoint(kTargetPt);
        float h = kTargetPt.xyDistanceFrom(kAgentPt);
        float x = cos(dtheta) * h;
        float y = sin(dtheta) * h;
        float z = 0.0f;

        switch (targets[0]->getType()) {
            case DualCoding::cylinderDataType:
                z = ShapeRootTypeConst(targets[0], CylinderData)->getHeight() / 2.0f;
                break;

            default:
                break;
        }

        NEW_SHAPE(gazePoint, PointData,
            new PointData(DualCoding::VRmixin::localShS,
                DualCoding::Point(x, y, z, DualCoding::egocentric)));
        mapreq.searchArea = gazePoint;
        return mapreq;
    }

    bool VisualNavErrMonTask::taskIsComplete(const KoduWorld& kWorldState) {
        switch (taskStatus) {
            case PerceptualTaskBase::TS_IN_PROGRESS:
            {
                if (!kWorldState.thisAgent.isExecutingMotionAction()) {
                    taskStatus = TS_COMPLETE;
                    return true;
                }

                //******** temp fix until base object is done
                if (distanceInBetweenAgentAndObject(targets[0])
                    < KoduConditionBump::kMaxDistanceAwayToSenseBump)
                {
                    taskStatus = TS_COMPLETE;
                    return true;
                }
                //*******************

                return false;
            }

            case PerceptualTaskBase::TS_SUCCESSFUL:
            case PerceptualTaskBase::TS_FAILURE:
            case PerceptualTaskBase::TS_COMPLETE:
                return true;

            default:
                return false;
        }
    }
}
