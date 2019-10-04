// INCLUDES
// tekkotsu
#include "Kodu/KoduWorld.h"
//#include "Kodu/PosOrientState.h"
#include "Kodu/PerceptualTasks/PerceptualTaskBase.h"
#include "Kodu/PerceptualTasks/GripperVisualMonitorTask.h"

namespace Kodu {
    
    /**
     * ASSUMPTION:
     * - grabbed object's radius is greater than or equal to 1/2 the distance from the inside of
     *   the gripper to the gripper's fingers.
    **/
    float const MatchesObjectTagInGripper::kMaxAprTagCentroidVariance = 30.0f;

    unsigned int VisualGripperMonitorTask::idCount = 70000;
    
    bool MatchesObjectTagInGripper::operator()(const DualCoding::ShapeRoot& kShape) const {
        if (kShape->getType() != DualCoding::aprilTagDataType)
            return false;
        std::cout << "diff in centroids = ";
        float diff = (kShape->getCentroid() - aprTagCentroid).xyNorm();
        std::cout << diff << "mm; ";
        // for debugging
        bool match = ShapeRootTypeConst(kShape, DualCoding::AprilTagData)->getTagID() == aprTagId;
        std::cout << "april tag ids " << (match ? "" : "do not") << " match\n";
        return (diff < kMaxAprTagCentroidVariance && match);
    }

    bool VisualGripperMonitorTask::canExecute(const KoduWorld& kWorldState) {
        // return (kWorldState.thisAgent.isHoldingAnObject() && kWorldState.thisAgent.isWalking());
        return (kWorldState.thisAgent.isHoldingAnObject() && kWorldState.thisAgent.bodyIsInMotion()
            && kWorldState.thisAgent.isExecutingMotionAction());
    }

    void VisualGripperMonitorTask::examineTaskResults() {
        std::cout << "Examining visual gripper monitor results...\n";
        // search for the gripper object's April Tag in the local shape space
        DualCoding::Shape<DualCoding::AprilTagData> objectTag
            = DualCoding::find_if<DualCoding::AprilTagData>(DualCoding::VRmixin::localShS,taskPred);

        std::cout << "objectTag reference valid? ";
        // if the tag was found, log the agent's current state
        if (objectTag.isValid()) {
            float orient = DualCoding::VRmixin::theAgent->getOrientation();
            lastSuccessfulState = PosOrientState(DualCoding::VRmixin::theAgent->getCentroid(), orient);
            std::cout << "yes, it is. state logged at: " << lastSuccessfulState.position
                << " w/ orient = " << lastSuccessfulState.orientation << std::endl;
        } else {
            std::cout << "no, reporting failure. last successful state: pos = "
                << lastSuccessfulState.position << "; orient = " << lastSuccessfulState.orientation
                << std::endl;
            taskStatus = TS_FAILURE;
        }
    }

    const DualCoding::ShapeRoot& VisualGripperMonitorTask::getGripperObject() const {
        return objInGripper;
    }

    const PosOrientState& VisualGripperMonitorTask::getLastSuccessfulState() const {
        return lastSuccessfulState;
    }

    const DualCoding::MapBuilderRequest& VisualGripperMonitorTask::getMapBuilderRequest() {
        NEW_SHAPE(tagApproxPosition, DualCoding::PointData,
            new DualCoding::PointData(DualCoding::VRmixin::localShS, taskPred.aprTagCentroid));
        mapreq = DualCoding::MapBuilderRequest(DualCoding::MapBuilderRequest::localMap);
        mapreq.searchArea = tagApproxPosition;
        mapreq.setAprilTagFamily();
        return mapreq;
    }

    const DualCoding::Point& VisualGripperMonitorTask::getTagCentroid() const {
        return taskPred.aprTagCentroid;
    }

    int VisualGripperMonitorTask::getTagId() const {
        return taskPred.aprTagId;
    }

    void VisualGripperMonitorTask::relocateTagCentroid(const DualCoding::Point& kNewLocation) {
        taskPred.aprTagCentroid = kNewLocation;
    }

    bool VisualGripperMonitorTask::taskIsComplete(const KoduWorld& kWorldState) {
        if (taskStatus != TS_IN_PROGRESS)
            return true;

        if (!kWorldState.thisAgent.isHoldingAnObject()) {
            taskStatus = TS_COMPLETE;
            return true;
        }
        return false;
    }
}
