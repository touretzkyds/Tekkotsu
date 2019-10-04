// INCLUDES
// tekkodu
#include "Kodu/KoduPage.h"
#include "Kodu/General/GeneralFncs.h"
#include "Kodu/PerceptualTasks/PerceptualTaskBase.h"
#include "Kodu/Primitives/PerceptionSearch.h"

#include "Kodu/KoduAgent.h"

// tekkotsu
#include "DualCoding/ShapeTypes.h"
#include "Shared/get_time.h"
#include "Shared/mathutils.h"
using namespace DualCoding;

namespace Kodu {

    KoduAgent::KoduAgent()
      : gripperObject(),
        agentIsExecutingManipAction(false),
        agentWantsToDropObject(false),
        agentWantsToGrabObject(false),
        targetObjectIsInGripper(false),
        agentIsExecutingMotionAction(false),
        currMotionCmd(),
        distanceTravelled(0.0f),
        agentWantsToGiveObject(false),
        giveTargetObject(),
        agentIsReceiving(false),
        agentReceivingFrom(0),
        gotGiveReady(false),
        gotGiveInformation(false),
        giveAngleToTurn(0.0),
        giveObjType(""),
        pages(),
        currPageIndex(0),
        newReqdPage(0),
        ptasks(),
        scoreQueue(),
        stringToSpeak(""),
        playQueue(),
        agentGazePoints()
    {
        // generate the gaze points
        generateGazePoints();
    }

    KoduAgent::~KoduAgent() {
        GeneralFncs::destroyAllPtrsInQueue(ptasks);
        GeneralFncs::destroyAllPtrsInVector(pages);
        stringToSpeak.clear();
    }

    KoduAgent& KoduAgent::operator=(const KoduAgent& kAgent) {
        if (this != &kAgent) {
            gripperObject = kAgent.gripperObject;
            agentIsExecutingManipAction = kAgent.agentIsExecutingManipAction;
            agentWantsToDropObject = kAgent.agentWantsToDropObject;
            agentWantsToGrabObject = kAgent.agentWantsToGrabObject;
            targetObjectIsInGripper = kAgent.targetObjectIsInGripper;
            agentIsExecutingMotionAction = kAgent.agentIsExecutingMotionAction;
            currMotionCmd = kAgent.currMotionCmd;
            distanceTravelled = kAgent.distanceTravelled;
            pages = kAgent.pages;
            currPageIndex = kAgent.currPageIndex;
            newReqdPage = kAgent.newReqdPage;
            ptasks = kAgent.ptasks;
            scoreQueue = kAgent.scoreQueue;
            stringToSpeak = kAgent.stringToSpeak;
            playQueue = kAgent.playQueue;
            agentGazePoints = kAgent.agentGazePoints;
        }
        return *this;
    }

    /// ================================ Static initializations ================================ ///
    const float KoduAgent::kLocalizationDistanceThreshold = 1000.0f;

    /// ================================ Gaze functions ================================ ///
    void KoduAgent::generateGazePoints() {
        // SA: the search area (in degrees)--(from -SA/2 to SA/2)
			  const float kSearchArea = mathutils::deg2rad(50.0f);  // was 200 degrees
        const float kAngleIncrement = mathutils::deg2rad(50.0f);   // was 25 degrees from Troi
        // the search radius
        float radius = 750.0f;
        // the beginning angle
        float currAngle = -1.0f * kSearchArea / 2.0f;
        // loop until all the search points have been generated
        while (currAngle <= (kSearchArea / 2.0f)) {
            agentGazePoints.push_back(
                Point(
                    cos(currAngle) * radius,  // x-value
                    sin(currAngle) * radius,  // y-value
                    0.0f,                     // z-value
                    DualCoding::egocentric    // point is relative to agent body
                ));
            currAngle += kAngleIncrement; // increment the current angle after generating each point
        }
    }

    const std::vector<Point>& KoduAgent::getGazePoints() const {
        return agentGazePoints;
    }

    /// ================================ Grasper functions ================================ ///
    bool KoduAgent::isExecutingManipAction() const {
        return agentIsExecutingManipAction;
    }

    bool KoduAgent::isHoldingAnObject() const {
        return targetObjectIsInGripper;
    }

    void KoduAgent::manipulationComplete() {
        // check the type of manipulation completed
        if (agentWantsToDropObject) {
            setWantsToDropObjectFlag(false);    // the robot no longer "wants to drop an object"
            setTargetInGripperFlag(false);      // the object is no longer in the gripper
        }
        // else, it must be the grab action
        else {
            setWantsToGrabObjectFlag(false);    // the robot no longer "wants to grab an object"
            setTargetInGripperFlag();           // (implicit true) object is in the gripper
        }
        setIsExecutingManipActionFlag(false);   // the manipulation action is no longer executing
    }

    void KoduAgent::setIsExecutingManipActionFlag(bool bval) {
        agentIsExecutingManipAction = bval;
    }

    void KoduAgent::setTargetInGripperFlag(bool bval) {
        targetObjectIsInGripper = bval;
    }

    void KoduAgent::setWantsToDropObjectFlag(bool bval) {
        agentWantsToDropObject = bval;
    }

    void KoduAgent::setWantsToGrabObjectFlag(bool bval) {
        agentWantsToGrabObject = bval;
    }

    void KoduAgent::signalDropActionStart() {
        setIsExecutingManipActionFlag();    // states the robot is executing a manipulation action
        setWantsToDropObjectFlag();         // the robot wants to drop an object
    }

    void KoduAgent::signalGrabActionStart() {
        setIsExecutingManipActionFlag();    // states the robot is executing a manipulation action
        setWantsToGrabObjectFlag();         // the robot wants to grab an object
    }

    void KoduAgent::signalGiveActionStart() {
        setIsExecutingManipActionFlag();
        setIsExecutingMotionActionFlag();
        setWantsToGiveObjectFlag();
    }

    bool KoduAgent::wantsToDropObject() const {
        return agentWantsToDropObject;
    }

    bool KoduAgent::wantsToGrabObject() const {
        return agentWantsToGrabObject;
    }

    /// ================================ Motion functions ================================ ///
    bool KoduAgent::bodyIsInMotion() const {
        return VRmixin::isWalkingFlag;
    }

    bool KoduAgent::isExecutingMotionAction() const {
        return agentIsExecutingMotionAction;
    }

    bool KoduAgent::needsToLocalize() const {
        return (distanceTravelled >= kLocalizationDistanceThreshold);
    }

    void KoduAgent::motionComplete() {
			// make sure the current motion command is invalid
			currMotionCmd = MotionCommand();
			setIsExecutingMotionActionFlag(false);
    }

    void KoduAgent::signalMotionActionStart() {
        setIsExecutingMotionActionFlag();
    }

    void KoduAgent::setIsExecutingMotionActionFlag(bool bval) {
        agentIsExecutingMotionAction = bval;
    }

    void KoduAgent::setMotionCommand(const MotionCommand& kCmd) {
        currMotionCmd = kCmd;
    }

    /// ================================ Give functions =================================== ///
    void KoduAgent::setWantsToGiveObjectFlag(bool bval) {
        agentWantsToGiveObject = bval;
    }

    void KoduAgent::setIsReceiving(bool bval) {
        agentIsReceiving = bval;
        setIsExecutingManipActionFlag(bval);
        setIsExecutingMotionActionFlag(bval);

    }

    bool KoduAgent::wantsToGiveObject() const {
        return agentWantsToGiveObject;
    }


    /// ================================ Scoring functions ================================ ///
    bool KoduAgent::hasNewScoreChanges() const {
        return (!scoreQueue.empty());
    }

    /// ================================ Page functions ================================ ///
    KoduPage* KoduAgent::getCurrentPage() const {
        return pages[currPageIndex];
    }

    KoduPage* KoduAgent::getPage(unsigned int pageNumber) const {
        return getPageInPos(pageNumber - 1);
    }

    KoduPage* KoduAgent::getPageInPos(unsigned int pageIndex) const {
        if (pageIndex < pages.size()) {
            return pages[pageIndex];
        } else {
            std::cout << "Page index \"" << pageIndex << "\" is out of bounds!\nReturning NULL...\n";
            return NULL;
        }
    }

    bool KoduAgent::hasNewPageNumber() const {
        return (newReqdPage > 0);
    }

    /// ================================ Speech functions ================================ ///
    bool KoduAgent::hasTextToSay() const {
        return (!stringToSpeak.empty());
    }

    /// ================================ Sound functions ================================ ///
    bool KoduAgent::hasSoundsToPlay() const {
        return (!playQueue.empty());
    }
}
