#ifndef KODU_AGENT_H_
#define KODU_AGENT_H_

// INCLUDES
// c++
#include <cmath>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

// tekkotsu
#include "DualCoding/Point.h"
#include "DualCoding/ShapeRoot.h"
using namespace DualCoding;

// tekkodu
#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Keepers/ScoreKeeper.h"
#include "Kodu/Primitives/KoduActionMotion.h"

namespace Kodu {

    // forward declarations
    class KoduPage;             // KoduPage.h
    class PerceptualTaskBase;   // PerceptualTaskBase.h

    class KoduAgent {
    public:
        //! Constructor
        KoduAgent();

        //! Destructor
        ~KoduAgent();

        //! Assignment operator
        KoduAgent& operator=(const KoduAgent&);

        /// ================================ Gazing functions ================================ ///
        //! Returns the gaze points
        const std::vector<Point>& getGazePoints() const;

        /// ================================ Grasper functions ================================ ///
        //! States if the agent is attempting to grab an object
        bool isExecutingManipAction() const;

        //! States whether or not the agent is (supposed to be) holding something
        bool isHoldingAnObject() const;

        //! Signals the robot has completed the manipulation task (sets a particular set of flags)
        void manipulationComplete();

        //! Sets the "(agent) is attempting grab" flag
        void setIsExecutingManipActionFlag(bool bval = true);

        //! Sets the "target object is in gripper" flag
        void setTargetInGripperFlag(bool bval = true);

        //! Signals that the agent wants to drop something
        void setWantsToDropObjectFlag(bool bval = true);

        //! Signals that the agent wants to grab something
        void setWantsToGrabObjectFlag(bool bval = true);

        //! Signals the robot wants to drop an object (sets a particular set of flags)
        void signalDropActionStart();

        //! Signals the robot wants to grab an object (sets a particular set of flags)
        void signalGrabActionStart();

        //! States whether or not the agent wants to drop an object
        bool wantsToDropObject() const;

        //! States whether or not the agent has an object it wants to grab
        bool wantsToGrabObject() const;

        /// ================================ Give functions ================================== ///
        //! Signals that the agent wants to give something
        void setWantsToGiveObjectFlag(bool bval = true);

        //! Signals the robot wants to give an objec to another robot (sets flags)
        void signalGiveActionStart();

        //! Signals whether the agent is currently receiving
        void setIsReceiving(bool bval = true);

        //! States whether or not the agent wants to give an object to another agent
        bool wantsToGiveObject() const;

        /// ================================ Motion functions ================================ ///
        //! Checks if the robot's body is moving (during motion action the body may stop)
        bool bodyIsInMotion() const;

        //! Checks if the agent is walking (returns value of "agent is walking" flag)
        bool isExecutingMotionAction() const;

        //! Checks if the agent needs to localize
        bool needsToLocalize() const;

        //! Signals the robot has completed a motion command
        void motionComplete();

        //! Signals the robot is executing a motion command
        void signalMotionActionStart();

        //! Sets the 'agent is executing motion command' flag (default value = true)
        void setIsExecutingMotionActionFlag(bool bval = true);

        //! Sets the current motion command
        void setMotionCommand(const MotionCommand&);

        /// ================================ Page functions ================================ ///
        //! Returns the page currently being evaluated (determined by current page index variable)
        KoduPage* getCurrentPage() const;

        //! Returns the specified page (using it's page number)
        KoduPage* getPage(unsigned int pageNumber) const;

        //! Returns the specified page (using it's location in the vector--(PAGE# - 1))
        KoduPage* getPageInPos(unsigned int pageIndex) const;

        //! Checks if the agent wants to switch pages
        bool hasNewPageNumber() const;

        /// ================================ Scoring functions ================================ ///
        //! Checks if the agent has a string to speak
        bool hasNewScoreChanges() const;

        /// ================================ Speech functions ================================ ///
        //! Checks if the agent has a string to speak
        bool hasTextToSay() const;

        /// ================================ Sound functions ================================ ///
        //! Checks if the agent has sounds to play
        bool hasSoundsToPlay() const;

    private:
        /// ================================ Gaze functions ================================ ///
        //! Generates the agent's gaze points (the points in space to search for objects)
        void generateGazePoints();

        // Disallows the copy constructor
        DISALLOW_COPY(KoduAgent);

    public: //// ================= The Public Agent Variables ================= ////
        // === Grasp variables === //
        ShapeRoot gripperObject;        //!< The object the agent is holding/will be holding
        bool agentIsExecutingManipAction; //!< (flag) States if the agent is attempting to grab an object
        bool agentWantsToDropObject;    //!< (flag) States if the agent wants to drop an object
        bool agentWantsToGrabObject;    //!< (flag) States if the agent wants to grab an object
        bool targetObjectIsInGripper;   //!< (flag) States if the target object is in the gripper

        // === Motion variables === //
        //! The mininum travelling distance (including turns) to consider performing localization
        static const float kLocalizationDistanceThreshold;
        bool agentIsExecutingMotionAction;
        MotionCommand currMotionCmd;        //!< The current motion command
        float distanceTravelled;            //!< The accumulated distance the robot has travelled

        // === Give variables === //
        bool agentWantsToGiveObject;        //!< (flag) States if the agent wants to give an object
        Shape<AgentData> giveTargetObject;
        bool agentIsReceiving;
        int agentReceivingFrom;
        bool gotGiveReady;
        bool gotGiveInformation;
        float giveAngleToTurn;
        std::string giveObjType;

        // === Page variables === //
        std::vector<KoduPage*> pages;       //!< The vector of pages containing kode
        unsigned int currPageIndex;         //!< The page (index) currently being executed
        unsigned int newReqdPage;           //!< Stores the page number the agent wants to switch to

        // === Perceptual Tasks container === //
        std::queue<PerceptualTaskBase*> ptasks; //!< A queue containing all the perceptual tasks

        // === Score variables === //
        std::queue<ScoreChange> scoreQueue; //!< The queue of score operations and their value

        // === Speech variables === //
        std::string stringToSpeak;          //!< The string the agent wants to speak

        // === Sound variables === //
        std::queue<std::string> playQueue;  //!< The queue of sound files the agent wants to play

    private: //// ================= The Private Agent Variables ================= ////
        // === Gaze points variables === //
        //! egocentric (relative to the robot's body) points to look at
        std::vector<Point> agentGazePoints;
    };
}

#endif // end of KODU_AGENT_H_
