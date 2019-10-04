#ifndef VISUAL_GRIPPER_MONITOR_TASK_H_
#define VISUAL_GRIPPER_MONITOR_TASK_H_

// INCLUDES
// c++
#include <deque>
#include <iostream>

// tekkotsu
#include "DualCoding/ShapeAprilTag.h"
#include "DualCoding/ShapeRoot.h"
#include "DualCoding/VRmixin.h"

// tekkodu
#include "Kodu/PosOrientState.h"

namespace Kodu {
    // tekkodu forward declarations
    class KoduWorld;
    class PerceptualTaskBase;
    class PosOrientState;

    //! A functor that will check if the object is still in the robot's gripper
    class MatchesObjectTagInGripper : public DualCoding::UnaryShapeRootPred {
    public:
        //! Constructor
        MatchesObjectTagInGripper()
          : DualCoding::UnaryShapeRootPred(),
            aprTagCentroid(),
            aprTagId(0)
        { }

        //! Copy constructor
        MatchesObjectTagInGripper(const MatchesObjectTagInGripper& kPred)
          : DualCoding::UnaryShapeRootPred(),
            aprTagCentroid(kPred.aprTagCentroid),
            aprTagId(kPred.aprTagId)
        {
            if (aprTagId < 0) std::cerr << "The object's april tag was not properly identified.\n";
        }

        //! Destructor
        ~MatchesObjectTagInGripper() {
            // no explicit implementation
        }

        //! Assignment operator
        MatchesObjectTagInGripper& operator=(const MatchesObjectTagInGripper& kPred) {
            if (this != &kPred) {
                DualCoding::UnaryShapeRootPred::operator=(kPred);
                aprTagCentroid = kPred.aprTagCentroid;
                aprTagId = kPred.aprTagId;
            }
            return *this;
        }

        //! Checks if a shape matches the specified criteria
        bool operator()(const DualCoding::ShapeRoot&) const;

        friend class VisualGripperMonitorTask;

    private:
        static float const kMaxAprTagCentroidVariance;
        DualCoding::Point aprTagCentroid;   //!< The april tag's centroid (in local shape space)
        int aprTagId;                       //!< The object's april tag id number
    };

    class VisualGripperMonitorTask : public PerceptualTaskBase {
    public:
        //! Constructor
        VisualGripperMonitorTask(const DualCoding::ShapeRoot& kObjInGripper,
            const DualCoding::Shape<DualCoding::AprilTagData>& kObjAprilTag)
          : PerceptualTaskBase(PT_GRIPPER_VIS_MONITOR, ++idCount),
            taskPred(),
            objInGripper(kObjInGripper),
            lastSuccessfulState()
        {
            int tagId = -1;
            DualCoding::Point tagCen;
            
            if (!objInGripper.isValid()) {
                std::cerr << "The objInGripper reference is not valid.\n";
            }

            if (kObjAprilTag.isValid()) {
                tagId = kObjAprilTag->getTagID();
                switch (kObjAprilTag->getRefFrameType()) {
                    case DualCoding::egocentric:
                        tagCen = kObjAprilTag->getCentroid();
                        break;

                    case DualCoding::allocentric:
                        tagCen = VRmixin::mapBuilder->importWorldToLocal(kObjAprilTag)->getCentroid();
                        break;

                    default:
                        std::cerr << "Unhandled reference frame type in VisualGripperMonitorTask.\n";
                        break;
                }
            } else {
                std::cerr << "April Tag reference is not valid!!!\n";
            }
            taskPred.aprTagId = tagId;
            taskPred.aprTagCentroid = tagCen;
            float orient = VRmixin::theAgent->getOrientation();
            lastSuccessfulState = PosOrientState(VRmixin::theAgent->getCentroid(), orient);
        }

        //! Copy constructor
        VisualGripperMonitorTask(const VisualGripperMonitorTask& kTask)
          : PerceptualTaskBase(kTask),
            taskPred(kTask.taskPred),
            objInGripper(kTask.objInGripper),
            lastSuccessfulState(kTask.lastSuccessfulState)
        {
            if (!objInGripper.isValid()) std::cerr << "The objInGripper reference is not valid.\n";
        }

        //! Destructor
        ~VisualGripperMonitorTask() {
            // no explicit implementation
        }

        //! Assignment operator
        VisualGripperMonitorTask& operator=(const VisualGripperMonitorTask& kTask) {
            if (this != &kTask) {
                PerceptualTaskBase::operator=(kTask);
                taskPred = kTask.taskPred;
                objInGripper = kTask.objInGripper;
                lastSuccessfulState = kTask.lastSuccessfulState;
            }
            return *this;
        }

        //! Checks if the VisualGripperMonitorTask can execute
        virtual bool canExecute(const KoduWorld&);

        //! Examines the results from the MapBuilder request to see if the object is still in the gripper
        virtual void examineTaskResults();

        //! Returns the object in the robot's gripper
        const DualCoding::ShapeRoot& getGripperObject() const;

        //! Returns the last successful state where the object was still in the robot gripper
        const PosOrientState& getLastSuccessfulState() const;

        //! Returns the MapBuilder request telling the robot where to look at the gripper
        virtual const DualCoding::MapBuilderRequest& getMapBuilderRequest();

        //! Returns the April Tag's centroid (relative to the body when the object is in the gripper)
        const DualCoding::Point& getTagCentroid() const;

        //! Returns the April Tag's id
        int getTagId() const;

        //! Relocates the object's tag (used after manipulation recovery)
        void relocateTagCentroid(const DualCoding::Point&);

        //! Checks whether a task is complete
        virtual bool taskIsComplete(const KoduWorld&);

    private:
        static unsigned int idCount;    //!< Used to generate id numbers for VisualGripperMonitorTask
        MatchesObjectTagInGripper taskPred;
        DualCoding::ShapeRoot objInGripper; //!< The reference to the object in the gripper
        PosOrientState lastSuccessfulState;
    };
}

#endif // VISUAL_GRIPPER_MONITOR_TASK_H_
