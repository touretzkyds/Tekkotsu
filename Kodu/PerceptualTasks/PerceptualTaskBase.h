#ifndef PERCEPTUAL_TASK_BASE_H_
#define PERCEPTUAL_TASK_BASE_H_

// INCLUDES
// c++
#include <string>
#include <vector>

// tekkotsu
#include "Crew/MapBuilderRequest.h"
#include "Crew/PilotRequest.h"
#include "DualCoding/ShapeRoot.h"

namespace Kodu {

    // tekkodu forward declarations
    class KoduWorld;

    enum PerceptualTaskType_t {
        PT_VIS_BUMP_DETECTION = 0,
        PT_GRIPPER_VIS_MONITOR,
        PT_VIS_LOCALIZATION,
        PT_VIS_NAV_ERR_MON
    };

    class PerceptualTaskBase {
    public:
        enum TaskStatus_t {
            TS_IN_PROGRESS = 0,
            TS_COMPLETE,
            TS_SUCCESSFUL,
            TS_FAILURE
        };

        //! Constructor
        PerceptualTaskBase(PerceptualTaskType_t perceptTaskType, unsigned int taskId)
          : type(perceptTaskType),
            id(taskId),
            mapreq(),
            taskStatus(TS_IN_PROGRESS)
        {
					// std::cout << "Created task #" << id << std::endl;
        }

        //! Copy constructor
        PerceptualTaskBase(const PerceptualTaskBase& kTask)
          : type(kTask.type),
            id(kTask.id),
            mapreq(kTask.mapreq),
            taskStatus(kTask.taskStatus)
        { }

        //! Destructor
        virtual ~PerceptualTaskBase() {
            // no explicit implementation
        }

        //! Assignment operator
        PerceptualTaskBase& operator=(const PerceptualTaskBase& kTask) {
            if (this != &kTask) {
                type = kTask.type;
                id = kTask.id;
                mapreq = kTask.mapreq;
                taskStatus = kTask.taskStatus;
            }
            return *this;
        }

        //! Checks if a task can execute (reimplemented in all derived class)
        virtual bool canExecute(const KoduWorld&) = 0;

        //! Examines the results from a task
        virtual void examineTaskResults();

        //! Returns the MapBuilder request the robot should perform
        virtual const DualCoding::MapBuilderRequest& getMapBuilderRequest();

        //! Returns the Pilot request the robot should perform
        virtual const DualCoding::PilotRequest getPilotRequest() { return PilotRequest(); }

        //! Returns the task's current status
        TaskStatus_t getStatus() const;

        //! Returns the task's id
        float getTaskId() const;

        //! Returns the task type
        PerceptualTaskType_t getType() const;

        void setTaskStatus(TaskStatus_t);

        //! Checks whether a task is complete
        virtual bool taskIsComplete(const KoduWorld&);

    protected:
        PerceptualTaskType_t type;              //!< The type of perceptual task
        unsigned int id;                        //!< The task's id
        DualCoding::MapBuilderRequest mapreq;   //!< The MapBuilder request the robot needs to perform
        TaskStatus_t taskStatus;                //!< The (current) status of a perceptual task
    };
}


#endif // PERCEPTUAL_TASK_BASE_H_
