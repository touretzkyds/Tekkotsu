#ifndef VISUAL_NAV_ERR_MON_TASK_H_
#define VISUAL_NAV_ERR_MON_TASK_H_

// INCLUDES
// c++
#include <vector>

// tekkotsu
#include "DualCoding/ShapeRoot.h"

namespace Kodu {

    // tekkodu forward declarations
    class PerceptualTaskBase;
    
    /**
     * ASSUMPTIONS:
     * - The target shape passed to the constructor is in the world shape space.
     *
    **/
    class VisualNavErrMonTask : public PerceptualTaskBase {
    public:
        //! Constructor #1
        VisualNavErrMonTask(const DualCoding::ShapeRoot& kTargetShape)
          : PerceptualTaskBase(PT_VIS_NAV_ERR_MON, ++idCount),
            errorCount(0),
            targets(),
            agentLastOrientation(0.0f)
        {
            targets.push_back(kTargetShape);
            std::cout << "Created VisualNavErrorMonTask #" << id << " to track " << kTargetShape << std::endl;
        }

        //! Constructor #2
        VisualNavErrMonTask(const std::vector<DualCoding::ShapeRoot>& kTargetShapes)
          : PerceptualTaskBase(PT_VIS_NAV_ERR_MON, ++idCount),
            errorCount(0),
            targets(kTargetShapes),
            agentLastOrientation(0.0f)
        { }

        //! Copy constructor
        VisualNavErrMonTask(const VisualNavErrMonTask& kTask)
          : PerceptualTaskBase(kTask),
            errorCount(kTask.errorCount),
            targets(kTask.targets),
            agentLastOrientation(kTask.agentLastOrientation)
        { }

        //! Destructor
        ~VisualNavErrMonTask() {
            // no explicit implementation
        }

        //! Assignment operator
        VisualNavErrMonTask& operator=(const VisualNavErrMonTask& kTask) {
            if (this != &kTask) {
                PerceptualTaskBase::operator=(kTask);
                errorCount = kTask.errorCount;
                targets = kTask.targets;
                agentLastOrientation = kTask.agentLastOrientation;
            }
            return *this;
        }

        //! Checks if the VisualWalkProgressTask can execute
        virtual bool canExecute(const KoduWorld&);

        //! Examines the results from the MapBuilder request to see if the robot made "progress"
        virtual void examineTaskResults();

        //! Dynamically generates the point the agent should fixate on
        virtual const DualCoding::MapBuilderRequest& getMapBuilderRequest();

        //! Checks if the task is complete
        virtual bool taskIsComplete(const KoduWorld&);

        //! The max number of cumulative errors (error: when the agent did not find a match)
        //static unsigned int const kMaxErrorOccurences;

    private:
        static unsigned int idCount;                //!< Used to generate id numbers
        unsigned int errorCount;                    //!< Cumulative error count
        std::vector<DualCoding::ShapeRoot> targets; //!< The shapes the agent needs to look at
        float agentLastOrientation;
    };

}

#endif // VISUAL_NAV_ERR_MON_TASK_H_
