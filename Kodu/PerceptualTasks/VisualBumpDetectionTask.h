#ifndef VISUAL_BUMP_DETECTION_TASK_H_
#define VISUAL_BUMP_DETECTION_TASK_H_

// INCLUDES
// tekkotsu
#include "DualCoding/ShapeRoot.h"
#include "DualCoding/VRmixin.h"

#include "Kodu/Primitives/KoduConditionBump.h"

namespace Kodu {
    // tekkodu forward declarations
    //class KoduConditionBump;
    class KoduWorld;
    class PerceptualTaskBase;
    
    class VisualBumpDetectionTask : public PerceptualTaskBase {
    public:
        //! Constructor
        VisualBumpDetectionTask(KoduConditionBump* bmpCondition, KoduWorld* worldState)
          : PerceptualTaskBase(PT_VIS_BUMP_DETECTION, ++idCount),
            condition(bmpCondition),
            wState(worldState)
        {
            condition->setAgentCanUsePrimitive(false);
            std::cout << "Created VisualBumpDetectionTask #" << id << " to detect a(n) " << condition->getObjectColor()
											<< " " << condition->getObjectType() << std::endl;
        }

        //! Copy constructor
        VisualBumpDetectionTask(const VisualBumpDetectionTask& kTask)
          : PerceptualTaskBase(kTask),
            condition(kTask.condition),
            wState(kTask.wState)
        { }

        //! Destructor
        ~VisualBumpDetectionTask() {
            condition = NULL;
            wState = NULL;
        }

        //! Assignment operator
        VisualBumpDetectionTask& operator=(const VisualBumpDetectionTask& kTask) {
            if (this != &kTask) {
                PerceptualTaskBase::operator=(kTask);
                condition = kTask.condition;
                wState = kTask.wState;
            }
            return *this;
        }

        //! Checks if the VisualBumpDetectionTask can execute
        virtual bool canExecute(const KoduWorld&);

        //! Examines the results from the MapBuilder request to see if an object was "bumped"
        virtual void examineTaskResults();

        //! Creates a MapBuilder request telling the robot where to look to detect the bump
        virtual const DualCoding::MapBuilderRequest& getMapBuilderRequest();

    private:
        static unsigned int idCount;    //!< Used to generate id numbers for VisualBumpDetectionTask
        KoduConditionBump* condition;   //!< The bump condition that needs to detect the bump
        KoduWorld* wState;
    };
}

#endif // VISUAL_BUMP_DETECTION_TASK_H_
