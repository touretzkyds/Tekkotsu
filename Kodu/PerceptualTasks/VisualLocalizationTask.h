#ifndef VISUAL_LOCALIZATION_TASK_H_
#define VISUAL_LOCALIZATION_TASK_H_

// INCLUDES
// c++
#include <iostream>

// tekkotsu
#include "Crew/PilotRequest.h"
#include "DualCoding/Point.h"
#include "DualCoding/ShapeFuns.h"
#include "DualCoding/ShapePoint.h"
#include "DualCoding/ShapeRoot.h"

namespace Kodu {

    // tekkodu forward declarations
    class PerceptualTaskBase;
    
    class VisualLocalizationTask : public PerceptualTaskBase {
    public:
        //! Constructor
        VisualLocalizationTask(const std::map<int, DualCoding::Point>& kStarConstellation,
            int maxStarsNeededToLocalize = kMinStarsRequiredToLocalize)
          : PerceptualTaskBase(PT_VIS_LOCALIZATION, ++idCount),
            localizationPoints(kStarConstellation),
            maxStarsRequested(maxStarsNeededToLocalize)
        {
            if (maxStarsNeededToLocalize > static_cast<int>(localizationPoints.size())) {
                std::cout << "VisualLocalizationTask: WARNING---you are requesting a max of "
                    << maxStarsNeededToLocalize << " stars, but there are only "
                    << localizationPoints.size() << " stars in the constellation (available).\n";
            }

            if (maxStarsNeededToLocalize < kMinStarsRequiredToLocalize) {
                std::cout << "VisualLocalizationTask: WARNING---you are requesting a max of "
                    << maxStarsNeededToLocalize << " stars, but you need a minimun of "
                    << kMinStarsRequiredToLocalize << " stars to localize.\n";
                maxStarsRequested = kMinStarsRequiredToLocalize;
            }
        }

        //! Copy constructor
        VisualLocalizationTask(const VisualLocalizationTask& kTask)
          : PerceptualTaskBase(kTask),
            localizationPoints(kTask.localizationPoints),
            maxStarsRequested(kTask.maxStarsRequested)
        { }

        //! Destructor
        ~VisualLocalizationTask() {
            // no explicit implementation
        }

        //! Assignment operator
        VisualLocalizationTask& operator=(const VisualLocalizationTask& kTask) {
            if (this != &kTask) {
                PerceptualTaskBase::operator=(kTask);
                localizationPoints = kTask.localizationPoints;
                maxStarsRequested = kTask.maxStarsRequested;
            }
            return *this;
        }

        //! Checks if this task can execute
        virtual bool canExecute(const KoduWorld&);

        //! Generates the Pilot request the agent needs to localize
        virtual const DualCoding::PilotRequest getPilotRequest();

        static const int kMinStarsRequiredToLocalize;
        
    private:
        static unsigned int idCount;    //!< used to create an id for each task
        std::map<int, DualCoding::Point> localizationPoints;   //!< a copy of the constellation
        int maxStarsRequested;
    };
}

#endif // VISUAL_LOCALIZATION_TASK_H_
