// INCLUDES
// tekkodu
#include "Kodu/KoduAgent.h"
#include "Kodu/KoduWorld.h"
#include "Kodu/PerceptualTasks/PerceptualTaskBase.h"

namespace Kodu {

    void PerceptualTaskBase::examineTaskResults() {
        // does nothing
        return;
    }

    const DualCoding::MapBuilderRequest& PerceptualTaskBase::getMapBuilderRequest() {
        return mapreq;
    }

    PerceptualTaskBase::TaskStatus_t PerceptualTaskBase::getStatus() const {
        return taskStatus;
    }

    float PerceptualTaskBase::getTaskId() const {
        return id;
    }

    PerceptualTaskType_t PerceptualTaskBase::getType() const {
        return type;
    }

    void PerceptualTaskBase::setTaskStatus(TaskStatus_t newStatus) {
        taskStatus = newStatus;
    }

    bool PerceptualTaskBase::taskIsComplete(const KoduWorld& kWorldState) {
        return (taskStatus == TS_SUCCESSFUL
             || taskStatus == TS_FAILURE
             || taskStatus == TS_COMPLETE);
    }
}
