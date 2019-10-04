#include "Shared/RemoteState.h"
#include "Events/RemoteRouter.h"
#include "Events/EventRouter.h"
#include "Events/DataEvent.h"

const int RemoteState::sizes[] = {
    NumOutputs*sizeof(float),
	NumButtons*sizeof(float),
	NumSensors*sizeof(float),
};

RemoteState::RemoteState(const RemoteRouter *p) : parent(p) {
    unsigned int i = 0;
    for (i = 0; i < NumOutputs; i++)
        outputs[i] = 0;
}

RemoteState::~RemoteState() {

}

void RemoteState::update(char *data) {
    //Get the type of the data, a StateType
    StateType t = *(StateType *)data;
    data += sizeof(StateType);

    //Get the size of the data, an int
    int size = *(int *)data / sizeof(float);
    data += sizeof(int);

    //Set up the source and destination arrays
    float *src = (float *)data;
    float *dest;
    
    switch (t) {
    case OutputState:
        dest = outputs;
        break;
    default:
        return;
    }

    //Copy the values
    int i;
    for (i = 0; i < size; i++)
        dest[i] = src[i];

    //Post the event
	DataEvent<const RemoteState *> event(this, EventBase::remoteStateEGID,
										 t, EventBase::statusETID);
	
    erouter->postEvent(event);
}

