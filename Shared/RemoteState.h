#ifndef REMOTESTATE_H_
#define REMOTESTATE_H_

#include "Shared/RobotInfo.h"
#include <vector>
#include <iostream>

class RemoteRouter;

/*! This class represents remote state information recieved from a
 *  remote dog, and can be treated like a WorldState object */
class RemoteState {
    public:
    RemoteState(const RemoteRouter *p);
    virtual ~RemoteState();
    float outputs[NumOutputs];
	float buttons[NumButtons];
	float sensors[NumSensors];

    void update(char *data);
    
    enum StateType {
        OutputState,
		ButtonState,
		SensorState,
    };

    static const int sizes[];
    
    private:
    const RemoteRouter *parent;

    
    RemoteState(RemoteState&);
    RemoteState &operator=(const RemoteState&);
};

// no-op, just eat a pointer
inline std::istream& operator>>(std::istream& s, const RemoteState*&) { void * x; s>>x; return s; }

#endif /* REMOTESTATE_H_ */
