#ifndef POS_ORIENT_STATE_H_
#define POS_ORIENT_STATE_H_

#include "DualCoding/Point.h"

namespace Kodu {

    class PosOrientState {
    public:
        //! Default constructor
        PosOrientState();

        //! Constructor #2
        PosOrientState(const DualCoding::Point&, const float&);

        //! Copy constructor
        PosOrientState(const PosOrientState&);

        //! Destructor
        ~PosOrientState();

        //! Assignment operator
        PosOrientState& operator=(const PosOrientState&);

        //! Difference operator
        PosOrientState operator-(const PosOrientState&) const;

        //! Equal operator
        bool operator==(const PosOrientState&) const;

        //! Not equal operator
        bool operator!=(const PosOrientState&) const;

        DualCoding::Point position;     //!< Position of the robot
        float orientation;              //!< Orientation of the robot
    };
}

#endif // POS_ORIENT_STATE_H_
