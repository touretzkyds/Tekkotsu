#ifndef KODU_ACTION_MOTION_H_
#define KODU_ACTION_MOTION_H_

// INCLUDES
// c++
#include <cmath>

// tekkotsu
#include "DualCoding/BaseData.h"
#include "DualCoding/DualCoding.h"
#include "DualCoding/ShapeRoot.h"

// tekkodu
#include "Kodu/Primitives/KoduAction.h"
#include "Kodu/Primitives/PerceptionSearch.h"
#include "Kodu/Generators/KoduGenerators.h"
#include "Kodu/Keepers/ObjectKeeper.h"

namespace Kodu {

    class MotionCommand {
    public:
        //! Constructor #1
        MotionCommand();

        //! Constructor #2
        MotionCommand(const ShapeRoot& kTarget, float fwdSpeed, float trnSpeed);

        //! Constructor #3
        MotionCommand(float fwdDist, float trnAngle, float fwdSpeed, float trnSpeed);

        //! Constructor #4
        MotionCommand(const ShapeRoot& kTarget, float fwdDist, float trnAngle,
            float fwdSpeed, float trnSpeed);
				//! Constructor #5
        MotionCommand(float fwdDist, float trnAngle, float fwdSpeed, float trnSpeed, bool driven);

        //! Copy Constructor
        MotionCommand(const MotionCommand&);

        //! Destructor
        ~MotionCommand();

        //! Assignment operator
        MotionCommand& operator=(const MotionCommand&);

        //! Equals operator
        bool operator==(const MotionCommand&);

        //! Not equals operator
        bool operator!=(const MotionCommand&);

        //! Returns the forward speed
        float getForwardSpeed() const;

        //! Returns the turning speed
        float getTurnSpeed() const;

        //! Returns the distance to travel in the x-direction
        float getDistanceToTravel() const;

        //! Returns the angle to turn (in degrees)
        float getTurningAngle() const;

        //! True if this is a feasible motion command
        bool isValid() const;

        //! Returns if the shape reference is valid or not
        bool targetObjectIsValid() const;
	
				bool isDriven() const;

        //! Returns the target object (if available)
        const ShapeRoot& getTargetObject() const;

        friend class KoduActionMotion;
				friend std::ostream& operator<<(std::ostream &os, const MotionCommand &cmd);

    private:
        ShapeRoot targetObject;
        float dx;           //!< How far the robot should move forward (units: mm)
        float da;           //!< How much the robot should turn (units: rads)
        float forwardSpeed; //!< How fast the robot should move forward (units: mm/s)
        float turnSpeed;    //!< How fast the robot should turn (units: rads/s)
        bool cmdDriven;
    };

		std::ostream& operator<<(std::ostream &os, const MotionCommand &cmd);

    //! Kodu Action Motion (derived from Kodu Action)
    class KoduActionMotion : public KoduAction {
    public:
        //! The different types of move and turn movements
        enum MotionType_t {
            MT_MOVE_WANDER          = 1L << 0,
            MT_MOVE_FORWARD         = 1L << 1,
            MT_MOVE_TOWARDS         = 1L << 2,
            MT_EMPTY_MOTION_TYPE    = 1L << 3,  // place holder
            MT_TURN_DIRECTION       = 1L << 4,
            MT_TURN_LEFT            = 1L << 5,
            MT_TURN_RIGHT           = 1L << 6,
            MT_MOVE_GAMEPAD         = 1L << 7
        };

        //! Helps set the speed of a particular motion type
        enum MotionRate_t {
            MR_NORMAL               = 1L << 0,
            MR_QUICKLY              = 1L << 1,
            MR_SLOWLY               = 1L << 2
        };

        //! Constructor #1
        KoduActionMotion(MotionType_t type, MotionRate_t rate, unsigned int motionMagCount);

        //! Constructor #2
        KoduActionMotion(Direction_t direction, MotionRate_t rate, unsigned int motionMagCount);

        //! Copy constructor
        KoduActionMotion(const KoduActionMotion& kAction);

        //! Destructor
        ~KoduActionMotion();

        //! Assignment operator
        KoduActionMotion& operator=(const KoduActionMotion& kAction);

        //! Returns the motion command
        const MotionCommand getMotionCommand();

        //! Returns the motion type
        MotionType_t getMotionType() const;

        //! Returns true if the motion type (action) is "move"
        bool motionTypeIsMove() const;

        //! Returns true if the motion type (action) is "turn"
        bool motionTypeIsTurn() const;

        //! Used to reinitialize certain variables (e.g. when switching to another page)
        virtual void reinitialize();

    private:
        //! States what type of movement the robot will do
        MotionType_t motionType;

        //! Used to generate constant or random angle values
        NumericGenerator angleGen;

        //! Used to generate constant or random distance values
        NumericGenerator distGen;

        //! The direction the robot wants to face (north, west, etc.)
        Direction_t directionToFace;
    };
}

#endif // KODU_ACTION_MOTION_H_
