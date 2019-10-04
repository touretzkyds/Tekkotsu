#ifndef KODU_OBJECT_H_
#define KODU_OBJECT_H_

// Tekkodu Library
#include "Kodu/Primitives/PerceptionSearch.h"

// Tekkotsu Library
#include "DualCoding/ShapeRoot.h"
#include "DualCoding/ShapeAprilTag.h"

// C++ Library
#include <string>

namespace Kodu {
    //! Kodu Object class
    class KoduObject {
    public:
        //! Constructor
        KoduObject(const std::string& kColor, const std::string& kType, SearchLocation_t locationsToSearch,
            bool canBeLandmark);

        //! Copy constructor
        KoduObject(const KoduObject& kObject);

        //! Destructor
        ~KoduObject();

        //! Assignment operator
        KoduObject& operator=(const KoduObject& kObject);

        //! Returns the object's color
        const std::string& getColor() const;

        //! Returns the object's type
        const std::string& getType() const;

        //! Returns true if an object is a landmark or not
        bool isLandmark() const;

        //! Returns true if an object is movable
        bool isMovable() const;

        //! Returns true if an object can move throughout the environment (autonomously)
        bool hasMobility() const;

        //! Returns true if the agent found an object that met the search criteria (color, type, etc)
        bool foundValidMatch() const;

        //! Returns the object that matches the description
        const DualCoding::ShapeRoot& getObject() const;

        //! Returns the matching object's april tag
        const DualCoding::Shape<DualCoding::AprilTagData>& getObjectIdentifier() const;

        //! Stores the object that matches the search criteria
        void setObject(const DualCoding::ShapeRoot&);

        //! Stores the object's identifier
        void setObjectIdentifier(const DualCoding::Shape<DualCoding::AprilTagData>&);

    protected:
        std::string color;                  //!< the object's color
        std::string type;                   //!< the object's type (e.g. tree, rock, apple)
        SearchLocation_t searchLocations;   //!< the locations the perceiver should search for an object
        bool objectIsLandmark;              //!< can "this" object be used as a landmark

    public:
        DualCoding::ShapeRoot matchingObject; //!< the object that matches search criteria
        DualCoding::Shape<DualCoding::AprilTagData> matchingObjTag;
    };
}

#endif // end of KODU_OBJECT_H_
