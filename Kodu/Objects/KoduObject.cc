#include "Kodu/Objects/KoduObject.h"

namespace Kodu {

    // constructor
    KoduObject::KoduObject(const std::string& kColor, const std::string& kType,
        SearchLocation_t locationsToSearch, bool canBeLandmark)
      : color(kColor),
        type(kType),
        searchLocations(locationsToSearch),
        objectIsLandmark(canBeLandmark),
        matchingObject(),
        matchingObjTag()
    { }

    // copy constructor
    KoduObject::KoduObject(const KoduObject& kObject)
      : color(kObject.color),
        type(kObject.type),
        searchLocations(kObject.searchLocations),
        objectIsLandmark(kObject.objectIsLandmark),
        matchingObject(kObject.matchingObject),
        matchingObjTag(kObject.matchingObjTag)
    { }

    // destructor
    KoduObject::~KoduObject() {
        // no explicit implementation
    }

    // assignment operator
    KoduObject& KoduObject::operator=(const KoduObject& kObject) {
        if (this != &kObject) {
            color = kObject.color;
            type = kObject.type;
            searchLocations = kObject.searchLocations;
            objectIsLandmark = kObject.objectIsLandmark;
            matchingObject = kObject.matchingObject;
            matchingObjTag = kObject.matchingObjTag;
        }
        return *this;
    }

    const std::string& KoduObject::getColor() const {
        return color;
    }

    const std::string& KoduObject::getType() const {
        return type;
    }

    bool KoduObject::isLandmark() const {
        return objectIsLandmark;
    }

    bool KoduObject::isMovable() const {
        // landmark objects should not be movable
        //return (!isLandmark());
        return false;
    }

    bool KoduObject::foundValidMatch() const {
        return matchingObject.isValid();
    }

    const DualCoding::ShapeRoot& KoduObject::getObject() const {
        return matchingObject;
    }

    const DualCoding::Shape<DualCoding::AprilTagData>& KoduObject::getObjectIdentifier() const {
        return matchingObjTag;
    }

    void KoduObject::setObject(const DualCoding::ShapeRoot& kShape) {
        matchingObject = kShape;
    }

    void KoduObject::setObjectIdentifier(const DualCoding::Shape<DualCoding::AprilTagData>& kIdentifier) {
        matchingObjTag = kIdentifier;
    }
}
