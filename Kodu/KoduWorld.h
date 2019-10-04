#ifndef KODU_WORLD_H_
#define KODU_WORLD_H_

// INCLUDES
// c++
#include <map>
#include <queue>
#include <vector>

// tekkotsu
#include "DualCoding/PolygonData.h"
#include "DualCoding/ShapeAprilTag.h"
#include "DualCoding/ShapePolygon.h"
#include "DualCoding/ShapeRoot.h"
using namespace DualCoding;

// tekkodu
#include "Kodu/KoduAgent.h"
#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Keepers/ScoreKeeper.h"

namespace Kodu {

    class KoduWorld {
    public:
        //! Constructor
        KoduWorld();

        //! Destructor
        ~KoduWorld();

        /// ================================ Scoring functions ================================ ///
        //! Applies the scores changes to the global score keeper
        void applyGlobalScoreChanges(std::queue<ScoreChange>&);

        //! Returns the value of a particular score (identified by the designator)
        static int getScoreValue(const std::string&);

        /// ================================ Shape-Tag Pair functions ========================= ///
        //! Returns the april tag id corresponding to a particular shape id
        int getTagIdForShape(int);

        //! Checks if a shapt-tag pair exists
        bool shapeTagPairExists(int) const;

        //! Pairs an april tag with a particular shape
        void pairShapeWithTag(int, int);

        /// ================================ World Bounds and North Star functions =========== ///
        //! Returns the world North Star
        const Point& getNorthStarLocation() const;

        //! Returns the "stars" seen and their location
        const std::map<int, Point>& getStarConstellation() const;

        //! Returns the world bounds polygon
        const Shape<PolygonData>& getWorldBoundsPolygon() const;

        //! Creates the world bounds polygon
        void generateWorldBoundsPolygon();

        //! Sets the "stars" seen and their allocentric locations
        void setStarConstellation(const std::vector<ShapeRoot>&);

    private:
        //! Disallows the copy constructor and assignment operator
        DISALLOW_COPY_ASSIGN(KoduWorld);

    public:
        //! Represents the agent that the Kodu Behavior is running on
        KoduAgent thisAgent;

    private:
        //! Local copy of the Score Keeper
        static ScoreKeeper globalScoreKeeper;

        //! Helps the robot discern between multiple objects of same type and color when near them
        std::map<int, int> shapeToTagMap;

        //! A map of the april tag (used as stars) and their allocentric positions
        std::map<int, Point> starConstellation;

        //! The world bounds polygon
        Shape<PolygonData> worldBoundsPolygon;

        //! The length of the world's sides; all sides are the same length making the world a square
        const float worldSideLength;
    };
}

#endif // end of KODU_WORLD_H_
