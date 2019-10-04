#ifndef SCORE_KEEPER_H_
#define SCORE_KEEPER_H_

// Tekkodu Library
#include "Kodu/General/GeneralFncs.h"
#include "Kodu/General/GeneralMacros.h"
#include "Kodu/Primitives/KoduActionScore.h"

// C++ Library
#include <iostream>
#include <string>

namespace Kodu {
    //! The timestamp
    typedef unsigned long Timestamp_t;
    
    // Forward declare the KoduActionScore class (will eliminate cyclic dependencies between
    // included files)
    class KoduActionScore;

    class ScoreChange {
    public:
        //! Constructor
        ScoreChange(KoduActionScore::ScoringType_t scoreOp, const std::string& kDesignator,
            unsigned int val)
          : operationType(scoreOp),
            designator(kDesignator),
            value(val),
            timeCreated(0)
        {
            timeCreated = GeneralFncs::getTime();
        }

        //! Copy constructor
        ScoreChange(const ScoreChange& kChange)
          : operationType(kChange.operationType),
            designator(kChange.designator),
            value(kChange.value),
            timeCreated(kChange.timeCreated)
        { }

        //! Destructor
        ~ScoreChange() {
            // no explicit implementation
        }

        //! Assignment operator
        ScoreChange& operator=(const ScoreChange& kChange) {
            if (this != &kChange) {
                operationType = kChange.operationType;
                designator = kChange.designator;
                value = kChange.value;
                timeCreated = kChange.timeCreated;
            }
            return *this;
        }

        Timestamp_t getTimestamp() const;

        //! Overloaded greater than operator
        bool operator>(const ScoreChange&);

        //! Overloaded less than operator
        bool operator<(const ScoreChange&);

        //! Overloaded equal to operator
        bool operator==(const ScoreChange&);

        //! Overloaded not equal to operator
        bool operator!=(const ScoreChange&);

        // The ScoreKeeper class can directly access all the members of this class
        friend class ScoreKeeper;
    
        KoduActionScore::ScoringType_t operationType; //!< The operation type (e.g. add, set, subtract)
        std::string designator;     //!< The score designator (color or score letter)
        unsigned int value;         //!< The value to set, substract, or add

    private:
        Timestamp_t timeCreated;    //!< The approximate time the score was created
    };

    //! Score Keeper
    class ScoreKeeper {
    public:
        //! Constructor
        ScoreKeeper()
          : scoreBoard()
        { }

        //! Destructor
        ~ScoreKeeper() {
            scoreBoard.clear();
        }

        //! Assignment operator
        ScoreKeeper& operator=(const ScoreKeeper& kKeeper) {
            if (this != &kKeeper) {
                scoreBoard = kKeeper.scoreBoard;
            }
            return *this;
        }

        //! Adds to a particular score
        int addScore(const std::string&, int);
        
        //! Checks the value of a particular score
        int checkScoreValue(const std::string&);

        //! (Re)Initializes the score board
        void initialize();

        //! Registers an entry for a new score based on the key provided
        bool registerScore(const std::string&);
        
        //! Checks if a particular score type (identified by color+letter key) exists
        bool scoreExists(const std::string&);

        //! Sets a particular score
        int setScore(const std::string&, int);
        
        //! Subtracts from a particular score
        int subtractScore(const std::string&, int);

    private:
        //! Disallows the copy constructor and assignment operator
        DISALLOW_COPY(ScoreKeeper);

        //! Contains all the global scores
        std::map<std::string, int> scoreBoard;
    };

} // end of Kodu namespace

#endif // SCORE_KEEPER_H_
