#include "Kodu/Keepers/ScoreKeeper.h"

namespace Kodu {

    Timestamp_t ScoreChange::getTimestamp() const {
        return timeCreated;
    }

    bool ScoreChange::operator>(const ScoreChange& rhs) {
        return (timeCreated > rhs.timeCreated);
    }

    bool ScoreChange::operator<(const ScoreChange& rhs) {
        return (timeCreated < rhs.timeCreated);
    }

    bool ScoreChange::operator==(const ScoreChange& rhs) {
        return (timeCreated == rhs.timeCreated);
    }

    bool ScoreChange::operator!=(const ScoreChange& rhs) {
        return (!(*this == rhs));
    }
    
    int ScoreKeeper::addScore(const std::string& scoreKey, int value) {
        if (!scoreExists(scoreKey)) {
            registerScore(scoreKey);
        }
        scoreBoard[scoreKey] += value;
        // TODO (20/JUL/13) how to report a value in stdout with color using the KoduColor class
        std::cout << "ScoreKeeper: Score {" << scoreKey << "} = "
                  << scoreBoard[scoreKey] << std::endl;
        return scoreBoard[scoreKey];
    }

    int ScoreKeeper::checkScoreValue(const std::string& scoreKey) {
        if (!scoreExists(scoreKey)) {
            registerScore(scoreKey);
        }
        return scoreBoard[scoreKey];
    }

    void ScoreKeeper::initialize() {
        std::cout << "Initializing score board...\n";
        scoreBoard.clear();
    }

    bool ScoreKeeper::registerScore(const std::string& scoreKey) {
        if (!scoreExists(scoreKey)) {
            scoreBoard.insert(std::pair<std::string,int>(scoreKey, 0));
            std::cout << "ScoreKeeper: Registering score with key {"
                      << scoreKey << "}.\n";
        }
        return scoreExists(scoreKey);
    }
    
    bool ScoreKeeper::scoreExists(const std::string& scoreKey) {
        return (scoreBoard.count(scoreKey) > 0);
    }

    int ScoreKeeper::setScore(const std::string& scoreKey, int value) {
        if (!scoreExists(scoreKey)) {
            registerScore(scoreKey);
        }
        scoreBoard[scoreKey] = value;
        // TODO (20/JUL/13) how to report a value in stdout with color using the KoduColor class
        std::cout << "ScoreKeeper: Score {" << scoreKey << "} = "
                  << scoreBoard[scoreKey] << std::endl;
        return scoreBoard[scoreKey];
    }

    int ScoreKeeper::subtractScore(const std::string& scoreKey, int value) {
        if (!scoreExists(scoreKey)) {
            registerScore(scoreKey);
        }
        scoreBoard[scoreKey] -= value;
        // TODO (20/JUL/13) how to report a value in stdout with color using the KoduColor class
        std::cout << "ScoreKeeper: Score {" << scoreKey << "} = "
                  << scoreBoard[scoreKey] << std::endl;
        return scoreBoard[scoreKey];
    }

}
