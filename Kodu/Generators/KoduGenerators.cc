#include "Kodu/Generators/KoduGenerators.h"

namespace Kodu {
    
    const std::string& LiteralGenerator::getLiteralString() {
        switch (order) {
            case RO_SEQUENTIAL:
                vecIndex = (vecIndex + 1) % literalStrings.size();
                break;

            case RO_RANDOM:
                vecIndex = rand() % literalStrings.size();
                break;
        }
        return literalStrings[vecIndex];
    }

    void LiteralGenerator::printAttrs() const {
        std::cout << "Literal Generator:\n";
        std::cout << "Literal generator string count: " << literalStrings.size() << std::endl;
        std::cout << "Spoken order: ";
        switch (order) {
            case RO_SEQUENTIAL:
                std::cout << "sequential";
                break;

            case RO_RANDOM:
                std::cout << "random";
                break;
        }
        std::cout << std::endl;
        for (std::size_t i = 0; i < literalStrings.size(); i++)
            std::cout << "\tString [" << i << "]: " << literalStrings[i] << std::endl;
    }

    void LiteralGenerator::setLiteralStrings(const std::vector<std::string>& kLiteralStrings) {
        literalStrings = kLiteralStrings;
        vecIndex = literalStrings.size() - 1;
    }

    float NumericGenerator::getNumericValue() {
        if (moduloDivisor >= 2.0f) {
            float randVal = static_cast<float>(rand() % static_cast<int>(moduloDivisor));
            return (randVal + constant);
        } else {
            return constant;
        }
    }

    void NumericGenerator::printAttrs() const {
        std::cout << "Numeric Generator:\n";
        std::cout << "Constant: " << constant << std::endl;
        std::cout << "Upper bound: " << moduloDivisor << std::endl;
        std::cout << "Performing random numeric selection: " << (moduloDivisor >= 2 ? "yes" : "no") << std::endl;
    }

    void NumericGenerator::setNumericValues(float constantVal, float upperBoundVal) {
        constant = constantVal;
        moduloDivisor = upperBoundVal;
    }
    
    /*
    float TimerGenerator::getNumericValue() {
        if (moduloDivisor >= 2.0f) {
            int randIntVal = (rand() % (int)(moduloDivisor * 4));
            return (constant + ((float)(randIntVal) / 4.0f));
        } else {
            return constant;
        }
    }

    void TimerGenerator::printAttrs() const {
        std::cout << "Numeric Generator:\n";
        std::cout << "Constant: " << constant << std::endl;
        std::cout << "Upper bound: " << moduloDivisor << std::endl;
        std::cout << "Performing random numeric selection: " << (moduloDivisor >= 2 ? "yes" : "no") << std::endl;
    }
    */
}