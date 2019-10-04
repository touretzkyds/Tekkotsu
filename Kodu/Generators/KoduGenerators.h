#ifndef KODU_GENERATORS_H_
#define KODU_GENERATORS_H_

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

namespace Kodu {
    
    class LiteralGenerator {
    public:
        //! A enumeration of ways a literal string can be returned
        enum ReturnOrder_t {
            RO_SEQUENTIAL = 0,
            RO_RANDOM
        };

        //! Constructor #1
        LiteralGenerator(const std::string& kLiteralString, ReturnOrder_t returnOrder)
          : literalStrings(),
            order(returnOrder),
            vecIndex(0)
        {
            if (!kLiteralString.empty())
                literalStrings.push_back(kLiteralString);
        }

        //! Constructor #2
        LiteralGenerator(const std::vector<std::string>& kLiteralStrVector, ReturnOrder_t returnOrder)
          : literalStrings(kLiteralStrVector),
            order(returnOrder),
            vecIndex(literalStrings.size() - 1)
        { }

        //! Copy constructor
        LiteralGenerator(const LiteralGenerator& kGenerator)
          : literalStrings(kGenerator.literalStrings),
            order(kGenerator.order),
            vecIndex(kGenerator.vecIndex)
        { }

        //! Destructor
        ~LiteralGenerator() {
            // no explicit implementation
        }

        //! Assignment operator
        LiteralGenerator& operator=(const LiteralGenerator& kGenerator) {
            if (this != &kGenerator) {
                literalStrings = kGenerator.literalStrings;
                order = kGenerator.order;
                vecIndex = kGenerator.vecIndex;
            }
            return *this;
        }

        //! Adds a literal string
        void addLiteralString(const std::string&);

        //! Returns a literal string
        const std::string& getLiteralString();

        //! Prints the attributes
        void printAttrs() const;

        //! Sets the literal string vector
        void setLiteralStrings(const std::vector<std::string>&);

    private:
        std::vector<std::string> literalStrings;    //!< A vector of literal strings
        ReturnOrder_t order;        //!< Used to control what order the strings will be returned in
        unsigned int vecIndex;      //!< Used to return the "next" string (mostly used for sequential order)
    };

    class NumericGenerator {
    public:
        //! Constructor
        NumericGenerator(float constantVal, float moduloDivisorVal)
          : constant(constantVal),
            moduloDivisor(moduloDivisorVal)
        {
            srand(time(NULL));
        }

        //! Copy constructor
        NumericGenerator(const NumericGenerator& kGenerator)
          : constant(kGenerator.constant),
            moduloDivisor(kGenerator.moduloDivisor)
        {
            srand(time(NULL));
        }

        //! Destructor
        virtual ~NumericGenerator() {
            // no explicit implementation
        }

        //! Assignment operator
        NumericGenerator& operator=(const NumericGenerator& kGenerator) {
            if (this != &kGenerator) {
                constant = kGenerator.constant;
                moduloDivisor = kGenerator.moduloDivisor;
                srand(time(NULL));
            }
            return *this;
        }

        //! Returns a constant or random value
        virtual float getNumericValue();

        //! Prints info about the attributes
        void printAttrs() const;

        //! Sets the constant and upper bound values
        void setNumericValues(float, float);

    private:
        float constant;         //!< The constant value that is added to modulo operation's result
        float moduloDivisor;    //!< The divisor in the modulo operation
    };
    
    // class TimerGenerator : public NumericGenerator {
    // public:
    //  //! Constructor
    //  TimerGenerator(float constantVal, float upperBoundVal)
    //    : NumericGenerator(constantVal, upperBoundVal)
    //  { }

    //  //! Copy constructor
    //  TimerGenerator(const TimerGenerator& kGenerator)
    //    : NumericGenerator(kGenerator)
    //  { }

    //  //! Destructor
    //  ~TimerGenerator() { /* no explicit implementation */ }

    //  //! Assignment operator
    //  TimerGenerator& operator=(const TimerGenerator& kGenerator) {
    //      if (this != &kGenerator) {
    //          NumericGenerator::operator=(kGenerator);
    //      }
    //      return *this;
    //  }

    //  //! Prints info about the attributes
    //  void printAttrs() const;
    // };
}

#endif // KODU_GENERATORS_H_