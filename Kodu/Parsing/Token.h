#ifndef TOKEN_H_
#define TOKEN_H_

// C++ Library 
#include <string>

namespace Kodu {
// ===================== Token Class Definitions ========================== //
    class TokenBase {
    public:
        //! Constructor
        TokenBase() {
            // no explicit implementation
        }

        //! Copy constructor
        explicit TokenBase(const TokenBase& kToken) {
            // no explicit implementation
        }

        //! Destructor
        virtual ~TokenBase() {
            // no explicit implementation
        }

        //! Assignment operator
        TokenBase& operator=(const TokenBase& kToken) {
            return *this;
        }
        
        bool isKeywordToken() const;

        //! Returns whether or not an instance is a string token
        bool isStringToken() const;
        
        //! Returns whether or not an instance is a numeric token
        bool isNumericToken() const;
        
        const std::string& getKeywordData() const;

        //! Returns the string data of a StringToken instance
        const std::string& getStringData() const;
        
        //! Returns the numeric data of a NumericToken instance
        const float& getNumericData() const;
    };

    //! Token Super Class
    template<typename DataType> class Token;

#define TOKEN_SUPER_CLASS(DataType)                                 \
    template<>                                                      \
    class Token<DataType> : public TokenBase {                      \
    public:                                                         \
        explicit Token<DataType>(const DataType& kData)             \
          : TokenBase(),                                            \
            data(kData)                                             \
        { }                                                         \
                                                                    \
        explicit Token<DataType>(const Token<DataType>& kToken)     \
          : TokenBase(kToken),                                      \
            data(kToken.data)                                       \
        { }                                                         \
                                                                    \
        virtual ~Token<DataType>() { }                              \
                                                                    \
        Token<DataType>& operator=(const Token<DataType>& kToken) { \
            if (this != &kToken) {                                  \
                TokenBase::operator=(kToken);                       \
                data = kToken.data;                                 \
            }                                                       \
            return *this;                                           \
        }                                                           \
                                                                    \
        const DataType& getData() const {                           \
            return data;                                            \
        }                                                           \
                                                                    \
    protected:                                                      \
        DataType data;                                              \
    };


#define TOKEN_DERIVED_CLASS(DerivedClassName, SuperClassName, DataType)     \
    class DerivedClassName : public SuperClassName {                        \
    public:                                                                 \
        explicit DerivedClassName(const DataType& kData)                    \
          : SuperClassName(kData)                                           \
        { }                                                                 \
                                                                            \
        explicit DerivedClassName(const DerivedClassName& kToken)           \
          : SuperClassName(kToken)                                          \
        { }                                                                 \
                                                                            \
        ~DerivedClassName() { }                                             \
                                                                            \
        DerivedClassName& operator=(const DerivedClassName& kToken) {       \
            if (this != &kToken) {                                          \
                SuperClassName::operator=(kToken);                          \
            }                                                               \
            return *this;                                                   \
        }                                                                   \
    };
        
    // Specializations
    //! Token<std::string>
    TOKEN_SUPER_CLASS(std::string);

    //! Token<float>
    TOKEN_SUPER_CLASS(float);

    //! KeywordToken
    TOKEN_DERIVED_CLASS(KeywordToken, Token<std::string>, std::string);

    //! NumericToken
    TOKEN_DERIVED_CLASS(NumericToken, Token<float>, float);

    //! StringToken
    TOKEN_DERIVED_CLASS(StringToken, Token<std::string>, std::string);

// ======================================================================== //
}

#endif // TOKEN_H_