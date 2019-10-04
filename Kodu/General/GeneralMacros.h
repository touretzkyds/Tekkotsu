#ifndef GENERAL_MACROS_H_
#define GENERAL_MACROS_H_

#include <iostream>

//! A printing shortcut for boolean attributes
#define PRINT_ATTRS(AttrName, BooleanVal) \
    std::cout << AttrName << ": " << (BooleanVal ? "true" : "false") << std::endl;

//! Disallows the copy constructor
#define DISALLOW_COPY(TypeName) TypeName(const TypeName&)

//! Disallows the assignment operator
#define DISALLOW_ASSIGN(TypeName) TypeName& operator=(const TypeName&)

//! Disallows both the copy constructor and the assignment operator
#define DISALLOW_COPY_ASSIGN(TypeName)  \
            DISALLOW_COPY(TypeName);    \
            DISALLOW_ASSIGN(TypeName)

//! Disallows a class from being instantiated
#define DISALLOW_INSTANTIATION(TypeName)    \
            DISALLOW_COPY_ASSIGN(TypeName); \
            TypeName();                     \
            ~TypeName();

#endif // GENERAL_MACROS_H_