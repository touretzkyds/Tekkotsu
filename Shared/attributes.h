#ifndef INCLUDED_attributes_h
#define INCLUDED_attributes_h

#if defined(__GNUC__) && __GNUC__ >= 3

#  if __GNUC__ > 3 || __GNUC__ == 3 && (__GNUC_MINOR__ > 4)
// Currently no 3.5 branch exists, so this boils down to 4.0 or better
//! indicates that a warning should be given if caller doesn't use the return value, e.g. newly allocated memory regions
#    define ATTR_must_check	__attribute__ ((warn_unused_result))
#  else
// 3.3 doesn't even support warn_unused_result, and 3.4.2 generates spurious errors.
//! indicates that a warning should be given if caller doesn't use the return value, e.g. newly allocated memory regions
#    define ATTR_must_check	/* no warn_unused_result */
#  endif

//! triggers inlining even when no optimization is specified
#  define ATTR_always_inline		__attribute__ ((always_inline))

//! compiler hint that the function has no side effects and return value depends only on arguments and non-volatile globals
#  define ATTR_pure		__attribute__ ((pure))

//! like #ATTR_pure, compiler hint that the function has no side effects and return value depends only on arguments -- unlike #ATTR_pure, cannot access globals or dereference pointer arguments.
#  define ATTR_const	__attribute__ ((const))

//! indicates that the function is 'fatal' and will not return; however, can still throw an exception!
#  define ATTR_noreturn	__attribute__ ((noreturn))

//! compiler hint that a non-NULL pointer return value is guaranteed to be a unique address (e.g. malloc)
#  define ATTR_malloc	__attribute__ ((malloc))

//! triggers a warning if the function/variable is referenced by any code which is not itself marked with #ATTR_deprecated
#  define ATTR_deprecated	__attribute__ ((deprecated))

//! forces the code for the function to be emitted even if nothing appears to reference it -- handy when called by inline assembly
#  define ATTR_used		__attribute__ ((used))

//! indicates no warning should be given if the specified value goes unused, e.g. fulfilling an interface which requires superfluous arguments
#  define ATTR_unused	__attribute__ ((unused))

//! requests that members of a struct or union be layed out a densely as possible to minimize memory usage; applied to enums requests smallest storage type be used
#  define ATTR_packed	__attribute__ ((packed))

//! should be passed a value within an 'if' statement to hint that the value is likely to be 'true' (i.e. non-zero)
#  define ATTR_likely(x)	__builtin_expect (!!(x), 1)

//! should be passed a value within an 'if' statement to hint that the value is unlikely to be 'true' (i.e. likely to be 'false' or 0)
#  define ATTR_unlikely(x)	__builtin_expect (!!(x), 0)

#else

//! triggers inlining even when no optimization is specified
#  define ATTR_always_inline		/* no always_inline */

//! compiler hint that the function has no side effects and return value depends only on arguments and non-volatile globals
#  define ATTR_pure		/* no pure */

//! like #ATTR_pure, compiler hint that the function has no side effects and return value depends only on arguments -- unlike #ATTR_pure, cannot access globals or dereference pointer arguments.
#  define ATTR_const	/* no const */

//! indicates that the function is 'fatal' and will not return; however, can still throw an exception!
#  define ATTR_noreturn	/* no noreturn */

//! compiler hint that a non-NULL pointer return value is guaranteed to be a unique address (e.g. malloc)
#  define ATTR_malloc	/* no malloc */

//! indicates that a warning should be given if caller doesn't use the return value, e.g. newly allocated memory regions
#  define ATTR_must_check	/* no warn_unused_result */

//! triggers a warning if the function/variable is referenced by any code which is not itself marked with #ATTR_deprecated
#  define ATTR_deprecated	/* no deprecated */

//! forces the code for the function to be emitted even if nothing appears to reference it -- handy when called by inline assembly
#  define ATTR_used		/* no used */

//! indicates no warning should be given if the specified value goes unused, e.g. fulfilling an interface which requires superfluous arguments
#  define ATTR_unused	/* no unused */

//! requests that members of a struct or union be layed out a densely as possible to minimize memory usage; applied to enums requests smallest storage type be used
#  define ATTR_packed	/* no packed */

//! should be passed a value within an 'if' statement to hint that the value is likely to be 'true' (i.e. non-zero)
#  define ATTR_likely(x)	(x)

//! should be passed a value within an 'if' statement to hint that the value is unlikely to be 'true' (i.e. likely to be 'false' or 0)
#  define ATTR_unlikely(x)	(x)

#endif

#ifdef _MSC_VER
#  ifdef BUILDING_DLL
#    define EXPORT_SYMBOL __declspec(dllexport)
#  else
#    define EXPORT_SYMBOL __declspec(dllimport)
#  endif
#  define LOCAL_SYMBOL
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define EXPORT_SYMBOL __attribute__ ((visibility("default")))
#  define LOCAL_SYMBOL __attribute__ ((visibility("hidden")))
#else
#  define EXPORT_SYMBOL
#  define LOCAL_SYMBOL
#endif

/*! @file
* @brief Defines variants of the __attribute__ macros to provide better portability
* @author Robert Love (Creator), ejt (more version specificity)
*
* Based on code published at http://rlove.org/log/2005102601
*/
#endif
