#ifndef INCLUDED_fmatCore_h_
#define INCLUDED_fmatCore_h_

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <limits>
#include "attributes.h"
#include <stdio.h>
#ifdef __APPLE__
#  include <AvailabilityMacros.h>
#endif

//! fixed-size matrix routines for high performance with small allocations
namespace fmat {
	
#ifdef FMAT_DEFAULT_DOUBLE
	typedef double fmatReal;
#else
	typedef float fmatReal;
#endif
	
	struct Length_mismatch_or_non_vector {};
	struct NonSquare_Matrix {};
	struct SubVector_is_longer_than_source {};
	struct SubMatrix_has_more_rows_than_source {};
	struct SubMatrix_has_more_columns_than_source {};
	struct Rotation_requires_larger_dimensions {};
	struct Matrix_multiplication_left_width_must_match_right_height {};
	
	template<size_t N, typename R> class SubVector;
	template<size_t H, size_t W, typename T> class SubMatrix;
	template<size_t H, size_t W, typename T> class Matrix;
	template<size_t N, typename R> class Column;
	template<size_t N, typename R> class Row;
	template<typename R> inline Column<2,R> packT(R x, R y);
	template<typename R> inline Column<3,R> packT(R x, R y, R z);
	template<typename R> inline Column<4,R> packT(R x, R y, R z, R d);
	inline Column<2,fmatReal> pack(fmatReal x, fmatReal y);
	inline Column<3,fmatReal> pack(fmatReal x, fmatReal y, fmatReal z);
	inline Column<4,fmatReal> pack(fmatReal x, fmatReal y, fmatReal z, fmatReal d);
	
	extern std::string defaultNumberFormat;
	
	template<typename R>
	std::string fullfmt(const R* const data, size_t H, size_t W,
		std::string const &numberFormat,
		std::string const &firstLineStart, std::string const &nextLineStart, std::string const &lastLineEnd,
		std::string const &rowBegin, std::string const &elementSep, std::string const &rowEnd,
		std::string const &rowSep)
	{
		std::ostringstream os;
		char buf[100];
		os << firstLineStart;
		for (size_t r=0; r<H; r++) {
			if ( r > 0 )
				os << nextLineStart;
			os << rowBegin;
			for (size_t c=0; c<W; c++) {
				snprintf(buf, 100, numberFormat.c_str(), data[c*H+r]);
				os << buf;
				if ( c < (W-1) )
					os << elementSep;
			}
			os << rowEnd;;
			if ( r < (H-1) )
				os << rowSep;
		}
		os << lastLineEnd;
		return os.str();
	}
	
	namespace fmat_internal {
		template<typename T> inline void tmemcpy(T* dst, const T* src, size_t n) { memcpy(dst,src,n*sizeof(T)); }
		
#if defined(__APPLE__) && defined(MAC_OS_X_VERSION_10_5)
		// memset_patternX() is only available on OS X 10.5 (and later?)
		template<typename T, size_t W> struct tmemset_pattern {};
		template<typename T> struct tmemset_pattern<T,1> { inline tmemset_pattern(T* dst, const T& src, size_t n) { memset(dst,src,n*sizeof(T)); } };
		template<typename T> struct tmemset_pattern<T,4> { inline tmemset_pattern(T* dst, const T& src, size_t n) { memset_pattern4(dst,&src,n*sizeof(T)); } };
		template<typename T> struct tmemset_pattern<T,8> { inline tmemset_pattern(T* dst, const T& src, size_t n) { memset_pattern8(dst,&src,n*sizeof(T)); } };
		template<typename T> struct tmemset_pattern<T,16> { inline tmemset_pattern(T* dst, const T& src, size_t n) { memset_pattern16(dst,&src,n*sizeof(T)); } };
#else
		template<typename T, size_t W> struct tmemset_pattern {
			inline tmemset_pattern(T* dst, const T& src, size_t n) { while(n!=0) dst[--n]=src; }
		};
		template<typename T> struct tmemset_pattern<T,1> { inline tmemset_pattern(T* dst, const T& src, size_t n) { memset(dst,src,n*sizeof(T)); } };
#endif
		
		template<typename T> void tmemset(T* dst, const T& src, size_t n) { tmemset_pattern<T,sizeof(T)>(dst,src,n); }

		template<class MSG,bool X> class CompileTimeAssert;
		template<class MSG> class CompileTimeAssert<MSG,true> {};
		
		struct NoInit { };
		
		template<typename T> struct precision_trait { };
		template<> struct precision_trait<bool> { static const int PRECISION_RANK = 0; };
		template<> struct precision_trait<char> { static const int PRECISION_RANK = 100; };
		template<> struct precision_trait<unsigned char> { static const int PRECISION_RANK = 200; };
		template<> struct precision_trait<short> { static const int PRECISION_RANK = 300; };
		template<> struct precision_trait<unsigned short> { static const int PRECISION_RANK = 400; };
		template<> struct precision_trait<int> { static const int PRECISION_RANK = 500; };
		template<> struct precision_trait<unsigned int> { static const int PRECISION_RANK = 600; };
		template<> struct precision_trait<long> { static const int PRECISION_RANK = 700; };
		template<> struct precision_trait<unsigned long> { static const int PRECISION_RANK = 800; };
		template<> struct precision_trait<float> { static const int PRECISION_RANK = 900; };
		template<> struct precision_trait<double> { static const int PRECISION_RANK = 1000; };
		template<> struct precision_trait<long double> { static const int PRECISION_RANK = 1100; };
		
		template<typename T> struct unconst { typedef T type; };
		template<> struct unconst<const bool> { typedef bool type; };
		template<> struct unconst<const char> { typedef char type; };
		template<> struct unconst<const unsigned char> { typedef unsigned char type; };
		template<> struct unconst<const short> { typedef short type; };
		template<> struct unconst<const unsigned short> { typedef unsigned short type; };
		template<> struct unconst<const int> { typedef int type; };
		template<> struct unconst<const unsigned int> { typedef unsigned int type; };
		template<> struct unconst<const long> { typedef long type; };
		template<> struct unconst<const unsigned long> { typedef unsigned long type; };
		template<> struct unconst<const float> { typedef float type; };
		template<> struct unconst<const double> { typedef double type; };
		template<> struct unconst<const long double> { typedef long double type; };
		
		template<typename T1, typename T2, bool promoteT1>
		struct do_promotion { typedef T1 type; };
		template<typename T1, typename T2>
		struct do_promotion<T1,T2,false> { typedef T2 type; };
		
		template<typename T1, typename T2>
		struct promotion_trait {
			typedef typename unconst<T1>::type mT1;
			typedef typename unconst<T2>::type mT2;
			static const bool USET1 = (precision_trait<mT1>::PRECISION_RANK) > (precision_trait<mT2>::PRECISION_RANK);
			typedef typename do_promotion<mT1,mT2,USET1>::type type;
		};

		template<class T1, class T2>
		struct mmops {
			typedef typename T1::storage_t R1;
			typedef typename T2::storage_t R2;
			typedef typename fmat_internal::promotion_trait<R1,R2>::type R;
			static Matrix<T1::HEIGHT,T2::WIDTH,R> multiply(const T1& a, const T2& b) {
				Matrix<T1::HEIGHT,T2::WIDTH,R> ans=fmat_internal::NoInit();
				for(size_t c2=T2::WIDTH; c2--!=0;) {
					for(size_t r1=T1::HEIGHT; r1--!=0;) {
						R x=0;
						for(size_t r2=T1::WIDTH; r2--!=0;)
							x+=a(r1,r2)*b(r2,c2);
						ans(r1,c2)=x;
					}
				}
				return ans;
			}
		};
	
		template<class M, class V>
		struct mvops {
			typedef typename M::storage_t MR;
			typedef typename V::storage_t VR;
			typedef typename fmat_internal::promotion_trait<MR,VR>::type R;
			enum {
				H=M::HEIGHT,
				N=V::CAP,
				W=M::WIDTH
			};
			static Column<H,R> multiply(const M& a, const V& b) {
				(void)sizeof(fmat_internal::CompileTimeAssert<Matrix_multiplication_left_width_must_match_right_height,M::WIDTH==V::HEIGHT>);
				Column<H,R> ans=fmat_internal::NoInit();
				for(size_t r=H; r--!=0;) {
					R x=0;
					for(size_t c=N; c--!=0;)
						x+=a(r,c)*b[c];
					ans[r]=x;
				}
				return ans;
			}
			static Row<W,R> multiply(const V& a, const M& b) {
				(void)sizeof(fmat_internal::CompileTimeAssert<Matrix_multiplication_left_width_must_match_right_height,V::WIDTH==M::HEIGHT>);
				Row<W,R> ans=fmat_internal::NoInit();
				for(size_t c=W; c--!=0;) {
					R x=0;
					for(size_t r=N; r--!=0;)
						x+=b[r]*a(r,c);
					ans[c]=x;
				}
				return ans;
			}
		};
	}
	
	template<size_t N, typename R=fmatReal>
	class SubVector {
		template<size_t S, typename T> friend class SubVector;
		template<size_t S, typename T> friend class Column;
		template<size_t S, typename T> friend class Row;
		template<size_t H, size_t W, typename T> friend class SubMatrix;
		template<size_t H, size_t W, typename T> friend class Matrix;
	public:
		// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if defined(__clang__) || !defined(__GNUC__) || (__GNUC__==4 && __GNUC_MINOR__<1)
		static const size_t HEIGHT=N;
		static const size_t WIDTH=1;
		static const size_t CAP=N;
#else
		static const size_t HEIGHT;
		static const size_t WIDTH;
		static const size_t CAP;
#endif
		typedef R storage_t;
		typedef typename fmat_internal::unconst<R>::type out_t;
		
		explicit SubVector(R* x) : data(x) {}
		// allow implicit casts for same size
		SubVector(const SubVector& x) : data(x.data) {}
		SubVector(Matrix<N,1,out_t>& x) : data(x.data) {}
		SubVector(Matrix<1,(N==1?0U:N),out_t>& x) : data(x.data) {} // funny template to avoid duplicate constructor for <1,1>
		template<typename T> SubVector(const Matrix<N,1,T>& x) : data(x.data) {}
		template<typename T> SubVector(const Matrix<1,(N==1?0U:N),T>& x) : data(x.data) {} // funny template to avoid duplicate constructor for <1,1>

		//! this constructor handles shrinking/slicing from other SubVectors
		/*! If @a x has const data and this is non-const, we rely on an error on the #data initializer to point that out */
		template<size_t S, typename T> explicit SubVector(const SubVector<S,T>& x, size_t offset=0) : data(&x[offset]) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubVector_is_longer_than_source,(N<=S)>);
		}
		
		//! this constructor handles shrinking/slicing from non-const rows
		template<size_t S> explicit SubVector(Matrix<1,S,out_t>& x, size_t offset=0) : data(&x.data[offset]) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubVector_is_longer_than_source,(N<=S)>);
		}
		//! this constructor handles shrinking/slicing from non-const columns
		template<size_t S> explicit SubVector(Matrix<S,1,out_t>& x, size_t offset=0) : data(&x.data[offset]) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubVector_is_longer_than_source,(N<=S)>);
		}
		//! this constructor handles shrinking/slicing from const rows (which can only be used if R is const too - error on #data init otherwise)
		template<size_t S, typename T> explicit SubVector(const Matrix<1,S,T>& x, size_t offset=0) : data(&x.data[offset]) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubVector_is_longer_than_source,(N<=S)>);
		}
		//! this constructor handles shrinking/slicing from const columns (which can only be used if R is const too - error on #data init otherwise)
		template<size_t S, typename T> explicit SubVector(const Matrix<S,1,T>& x, size_t offset=0) : data(&x.data[offset]) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubVector_is_longer_than_source,(N<=S)>);
		}
	
		operator SubVector<N,const R>() const { return SubVector<N,const R>(data); }
	
		template<typename T> SubVector& importFrom(const T& x) { size_t i=N; while(i!=0) { --i; data[i]=x[i]; } return *this; }
		template<typename T> T exportTo() const { T x; size_t i=N; while(i!=0) { --i; x[i]=data[i]; } return x; }
		template<typename T> T& exportTo(T& x) const { size_t i=N; while(i!=0) { --i; x[i]=data[i]; } return x; }
	
		const SubVector& operator=(R x) const { fmat_internal::tmemset<R>(data,x,N); return *this; }
		const SubVector& operator=(const R* x) const { fmat_internal::tmemcpy<R>(data,x,N); return *this; }
		const SubVector& operator=(const Matrix<N,1,out_t>& x) const { fmat_internal::tmemcpy<R>(data,x.data,N); return *this; }
		const SubVector& operator=(const SubVector& x) const { fmat_internal::tmemcpy<R>(data,x.data,N); return *this; }
		template<typename T> const SubVector& operator=(const SubVector<N,T>& x) const { fmat_internal::tmemcpy<R>(data,x.data,N); return *this; }
		
		const SubVector& operator+=(R x) const { size_t i=N; while(i!=0) data[--i]+=x; return *this; }
		const SubVector& operator-=(R x) const { size_t i=N; while(i!=0) data[--i]-=x; return *this; }
		const SubVector& operator*=(R x) const { size_t i=N; while(i!=0) data[--i]*=x; return *this; }
		const SubVector& operator/=(R x) const { x=1/x; size_t i=N; while(i!=0) data[--i]*=x; return *this; }
		Column<N,out_t> operator+(R x) const { return Column<N,out_t>(*this)+=x; }
		Column<N,out_t> operator-(R x) const { return Column<N,out_t>(*this)-=x; }
		Column<N,out_t> operator*(R x) const { return Column<N,out_t>(*this)*=x; }
		Column<N,out_t> operator/(R x) const { return Column<N,out_t>(*this)/=x; }
		Column<N,out_t> operator-() const { Column<N,out_t> ans(*this); size_t i=N; while(i--!=0) ans.data[i]=-ans.data[i]; return ans; }
		
		friend Column<N,out_t> operator+(R x, const SubVector& a) { return a+x; }
		friend Column<N,out_t> operator-(R x, const SubVector& a) { return (-a)+x; }
		friend Column<N,out_t> operator*(R x, const SubVector& a) { return a*x; }
		
		template<typename T> const SubVector& operator+=(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; data[i]+=x.data[i]; } return *this; }
		template<typename T> const SubVector& operator-=(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; data[i]-=x.data[i]; } return *this; }
		template<typename T> const SubVector& operator+=(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; data[i]+=x.data[i]; } return *this; }
		template<typename T> const SubVector& operator-=(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; data[i]-=x.data[i]; } return *this; }
		template<typename T> Column<N,out_t> operator+(const Matrix<N,1,T>& x) const { return Column<N,out_t>(*this)+=x; }
		template<typename T> Column<N,out_t> operator-(const Matrix<N,1,T>& x) const { return Column<N,out_t>(*this)-=x; }
		template<typename T> Column<N,out_t> operator+(const SubVector<N,T>& x) const { return Column<N,out_t>(*this)+=x; }
		template<typename T> Column<N,out_t> operator-(const SubVector<N,T>& x) const { return Column<N,out_t>(*this)-=x; }
	
		template<typename T> bool operator==(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		template<typename T> bool operator!=(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		template<typename T> bool operator<(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }
		template<typename T> bool operator==(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		template<typename T> bool operator!=(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		template<typename T> bool operator<(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }
	
		//! returns true if all elements are less than the corresponding element
		template<typename T> bool operator<<(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; if(x.data[i] <= data[i]) return false; } return true; }
		template<typename T> bool operator<<(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; if(x.data[i] <= data[i]) return false; } return true; }
		
		inline R& operator[](size_t i) const { return data[i]; }
		
		out_t norm() const { out_t ans=0; size_t i=N; while(i!=0) { const R x=data[--i]; ans+=x*x; } return std::sqrt(ans); }
		out_t sum() const { out_t ans=0; size_t i=N; while(i!=0) { ans+=data[--i]; } return ans; }
		out_t sumSq() const { out_t ans=0; size_t i=N; while(i!=0) { const R x=data[--i]; ans+=x*x; } return ans; }
		void abs() const { size_t i=N; while(i!=0) { --i; data[i]=std::abs(data[i]); } }
		out_t max() const { if(N==0) return out_t(); out_t ans=data[N-1]; size_t i=N-1; while(i--!=0) if(ans<data[i]) ans=data[i]; return ans; }
		out_t min() const { if(N==0) return out_t(); out_t ans=data[N-1]; size_t i=N-1; while(i--!=0) if(ans>data[i]) ans=data[i]; return ans; }
		template<typename F> void apply(const F& f) const { size_t i=N; while(i--!=0) data[i]=f(data[i]); }
		template<typename F> Column<N,out_t> map(const F& f) const { Column<N,out_t> ans; size_t i=N; while(i--!=0) ans.data[i]=f(data[i]); return ans; }
		
		template<typename T> void minimize(const Matrix<N,1,T>& x) { size_t i=N; while(i!=0) { --i; if(x.data[i] < data[i]) data[i]=x.data[i]; } }
		template<typename T> void minimize(const SubVector<N,T>& x) { size_t i=N; while(i!=0) { --i; if(x.data[i] < data[i]) data[i]=x.data[i]; } }
		template<typename T> void maximize(const Matrix<N,1,T>& x) { size_t i=N; while(i!=0) { --i; if(data[i] < x.data[i]) data[i]=x.data[i]; } }
		template<typename T> void maximize(const SubVector<N,T>& x) { size_t i=N; while(i!=0) { --i; if(data[i] < x.data[i]) data[i]=x.data[i]; } }
				
		inline Row<N,out_t> transpose() const ATTR_must_check {
			Row<N,out_t> ans=fmat_internal::NoInit();
			fmat_internal::tmemcpy<out_t>(ans.data,data,N);
			return ans;
		}

		inline std::string fmt(std::string const &numberFormat=defaultNumberFormat, 
			std::string const &firstLineStart="{",
			std::string const &nextLineStart= "",
			std::string const &lastLineEnd=   "}",
			std::string const &rowBegin="",
			std::string const &elementSep="",
			std::string const &rowEnd="",
			std::string const &rowSep=" ") const
		{
			return fullfmt(data,N,1,numberFormat,firstLineStart,nextLineStart,lastLineEnd,rowBegin,elementSep,rowEnd,rowSep);
		}

	protected:
		SubVector() : data(NULL) {}
		R* data;
	};
	
	// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if !defined(__clang__) && defined(__GNUC__) && !(__GNUC__==4 && __GNUC_MINOR__<1)
	template<size_t N, typename R> const size_t SubVector<N,R>::HEIGHT=N;
	template<size_t N, typename R> const size_t SubVector<N,R>::WIDTH=1;
	template<size_t N, typename R> const size_t SubVector<N,R>::CAP=N;
#endif
	
	
	template<size_t H, size_t W, typename R=fmatReal>
	class SubMatrix {
		template<size_t MH, size_t MW, typename MR> friend class Matrix;
		template<size_t MH, size_t MW, typename MR> friend class SubMatrix;
		template<size_t MN, typename MR> friend class SubVector;
		template<size_t MN, typename MR> friend class Column;
		template<size_t MN, typename MR> friend class Row;
	public:
		// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if defined(__clang__) || !defined(__GNUC__) || (__GNUC__==4 && __GNUC_MINOR__<1)
		static const size_t HEIGHT=H;
		static const size_t WIDTH=W;
		static const size_t CAP=H*W;
#else
		static const size_t HEIGHT;
		static const size_t WIDTH;
		static const size_t CAP;
#endif
		typedef R storage_t;
		typedef typename fmat_internal::unconst<R>::type out_t;
		
		explicit SubMatrix(R* x, size_t colStride=H) { size_t c=W; while(c--!=0) cols[c].data = &x[c*colStride]; }
		// allow implicit casts for same size
		SubMatrix(const SubMatrix<H,W,out_t>& x) { size_t c=W; while(c--!=0) cols[c].data = x.cols[c].data; }
		SubMatrix(Matrix<H,W,out_t>& x) { size_t c=W; while(c--!=0) cols[c].data = &x.data[c*H]; }
		template<typename T> SubMatrix(const SubMatrix<H,W,T>& x) { size_t c=W; while(c--!=0) cols[c].data = x.cols[c].data; }
		template<typename T> SubMatrix(const Matrix<H,W,T>& x) { size_t c=W; while(c--!=0) cols[c].data = &x.data[c*H]; }
		
		//! this constructor handles shrinking/slicing from other SubMatrix, starting at (0,0)
		/*! If @a x has const data and this is non-const, we rely on an error on the #cols initializer to point that out */
		template<size_t SH, size_t SW, typename T> explicit SubMatrix(const SubMatrix<SH,SW,T>& x) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_rows_than_source,(H<=SH)>);
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_columns_than_source,(W<=SW)>);
			size_t c=W; while(c--!=0) cols[c].data = x.cols[c].data; 
		}
		//! this constructor handles shrinking/slicing from other SubMatrix, at a specified offset
		/*! If @a x has const data and this is non-const, we rely on an error on the #cols initializer to point that out */
		template<size_t SH, size_t SW, typename T> explicit SubMatrix(const SubMatrix<SH,SW,T>& x, size_t rowOff, size_t colOff) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_rows_than_source,(H<=SH)>);
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_columns_than_source,(W<=SW)>);
			size_t c=W; while(c--!=0) cols[c].data = &x.cols[c+colOff].data[rowOff]; 
		}
		//! this constructor handles shrinking/slicing from non-const Matrix, starting at (0,0)
		template<size_t SH, size_t SW> explicit SubMatrix(Matrix<SH,SW,out_t>& x) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_rows_than_source,(H<=SH)>);
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_columns_than_source,(W<=SW)>);
			size_t c=W; while(c--!=0) cols[c].data = &x.data[c*SH]; 
		}
		//! this constructor handles shrinking/slicing from non-const Matrix, starting at specified offset
		template<size_t SH, size_t SW> explicit SubMatrix(Matrix<SH,SW,out_t>& x, size_t rowOff, size_t colOff) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_rows_than_source,(H<=SH)>);
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_columns_than_source,(W<=SW)>);
			size_t c=W; while(c--!=0) cols[c].data = &x.data[(c+colOff)*SH+rowOff];
		}
		//! this constructor handles shrinking/slicing from const Matrix (which can only be used if R is const too - error on #cols init otherwise)
		template<size_t SH, size_t SW, typename T> explicit SubMatrix(const Matrix<SH,SW,T>& x) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_rows_than_source,(H<=SH)>);
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_columns_than_source,(W<=SW)>);
			size_t c=W; while(c--!=0) cols[c].data = &x.data[c*SH]; 
		}
		//! this constructor handles shrinking/slicing from const Matrix (which can only be used if R is const too - error on #cols init otherwise)
		template<size_t SH, size_t SW, typename T> explicit SubMatrix(const Matrix<SH,SW,T>& x, size_t rowOff, size_t colOff) {
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_rows_than_source,(H<=SH)>);
			(void)sizeof(fmat_internal::CompileTimeAssert<SubMatrix_has_more_columns_than_source,(W<=SW)>);
			size_t c=W; while(c--!=0) cols[c].data = &x.data[(c+colOff)*SH+rowOff];
		}

		operator SubMatrix<H,W,const R>() const { return SubMatrix<H,W,const R>(*this); }

		template<typename T> SubMatrix& importFrom(const T& x) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { cols[c].data[r]=x(r,c); } return *this; }
		template<typename T> SubMatrix& importFromCMajor(const T& x, size_t stride=H) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { cols[c].data[r]=x[c*stride+r]; } return *this; }
		template<typename T> SubMatrix& importFromRMajor(const T& x, size_t stride=W) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { cols[c].data[r]=x[r*stride+c]; } return *this; }
		template<typename T> SubMatrix& importFrom2DArray(const T& x) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { cols[c].data[r]=x[r][c]; } return *this; }
		template<typename T> SubMatrix& importFromNewmat(const T& x) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { cols[c].data[r]=x(r+1,c+1); } return *this; }
		template<typename T> T exportTo() const { T x; for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x(r,c)=cols[c].data[r]; } return x; }
		template<typename T> T& exportTo(T& x) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x(r,c)=cols[c].data[r]; } return x; }
		template<typename T> T& exportToCMajor(T& x, size_t stride=H) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x[c*stride+r]=cols[c].data[r]; } return x; }
		template<typename T> T& exportToRMajor(T& x, size_t stride=W) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x[r*stride+c]=cols[c].data[r]; } return x; }
		template<typename T> T& exportTo2DArray(T& x) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x[r][c]=cols[c].data[r]; } return x; }
		template<typename T> T& exportToNewmat(T& x) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x(r+1,c+1)=cols[c].data[r]; } return x; }
		
		const SubMatrix& operator=(R x) const { size_t c=W; while(c!=0) cols[--c]=x; return *this; }
		const SubMatrix& operator=(const R* x) const { size_t c=W; while(c--!=0) cols[c]=&x[c*H]; return *this; }
		const SubMatrix& operator=(const Matrix<H,W,out_t>& x) const { size_t c=W; while(c--!=0) cols[c]=&x.data[c*H]; return *this; }
		const SubMatrix& operator=(const SubMatrix& x) const { size_t c=W; while(c--!=0) cols[c]=x.cols[c]; return *this; }
		// this can't convert types, but handles assignment to const R from non-const R (without duplicating assignment above)
		template<typename T> const SubMatrix& operator=(const SubMatrix<H,W,T>& x) const { size_t c=W; while(c--!=0) cols[c]=x.cols[c]; return *this; }
		
		const SubMatrix& operator+=(R x) const { size_t c=W; while(c!=0) cols[--c]+=x; return *this; }
		const SubMatrix& operator-=(R x) const { size_t c=W; while(c!=0) cols[--c]-=x; return *this; }
		const SubMatrix& operator*=(R x) const { size_t c=W; while(c!=0) cols[--c]*=x; return *this; }
		const SubMatrix& operator/=(R x) const { x=1/x; size_t c=W; while(c!=0) cols[--c]*=x; return *this; }
		Matrix<H,W,out_t> operator+(R x) const { return Matrix<H,W,out_t>(*this)+=x; }
		Matrix<H,W,out_t> operator-(R x) const { return Matrix<H,W,out_t>(*this)-=x; }
		Matrix<H,W,out_t> operator*(R x) const { return Matrix<H,W,out_t>(*this)*=x; }
		Matrix<H,W,out_t> operator/(R x) const { return Matrix<H,W,out_t>(*this)/=x; }
		Matrix<H,W,out_t> operator-() const { Matrix<H,W,out_t> ans(*this); size_t i=H*W; while(i--!=0) ans.data[i]=-ans.data[i]; return ans; }
		
		friend Matrix<H,W,out_t> operator+(R x, const SubMatrix& a) { return a+x; }
		friend Matrix<H,W,out_t> operator-(R x, const SubMatrix& a) { return (-a)+x; }
		friend Matrix<H,W,out_t> operator*(R x, const SubMatrix& a) { return a*x; }
		
		template<typename T> const SubMatrix& operator+=(const SubMatrix<H,W,T>& x) const { size_t c=W; while(c--!=0) cols[c]+=x.cols[c]; return *this; }
		template<typename T> const SubMatrix& operator-=(const SubMatrix<H,W,T>& x) const { size_t c=W; while(c--!=0) cols[c]-=x.cols[c]; return *this; }
		template<typename T> const SubMatrix& operator+=(const Matrix<H,W,T>& x) const {
			for(size_t c=W; c--!=0;) {
				const T * const tmp = &x.data[c*H];
				for(size_t r=H; r--!=0;)
					cols[c].data[r]+=tmp[r];
			}
			return *this;
		}
		template<typename T> const SubMatrix& operator-=(const Matrix<H,W,T>& x) const {
			for(size_t c=W; c--!=0;) {
				const T * const tmp = &x.data[c*H];
				for(size_t r=H; r--!=0;)
					cols[c].data[r]-=tmp[r];
			}
			return *this;
		}
		template<typename T> Matrix<H,W,out_t> operator+(const SubMatrix<H,W,T>& x) const { return Matrix<H,W,out_t>(*this)+=x; }
		template<typename T> Matrix<H,W,out_t> operator-(const SubMatrix<H,W,T>& x) const { return Matrix<H,W,out_t>(*this)-=x; }
		template<typename T> Matrix<H,W,out_t> operator+(const Matrix<H,W,T>& x) const { return Matrix<H,W,out_t>(*this)+=x; }
		template<typename T> Matrix<H,W,out_t> operator-(const Matrix<H,W,T>& x) const { return Matrix<H,W,out_t>(*this)-=x; }
		
		template<typename T> bool operator==(const SubMatrix<H,W,T>& x) const { size_t c=W; while(c!=0) { --c; if(cols[c]!=x.cols[c]) return false; } return true; }
		template<typename T> bool operator!=(const SubMatrix<H,W,T>& x) const { size_t c=W; while(c!=0) { --c; if(cols[c]!=x.cols[c]) return true; } return false; }
		template<typename T> bool operator==(const Matrix<H,W,T>& x) const { size_t c=W; while(c!=0) { --c; if(cols[c]!=x.column(c)) return false; } return true; }
		template<typename T> bool operator!=(const Matrix<H,W,T>& x) const { size_t c=W; while(c!=0) { --c; if(cols[c]!=x.column(c)) return true; } return false; }

		inline R& operator()(size_t r, size_t c) { return cols[c].data[r]; }
		inline const R& operator()(size_t r, size_t c) const { return cols[c].data[r]; }
		
		inline SubVector<H,R>& column(size_t i) { return cols[i]; }
		inline const SubVector<H,R>& column(size_t i) const { return cols[i]; }
		
		inline SubMatrix<1,W,R> row(size_t i) { return SubMatrix<1,W,R>(*this,i,0); }
		inline SubMatrix<1,W,const R> row(size_t i) const { return SubMatrix<1,W,const R>(*this,i,0); }

		Column<H,R> minC() const {
			if(CAP==0) return Column<H,R>();
			Column<H,R> ans = cols[0];
			size_t c=W; while(--c!=0) { size_t r=H; while(r!=0) { --r; if(cols[c].data[r]<ans[r]) ans[r]=cols[c].data[r]; } }
			return ans;
		}
		
		Column<H,R> maxC() const {
			if(CAP==0) return Column<H,R>();
			Column<H,R> ans = cols[0];
			size_t c=W; while(--c!=0) { size_t r=H; while(r!=0) { --r; if(ans[r]<cols[c].data[r]) ans[r]=cols[c].data[r]; } }
			return ans;
		}
		Row<W,R> minR() const { Row<W,R> ans=fmat_internal::NoInit(); size_t c=W; while(c!=0) { --c; ans[c]=cols[c].min(); } return ans; }
		Row<W,R> maxR() const { Row<W,R> ans=fmat_internal::NoInit(); size_t c=W; while(c!=0) { --c; ans[c]=cols[c].max(); } return ans; }
		void abs() const { size_t c=W; while(c!=0) cols[--c].abs(); }
		template<typename F> void apply(const F& f) const { size_t c=W; while(c--!=0) cols[c].apply(f); }
		template<typename F> Matrix<H,W,out_t> map(const F& f) const {
			if(H==0 || W==0) return Matrix<H,W,out_t>();
			Matrix<H,W,out_t> ans=fmat_internal::NoInit();
			size_t c=W; while(c!=0) { --c; size_t r=H; while(r!=0) { --r; ans.data[c*H+r] = f(cols[c].data[r]); } }
			return ans;
		}
		
		inline Matrix<W,H,out_t> transpose() const ATTR_must_check {
			Matrix<W,H,out_t> ans=fmat_internal::NoInit();
			for(size_t c=W; c--!=0;)
				for(size_t r=H; r--!=0;)
					ans.data[r*W+c]=cols[c].data[r];
			return ans;
		}

		std::string fmt(std::string const &numberFormat=defaultNumberFormat, 
			std::string const &firstLineStart="{ ",
			std::string const &nextLineStart= "  ",
			std::string const &lastLineEnd=   " }",
			std::string const &rowBegin="",
			std::string const &elementSep=" ",
			std::string const &rowEnd="",
			std::string const &rowSep="\n") const
		{
			Matrix<H,W,out_t> m(*this); // copy to a matrix so the data is all together for fullfmt... not ideal, but this is rarely used
			return fullfmt(m.data,H,W,numberFormat,firstLineStart,nextLineStart,lastLineEnd,rowBegin,elementSep,rowEnd,rowSep);
		}

	protected:
		SubVector<H,R> cols[(W==0)?1:W]; // funny definition is for older compilers that don't like 0-sized arrays
	};
	// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if !defined(__clang__) && defined(__GNUC__) && !(__GNUC__==4 && __GNUC_MINOR__<1)
	template<size_t H, size_t W, typename R> const size_t SubMatrix<H,W,R>::HEIGHT=H;
	template<size_t H, size_t W, typename R> const size_t SubMatrix<H,W,R>::WIDTH=W;
	template<size_t H, size_t W, typename R> const size_t SubMatrix<H,W,R>::CAP=H*W;
#endif
	
	
	template<size_t H, size_t W, typename R=fmatReal>
	class Matrix {
		template<size_t MH, size_t MW, typename MR> friend class Matrix;
		template<size_t MH, size_t MW, typename MR> friend class SubMatrix;
		template<size_t MN, typename MR> friend class SubVector;
		template<size_t MN, typename MR> friend class Column;
		template<size_t MN, typename MR> friend class Row;
	public:
		// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if defined(__clang__) || !defined(__GNUC__) || (__GNUC__==4 && __GNUC_MINOR__<1)
		static const size_t HEIGHT=H;
		static const size_t WIDTH=W;
		static const size_t CAP=H*W;
#else
		static const size_t HEIGHT;
		static const size_t WIDTH;
		static const size_t CAP;
#endif
		typedef R storage_t;
		typedef typename fmat_internal::unconst<R>::type out_t;
		
		Matrix(const fmat_internal::NoInit&) {}
		
		Matrix() { memset(data,0,sizeof(R)*CAP); }
		explicit Matrix(const R x) { fmat_internal::tmemset<R>(data,x,CAP); }
		explicit Matrix(const R* x, size_t colStride=H) { size_t c=W; while(c--!=0) fmat_internal::tmemcpy<R>(&data[c*H],&x[c*colStride],H); }
		Matrix(const SubMatrix<H,W,out_t>& x) { size_t c=W; while(c--!=0) fmat_internal::tmemcpy<R>(&data[c*H],x.cols[c].data,H); }
		Matrix(const SubMatrix<H,W,const out_t>& x) { size_t c=W; while(c--!=0) fmat_internal::tmemcpy<R>(&data[c*H],x.cols[c].data,H); }
		Matrix(const Matrix<H,W,out_t>& x) { fmat_internal::tmemcpy<R>(data,x.data,CAP); }
		Matrix(const Matrix<H,W,const out_t>& x) { fmat_internal::tmemcpy<R>(data,x.data,CAP); }

		template<size_t SH, size_t SW, typename T> explicit Matrix(const SubMatrix<SH,SW,T>& x) {
			memset(data,0,sizeof(R)*CAP);
			size_t toCopy = std::min(H,SH);
			size_t c = std::min(W,SW);
			while(c--!=0) 
				fmat_internal::tmemcpy<R>(&data[c*H],&x.cols[c][0],toCopy);
		}
		template<size_t SH, size_t SW, typename T> explicit Matrix(const SubMatrix<SH,SW,T>& x, size_t rowOff, size_t colOff) {
			memset(data,0,sizeof(R)*CAP);
			size_t toCopy = std::min(H,SH-rowOff);
			size_t c = std::min(W,SW-colOff);
			while(c--!=0) 
				fmat_internal::tmemcpy<R>(&data[c*H],&x.cols[c+colOff][rowOff],toCopy);
		}
		template<size_t SH, size_t SW> explicit Matrix(const Matrix<SH,SW,R>& x) {
			memset(data,0,sizeof(R)*CAP);
			size_t toCopy = std::min(H,SH);
			size_t c = std::min(W,SW);
			while(c--!=0) 
				fmat_internal::tmemcpy<R>(&data[c*H],&x.data[c*SH],toCopy);
		}
		template<size_t SH, size_t SW> explicit Matrix(const Matrix<SH,SW,R>& x, size_t rowOff, size_t colOff) {
			memset(data,0,sizeof(R)*CAP);
			size_t toCopy = std::min(H,SH-rowOff);
			size_t c = std::min(W,SW-colOff);
			while(c--!=0) 
				fmat_internal::tmemcpy<R>(&data[c*H],&x.data[(c+colOff)*SH+rowOff],toCopy);
		}
		
		// we want to avoid introducing any 'virtual' to maximize performance
		//virtual ~Matrix() {}
		
		static const Matrix& identity() { static IdentityMatrix<H,W,R> ident; return ident; }
		
		template<typename T> Matrix& importFrom(const T& x) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { data[c*H+r]=x(r,c); } return *this; }
		template<typename T> Matrix& importFromCMajor(const T& x, size_t stride=H) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { data[c*H+r]=x[c*stride+r]; } return *this; }
		template<typename T> Matrix& importFromRMajor(const T& x, size_t stride=W) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { data[c*H+r]=x[r*stride+c]; } return *this; }
		template<typename T> Matrix& importFrom2DArray(const T& x) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { data[c*H+r]=x[r][c]; } return *this; }
		template<typename T> Matrix& importFromNewmat(const T& x) { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { data[c*H+r]=x(r+1,c+1); } return *this; }
		template<typename T> T exportTo() const { T x; for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x(r,c)=data[c*H+r]; } return x; }
		template<typename T> T& exportTo(T& x) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x(r,c)=data[c*H+r]; } return x; }
		template<typename T> T& exportToCMajor(T& x, size_t stride=H) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x[c*stride+r]=data[c*H+r]; } return x; }
		template<typename T> T& exportToRMajor(T& x, size_t stride=W) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x[r*stride+c]=data[c*H+r]; } return x; }
		template<typename T> T& exportTo2DArray(T& x) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x[r][c]=data[c*H+r]; } return x; }
		template<typename T> T& exportToNewmat(T& x) const { for(size_t c=0; c<W; ++c) for(size_t r=0; r<H; ++r) { x((int)r+1,(int)c+1)=data[c*H+r]; } return x; }
		
		Matrix& operator=(const R* x) { fmat_internal::tmemcpy<R>(data,x,CAP); return *this; }
		Matrix& operator=(R x) { fmat_internal::tmemset<R>(data,x,CAP); return *this; }
		Matrix& operator=(const SubMatrix<H,W,R>& x) { size_t c=W; while(c--!=0) fmat_internal::tmemcpy<R>(&data[c*H],x.cols[c].data,H); return *this; }
		Matrix& operator=(const Matrix<H,W,R>& x) { fmat_internal::tmemcpy<R>(data,x.data,CAP); return *this; }
		
		Matrix& operator+=(R x) { size_t i=CAP; while(i!=0) data[--i]+=x; return *this; }
		Matrix& operator-=(R x) { size_t i=CAP; while(i!=0) data[--i]-=x; return *this; }
		Matrix& operator*=(R x) { size_t i=CAP; while(i!=0) data[--i]*=x; return *this; }
		Matrix& operator/=(R x) { x=1/x; size_t i=CAP; while(i!=0) data[--i]*=x; return *this; }
		Matrix<H,W,out_t> operator+(R x) const { return Matrix<H,W,out_t>(*this)+=x; }
		Matrix<H,W,out_t> operator-(R x) const { return Matrix<H,W,out_t>(*this)-=x; }
		Matrix<H,W,out_t> operator*(R x) const { return Matrix<H,W,out_t>(*this)*=x; }
		Matrix<H,W,out_t> operator/(R x) const { return Matrix<H,W,out_t>(*this)/=x; }
		Matrix<H,W,out_t> operator-() const { Matrix<H,W,out_t> ans(*this); size_t i=H*W; while(i--!=0) ans.data[i]=-ans.data[i]; return ans; }
		
		friend Matrix<H,W,out_t> operator+(R x, const Matrix& a) { return a+x; }
		friend Matrix<H,W,out_t> operator-(R x, const Matrix& a) { return (-a)+x; }
		friend Matrix<H,W,out_t> operator*(R x, const Matrix& a) { return a*x; }
		
		Matrix& operator+=(const SubMatrix<H,W,out_t>& x) {
			for(size_t c=W; c--!=0;) {
				R * const tmp = &data[c*H];
				for(size_t r=H; r--!=0;)
					tmp[r]+=x.cols[c].data[r];
			}
			return *this;
		}
		Matrix& operator-=(const SubMatrix<H,W,out_t>& x) {
			for(size_t c=W; c--!=0;) {
				R * const tmp = &data[c*H];
				for(size_t r=H; r--!=0;)
					tmp[r]-=x.cols[c].data[r];
			}
			return *this;
		}
		Matrix& operator+=(const SubMatrix<H,W,const out_t>& x) {
			for(size_t c=W; c--!=0;) {
				R * const tmp = &data[c*H];
				for(size_t r=H; r--!=0;)
					tmp[r]+=x.cols[c].data[r];
			}
			return *this;
		}
		Matrix& operator-=(const SubMatrix<H,W,const out_t>& x) {
			for(size_t c=W; c--!=0;) {
				R * const tmp = &data[c*H];
				for(size_t r=H; r--!=0;)
					tmp[r]-=x.cols[c].data[r];
			}
			return *this;
		}
		Matrix& operator+=(const Matrix<H,W,R>& x) { size_t i=CAP; while(i--!=0) data[i]+=x.data[i]; return *this; }
		Matrix& operator-=(const Matrix<H,W,R>& x) { size_t i=CAP; while(i--!=0) data[i]-=x.data[i]; return *this; }
		Matrix<H,W,R> operator+(const SubMatrix<H,W,R>& x) const { return Matrix<H,W,R>(*this)+=x; }
		Matrix<H,W,R> operator-(const SubMatrix<H,W,R>& x) const { return Matrix<H,W,R>(*this)-=x; }
		Matrix<H,W,R> operator+(const Matrix<H,W,R>& x) const { return Matrix<H,W,R>(*this)+=x; }
		Matrix<H,W,R> operator-(const Matrix<H,W,R>& x) const { return Matrix<H,W,R>(*this)-=x; }
		
		bool operator==(const SubMatrix<H,W,R>& x) const { size_t c=W; while(c!=0) { --c; if(column(c)!=x.cols[c]) return false; } return true; }
		bool operator!=(const SubMatrix<H,W,R>& x) const { size_t c=W; while(c!=0) { --c; if(column(c)!=x.cols[c]) return true; } return false; }
		bool operator==(const Matrix<H,W,R>& x) const { size_t i=H*W; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		bool operator!=(const Matrix<H,W,R>& x) const { size_t i=H*W; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }

		inline R& operator()(size_t r, size_t c) { return data[c*H+r]; }
		inline const R& operator()(size_t r, size_t c) const { return data[c*H+r]; }
		
		inline SubVector<H,R> column(size_t i) { return SubVector<H,R>(&data[i*H]); }
		inline const SubVector<H,const R> column(size_t i) const { return SubVector<H,const R>(&data[i*H]); }

		inline SubMatrix<1,W,R> row(size_t i) { return SubMatrix<1,W,R>(data+i, H); }
		inline const SubMatrix<1,W,const R> row(size_t i) const { return SubMatrix<1,W,const R>(data+i, H); }
		
		Column<H,R> minC() const {
			if(CAP==0) return Column<H,R>();
			Column<H,R> ans(data);
			size_t c=W; while(--c!=0) { size_t r=H; while(r!=0) { --r; if(data[c*H+r]<ans[r]) ans[r]=data[c*H+r]; } }
			return ans;
		}
		
		Column<H,R> maxC() const {
			if(CAP==0) return Column<H,R>();
			Column<H,R> ans(data);
			size_t c=W; while(--c!=0) { size_t r=H; while(r!=0) { --r; if(ans[r]<data[c*H+r]) ans[r]=data[c*H+r]; } }
			return ans;
		}
		Row<W,R> minR() const { Row<W,R> ans=fmat_internal::NoInit(); size_t c=W; while(c!=0) { --c; ans[c]=column(c).min(); } return ans; }
		Row<W,R> maxR() const { Row<W,R> ans=fmat_internal::NoInit(); size_t c=W; while(c!=0) { --c; ans[c]=column(c).max(); } return ans; }
		void abs() { size_t i=CAP; while(i!=0) { --i; data[i]=std::abs(data[i]); } }
		template<typename F> void apply(const F& f) { size_t i=H*W; while(i!=0) { --i;  data[i] = f(data[i]); } }
		template<typename F> Matrix map(const F& f) const {
			if(H==0 || W==0) return Matrix();
			Matrix ans=fmat_internal::NoInit();
			size_t i=H*W; while(i!=0) { --i;  ans.data[i] = f(data[i]); }
			return ans;
		}
		
		inline Matrix<W,H,R> transpose() const ATTR_must_check {
			Matrix<W,H,R> ans=fmat_internal::NoInit();
			for(size_t c=W; c--!=0;)
				for(size_t r=H; r--!=0;)
					ans.data[r*W+c]=data[c*H+r];
			return ans;
		}
		
		inline std::string fmt(std::string const &numberFormat=defaultNumberFormat, 
			std::string const &firstLineStart="[ ",
			std::string const &nextLineStart= "  ",
			std::string const &lastLineEnd=   " ]",
			std::string const &rowBegin="",
			std::string const &elementSep=" ",
			std::string const &rowEnd="",
			std::string const &rowSep="\n") const
		{
			return fullfmt(&data[0],H,W,numberFormat,firstLineStart,nextLineStart,lastLineEnd,rowBegin,elementSep,rowEnd,rowSep);
		}

	protected:
		R data[(H*W==0)?1:H*W]; // funny definition is for older compilers that don't like 0-sized arrays
		
		template<size_t HH, size_t WW, typename RR=fmatReal>
		struct IdentityMatrix : Matrix<HH,WW,RR> {
			IdentityMatrix() : Matrix() {
				size_t d = std::min(H,W);
				for(size_t i=0; i<d; ++i)
					this->data[i*H+i]=1;
			}
		};

	};
	// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if !defined(__clang__) && defined(__GNUC__) && !(__GNUC__==4 && __GNUC_MINOR__<1)
	template<size_t H, size_t W, typename R> const size_t Matrix<H,W,R>::HEIGHT=H;
	template<size_t H, size_t W, typename R> const size_t Matrix<H,W,R>::WIDTH=W;
	template<size_t H, size_t W, typename R> const size_t Matrix<H,W,R>::CAP=H*W;
#endif
	
	
	template<size_t N, typename R=fmatReal>
	class Column : public Matrix<N,1,R> {
		template<size_t S, typename T> friend class SubVector;
		template<size_t S, typename T> friend class Column;
		template<size_t S, typename T> friend class Row;
		friend Column<2,R> packT<>(R x, R y);
		friend Column<3,R> packT<>(R x, R y, R z);
		friend Column<4,R> packT<>(R x, R y, R z, R d);
		friend Column<2,fmatReal> pack(fmatReal x, fmatReal y);
		friend Column<3,fmatReal> pack(fmatReal x, fmatReal y, fmatReal z);
		friend Column<4,fmatReal> pack(fmatReal x, fmatReal y, fmatReal z, fmatReal d);
	public:
		// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if defined(__clang__) || !defined(__GNUC__) || (__GNUC__==4 && __GNUC_MINOR__<1)
		static const size_t HEIGHT=N;
		static const size_t WIDTH=1;
		static const size_t CAP=N;
#else
		static const size_t HEIGHT;
		static const size_t WIDTH;
		static const size_t CAP;
#endif
		typedef R storage_t;
		typedef typename fmat_internal::unconst<R>::type out_t;
		
		Column(const fmat_internal::NoInit& noinit) : Matrix<N,1,R>(noinit) {}
		
		Column() : Matrix<N,1,R>() {}
		explicit Column(const R x) : Matrix<N,1,R>(x) {}
		explicit Column(const R* x, size_t stride=sizeof(R)) : Matrix<N,1,R>(fmat_internal::NoInit()) { size_t i=N; while(i!=0) { --i; data[i]=x[i*stride/sizeof(R)]; } }
		Column(const SubVector<N,R>& x) : Matrix<N,1,R>(x.data) {}
		Column(const SubVector<N,const R>& x) : Matrix<N,1,R>(x.data) {}
		Column(const SubMatrix<N,1,R>& x) : Matrix<N,1,R>(x) {}
		Column(const SubMatrix<N,1,const R>& x) : Matrix<N,1,R>(x) {}
		Column(const Matrix<N,1,R>& x) : Matrix<N,1,R>(x) {}
		template<size_t S> explicit Column(const Column<S,R>& x, size_t srcOffset=0) : Matrix<N,1,R>() {
			size_t toCopy = std::min(N,S-srcOffset);
			fmat_internal::tmemcpy<R>(data,&x.data[srcOffset],toCopy);
		}
		template<size_t S> explicit Column(const SubVector<S,R>& x, size_t srcOffset=0) : Matrix<N,1,R>() {
			size_t toCopy = std::min(N,S-srcOffset);
			fmat_internal::tmemcpy<R>(data,&x.data[srcOffset],toCopy);
		}
		template<size_t S> explicit Column(const SubVector<S,const R>& x, size_t srcOffset=0) : Matrix<N,1,R>() {
			size_t toCopy = std::min(N,S-srcOffset);
			fmat_internal::tmemcpy<R>(data,&x.data[srcOffset],toCopy);
		}
		template<typename T> Column& importFrom(const T& x) { size_t i=N; while(i!=0) { --i; data[i]=x[i]; } return *this; }
		template<typename T> T exportTo() const { T x; size_t i=N; while(i!=0) { --i; x[i]=data[i]; } return x; }
		template<typename T> T& exportTo(T& x) const { size_t i=N; while(i!=0) { --i; x[i]=data[i]; } return x; }
		// provided by implicit SubVector constructors
		//operator SubVector<N,R>() { return SubVector<N,R>(data); }
		//operator SubVector<N,const R>() const { return SubVector<N,const R>(data); }
		
		Column& operator=(const R* x) { fmat_internal::tmemcpy<R>(data,x,CAP); return *this; }
		Column& operator=(R x) { fmat_internal::tmemset<R>(data,x,CAP); return *this; }
		Column& operator=(const SubVector<N,R>& x) { fmat_internal::tmemcpy<R>(data,x.data,N); return *this; }
		Column& operator=(const SubVector<N,const R>& x) { fmat_internal::tmemcpy<R>(data,x.data,N); return *this; }
		Column& operator=(const SubMatrix<N,1,R>& x) { fmat_internal::tmemcpy<R>(data,x.cols[0].data,N); return *this; }
		Column& operator=(const SubMatrix<N,1,const R>& x) { fmat_internal::tmemcpy<R>(data,x.cols[0].data,N); return *this; }
		Column& operator=(const Matrix<N,1,R>& x) { fmat_internal::tmemcpy<R>(data,x.data,CAP); return *this; }
		
		Column& operator+=(R x) { size_t i=N; while(i!=0) data[--i]+=x; return *this; }
		Column& operator-=(R x) { size_t i=N; while(i!=0) data[--i]-=x; return *this; }
		Column& operator*=(R x) { size_t i=N; while(i!=0) data[--i]*=x; return *this; }
		Column& operator/=(R x) { x=1/x; size_t i=N; while(i!=0) data[--i]*=x; return *this; }
		Column<N,out_t> operator+(R x) const { return Column<N,out_t>(*this)+=x; }
		Column<N,out_t> operator-(R x) const { return Column<N,out_t>(*this)-=x; }
		Column<N,out_t> operator*(R x) const { return Column<N,out_t>(*this)*=x; }
		Column<N,out_t> operator/(R x) const { return Column<N,out_t>(*this)/=x; }
		Column<N,out_t> operator-() const { Column<N,out_t> ans(*this); size_t i=N; while(i--!=0) ans.data[i]=-ans.data[i]; return ans; }
		
		friend Column<N,out_t> operator+(R x, const Column& a) { return a+x; }
		friend Column<N,out_t> operator-(R x, const Column& a) { return (-a)+x; }
		friend Column<N,out_t> operator*(R x, const Column& a) { return a*x; }
		
		Column& operator+=(const SubVector<N,R>& x) { size_t i=CAP; while(i--!=0) data[i]+=x.data[i]; return *this; }
		Column& operator-=(const SubVector<N,R>& x) { size_t i=CAP; while(i--!=0) data[i]-=x.data[i]; return *this; }
		Column& operator+=(const SubVector<N,const R>& x) { size_t i=CAP; while(i--!=0) data[i]+=x.data[i]; return *this; }
		Column& operator-=(const SubVector<N,const R>& x) { size_t i=CAP; while(i--!=0) data[i]-=x.data[i]; return *this; }
		Column& operator+=(const SubMatrix<N,1,R>& x) { Matrix<N,1,R>::operator+=(x); return *this;}
		Column& operator-=(const SubMatrix<N,1,R>& x) { Matrix<N,1,R>::operator-=(x); return *this;}
		Column& operator+=(const SubMatrix<N,1,const R>& x) { Matrix<N,1,R>::operator+=(x); return *this;}
		Column& operator-=(const SubMatrix<N,1,const R>& x) { Matrix<N,1,R>::operator-=(x); return *this;}
		Column& operator+=(const Matrix<N,1,R>& x) { Matrix<N,1,R>::operator+=(x); return *this;}
		Column& operator-=(const Matrix<N,1,R>& x) { Matrix<N,1,R>::operator-=(x); return *this;}
		
		Column operator+(const SubVector<N,R>& x) const { return Column(*this)+=x; }
		Column operator-(const SubVector<N,R>& x) const { return Column(*this)-=x; }
		Column operator+(const SubVector<N,const R>& x) const { return Column(*this)+=x; }
		Column operator-(const SubVector<N,const R>& x) const { return Column(*this)-=x; }
		Column operator+(const SubMatrix<N,1,R>& x) const { return Column(*this)+=x; }
		Column operator-(const SubMatrix<N,1,R>& x) const { return Column(*this)-=x; }
		Column operator+(const SubMatrix<N,1,const R>& x) const { return Column(*this)+=x; }
		Column operator-(const SubMatrix<N,1,const R>& x) const { return Column(*this)-=x; }
		Column operator+(const Matrix<N,1,R>& x) const { return Column(*this)+=x; }
		Column operator-(const Matrix<N,1,R>& x) const { return Column(*this)-=x; }
		
		bool operator==(const SubVector<N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		bool operator!=(const SubVector<N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		bool operator<(const SubVector<N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }
		bool operator==(const SubVector<N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		bool operator!=(const SubVector<N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		bool operator<(const SubVector<N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }
		bool operator==(const SubMatrix<N,1,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[0][i]) return false; } return true; }
		bool operator!=(const SubMatrix<N,1,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[0][i]) return true; } return false; }
		bool operator<(const SubMatrix<N,1,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.cols[0][i]) return true; if(x.cols[0][i]<data[i]) return false; } return false; }
		bool operator==(const SubMatrix<N,1,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[0][i]) return false; } return true; }
		bool operator!=(const SubMatrix<N,1,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[0][i]) return true; } return false; }
		bool operator<(const SubMatrix<N,1,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.cols[0][i]) return true; if(x.cols[0][i]<data[i]) return false; } return false; }
		bool operator==(const Matrix<N,1,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		bool operator!=(const Matrix<N,1,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		bool operator<(const Matrix<N,1,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }

		//! returns true if all elements are less than the corresponding element
		template<typename T> bool operator<<(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; if(x.data[i] <= data[i]) return false; } return true; }
		template<typename T> bool operator<<(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; if(x.data[i] <= data[i]) return false; } return true; }
		
		inline R& operator[](size_t i) { return data[i]; }
		inline const R& operator[](size_t i) const { return data[i]; }
		
		R norm() const ATTR_must_check { R ans=0; size_t i=N; while(i!=0) { const R x=data[--i]; ans+=x*x; } return std::sqrt(ans); }
		R sum() const ATTR_must_check { R ans=0; size_t i=N; while(i!=0) { ans+=data[--i]; } return ans; }
		R sumSq() const ATTR_must_check { R ans=0; size_t i=N; while(i!=0) { const R x=data[--i]; ans+=x*x; } return ans; }
		void abs() { size_t i=N; while(i!=0) { --i; data[i]=std::abs(data[i]); } }
		R max() const ATTR_must_check { if(N==0) return R(); R ans=data[N-1]; size_t i=N-1; while(i--!=0) if(ans<data[i]) ans=data[i]; return ans; }
		R min() const ATTR_must_check { if(N==0) return R(); R ans=data[N-1]; size_t i=N-1; while(i--!=0) if(ans>data[i]) ans=data[i]; return ans; }
		template<typename F> void apply(const F& f) { size_t i=N; while(i--!=0) data[i]=f(data[i]); }
		template<typename F> Column<N,R> map(const F& f) const { Column<N,R> ans; size_t i=N; while(i--!=0) ans.data[i]=f(data[i]); return ans; }
		
		template<typename T> void minimize(const Matrix<N,1,T>& x) { size_t i=N; while(i!=0) { --i; if(x.data[i] < data[i]) data[i]=x.data[i]; } }
		template<typename T> void minimize(const SubVector<N,T>& x) { size_t i=N; while(i!=0) { --i; if(x.data[i] < data[i]) data[i]=x.data[i]; } }
		template<typename T> void maximize(const Matrix<N,1,T>& x) { size_t i=N; while(i!=0) { --i; if(data[i] < x.data[i]) data[i]=x.data[i]; } }
		template<typename T> void maximize(const SubVector<N,T>& x) { size_t i=N; while(i!=0) { --i; if(data[i] < x.data[i]) data[i]=x.data[i]; } }
		
		inline Row<N,R> transpose() const ATTR_must_check {
			Row<N,R> ans=fmat_internal::NoInit();
			fmat_internal::tmemcpy<R>(ans.data,data,N);
			return ans;
		}

		inline std::string fmt(std::string const &numberFormat=defaultNumberFormat, 
			std::string const &firstLineStart="[",
			std::string const &nextLineStart="",
			std::string const &lastLineEnd="]\u1D40", // aka UTF-8 literal 'ᵀ'
			std::string const &rowBegin="",
			std::string const &elementSep="",
			std::string const &rowEnd="",
			std::string const &rowSep=" ") const
		{
			return fullfmt(&data[0],N,1,numberFormat,firstLineStart,nextLineStart,lastLineEnd,rowBegin,elementSep,rowEnd,rowSep);
		}

	protected:
		using Matrix<N,1,R>::data;
	};
	// with extern templates, g++ was having issues with the inline definition; clang was having issues with the out-of-line definition...
#if !defined(__clang__) && defined(__GNUC__) && !(__GNUC__==4 && __GNUC_MINOR__<1)
	template<size_t N, typename R> const size_t Column<N,R>::HEIGHT=N;
	template<size_t N, typename R> const size_t Column<N,R>::WIDTH=1;
	template<size_t N, typename R> const size_t Column<N,R>::CAP=N;
#endif
	
	
	template<size_t N, typename R=fmatReal>
	class Row : public Matrix<1,N,R> {
		template<size_t S, typename T> friend class SubVector;
		template<size_t S, typename T> friend class Column;
		template<size_t S, typename T> friend class Row;
	public:
		static const size_t HEIGHT=1;
		static const size_t WIDTH=N;
		static const size_t CAP=N;
		typedef R storage_t;
		typedef typename fmat_internal::unconst<R>::type out_t;
		
		Row(const fmat_internal::NoInit& noinit) : Matrix<1,N,R>(noinit) {}
		
		Row() : Matrix<1,N,R>() {}
		explicit Row(const R x) : Matrix<1,N,R>(x) {}
		explicit Row(const R* x, size_t stride=sizeof(R)) : Matrix<1,N,R>(fmat_internal::NoInit()) { size_t i=N; while(i!=0) { --i; data[i]=x[i*stride/sizeof(R)]; } }
		Row(const SubVector<N,R>& x) : Matrix<1,N,R>(x.data) {}
		Row(const SubVector<N,const R>& x) : Matrix<1,N,R>(x.data) {}
		Row(const SubMatrix<1,N,R>& x) : Matrix<1,N,R>(x) {}
		Row(const SubMatrix<1,N,const R>& x) : Matrix<1,N,R>(x) {}
		Row(const Matrix<1,N,R>& x) : Matrix<1,N,R>(x) {}
		template<size_t S> explicit Row(const Row<S,R>& x, size_t srcOffset=0) : Matrix<1,N,R>() {
			size_t toCopy = std::min(N,S-srcOffset);
			fmat_internal::tmemcpy<R>(data,&x.data[srcOffset],toCopy);
		}
		template<size_t S> explicit Row(const SubVector<S,R>& x, size_t srcOffset=0) : Matrix<1,N,R>() {
			size_t toCopy = std::min(N,S-srcOffset);
			fmat_internal::tmemcpy<R>(data,&x.data[srcOffset],toCopy);
		}
		template<size_t S> explicit Row(const SubVector<S,const R>& x, size_t srcOffset=0) : Matrix<1,N,R>() {
			size_t toCopy = std::min(N,S-srcOffset);
			fmat_internal::tmemcpy<R>(data,&x.data[srcOffset],toCopy);
		}
		template<typename T> Row& importFrom(const T& x) { size_t i=N; while(i!=0) { --i; data[i]=x[i]; } return *this; }
		template<typename T> T exportTo() const { T x; size_t i=N; while(i!=0) { --i; x[i]=data[i]; } return x; }
		template<typename T> T& exportTo(T& x) const { size_t i=N; while(i!=0) { --i; x[i]=data[i]; } return x; }
		// provided by implicit SubVector constructors
		//operator SubVector<N,R>() { return SubVector<N,R>(data); }
		//operator SubVector<N,const R>() const { return SubVector<N,const R>(data); }
		
		Row& operator=(const R* x) { fmat_internal::tmemcpy<R>(data,x,CAP); return *this; }
		Row& operator=(R x) { fmat_internal::tmemset<R>(data,x,CAP); return *this; }
		Row& operator=(const SubVector<N,R>& x) { fmat_internal::tmemcpy<R>(data,x.data,N); return *this; }
		Row& operator=(const SubVector<N,const R>& x) { fmat_internal::tmemcpy<R>(data,x.data,N); return *this; }
		Row& operator=(const SubMatrix<1,N,R>& x) { fmat_internal::tmemcpy<R>(data,x.cols[0].data,N); return *this; }
		Row& operator=(const SubMatrix<1,N,const R>& x) { fmat_internal::tmemcpy<R>(data,x.cols[0].data,N); return *this; }
		Row& operator=(const Matrix<1,N,R>& x) { fmat_internal::tmemcpy<R>(data,x.data,CAP); return *this; }
		
		Row& operator+=(R x) { size_t i=N; while(i!=0) data[--i]+=x; return *this; }
		Row& operator-=(R x) { size_t i=N; while(i!=0) data[--i]-=x; return *this; }
		Row& operator*=(R x) { size_t i=N; while(i!=0) data[--i]*=x; return *this; }
		Row& operator/=(R x) { x=1/x; size_t i=N; while(i!=0) data[--i]*=x; return *this; }
		Row<N,out_t> operator+(R x) const { return Row<N,R>(*this)+=x; }
		Row<N,out_t> operator-(R x) const { return Row<N,R>(*this)-=x; }
		Row<N,out_t> operator*(R x) const { return Row<N,R>(*this)*=x; }
		Row<N,out_t> operator/(R x) const { return Row<N,R>(*this)/=x; }
		Row<N,out_t> operator-() const { Row<N,R> ans(*this); size_t i=N; while(i--!=0) ans.data[i]=-ans.data[i]; return ans; }
		
		friend Row<N,out_t> operator+(R x, const Row& a) { return a+x; }
		friend Row<N,out_t> operator-(R x, const Row& a) { return (-a)+x; }
		friend Row<N,out_t> operator*(R x, const Row& a) { return a*x; }
		
		Row& operator+=(const SubVector<N,R>& x) { size_t i=CAP; while(i--!=0) data[i]+=x.data[i]; return *this; }
		Row& operator-=(const SubVector<N,R>& x) { size_t i=CAP; while(i--!=0) data[i]-=x.data[i]; return *this; }
		Row& operator+=(const SubVector<N,const R>& x) { size_t i=CAP; while(i--!=0) data[i]+=x.data[i]; return *this; }
		Row& operator-=(const SubVector<N,const R>& x) { size_t i=CAP; while(i--!=0) data[i]-=x.data[i]; return *this; }
		Row& operator+=(const SubMatrix<1,N,R>& x) { Matrix<1,N,R>::operator+=(x); return *this;}
		Row& operator-=(const SubMatrix<1,N,R>& x) { Matrix<1,N,R>::operator-=(x); return *this;}
		Row& operator+=(const SubMatrix<1,N,const R>& x) { Matrix<1,N,R>::operator+=(x); return *this;}
		Row& operator-=(const SubMatrix<1,N,const R>& x) { Matrix<1,N,R>::operator-=(x); return *this;}
		Row& operator+=(const Matrix<1,N,R>& x) { Matrix<1,N,R>::operator+=(x); return *this;}
		Row& operator-=(const Matrix<1,N,R>& x) { Matrix<1,N,R>::operator-=(x); return *this;}
		
		Row operator+(const SubVector<N,R>& x) const { return Row(*this)+=x; }
		Row operator-(const SubVector<N,R>& x) const { return Row(*this)-=x; }
		Row operator+(const SubVector<N,const R>& x) const { return Row(*this)+=x; }
		Row operator-(const SubVector<N,const R>& x) const { return Row(*this)-=x; }
		Row operator+(const SubMatrix<1,N,R>& x) const { return Row(*this)+=x; }
		Row operator-(const SubMatrix<1,N,R>& x) const { return Row(*this)-=x; }
		Row operator+(const SubMatrix<1,N,const R>& x) const { return Row(*this)+=x; }
		Row operator-(const SubMatrix<1,N,const R>& x) const { return Row(*this)-=x; }
		Row operator+(const Matrix<1,N,R>& x) const { return Row(*this)+=x; }
		Row operator-(const Matrix<1,N,R>& x) const { return Row(*this)-=x; }
		
		bool operator==(const SubVector<N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		bool operator!=(const SubVector<N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		bool operator<(const SubVector<N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }
		bool operator==(const SubVector<N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		bool operator!=(const SubVector<N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		bool operator<(const SubVector<N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }
		bool operator==(const SubMatrix<1,N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[i][0]) return false; } return true; }
		bool operator!=(const SubMatrix<1,N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[i][0]) return true; } return false; }
		bool operator<(const SubMatrix<1,N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.cols[i][0]) return true; if(x.cols[i][0]<data[i]) return false; } return false; }
		bool operator==(const SubMatrix<1,N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[i][0]) return false; } return true; }
		bool operator!=(const SubMatrix<1,N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.cols[i][0]) return true; } return false; }
		bool operator<(const SubMatrix<1,N,const R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.cols[i][0]) return true; if(x.cols[i][0]<data[i]) return false; } return false; }
		bool operator==(const Matrix<1,N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return false; } return true; }
		bool operator!=(const Matrix<1,N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]!=x.data[i]) return true; } return false; }
		bool operator<(const Matrix<1,N,R>& x) const { size_t i=N; while(i!=0) { --i; if(data[i]<x.data[i]) return true; if(x.data[i]<data[i]) return false; } return false; }
		
		//! returns true if all elements are less than the corresponding element
		template<typename T> bool operator<<(const Matrix<N,1,T>& x) const { size_t i=N; while(i!=0) { --i; if(x.data[i] <= data[i]) return false; } return true; }
		template<typename T> bool operator<<(const SubVector<N,T>& x) const { size_t i=N; while(i!=0) { --i; if(x.data[i] <= data[i]) return false; } return true; }
		
		inline R& operator[](size_t i) { return data[i]; }
		inline const R& operator[](size_t i) const { return data[i]; }
		
		R norm() const ATTR_must_check { R ans=0; size_t i=N; while(i!=0) { const R x=data[--i]; ans+=x*x; } return std::sqrt(ans); }
		R sum() const ATTR_must_check { R ans=0; size_t i=N; while(i!=0) { ans+=data[--i]; } return ans; }
		R sumSq() const ATTR_must_check { R ans=0; size_t i=N; while(i!=0) { const R x=data[--i]; ans+=x*x; } return ans; }
		void abs() { size_t i=N; while(i!=0) { --i; data[i]=std::abs(data[i]); } }
		R max() const ATTR_must_check { if(N==0) return R(); R ans=data[N-1]; size_t i=N-1; while(i--!=0) if(ans<data[i]) ans=data[i]; return ans; }
		R min() const ATTR_must_check { if(N==0) return R(); R ans=data[N-1]; size_t i=N-1; while(i--!=0) if(ans>data[i]) ans=data[i]; return ans; }
		template<typename F> void apply(const F& f) { size_t i=N; while(i--!=0) data[i]=f(data[i]); }
		template<typename F> Row<N,R> map(const F& f) const { Row<N,R> ans; size_t i=N; while(i--!=0) ans.data[i]=f(data[i]); return ans; }
		
		template<typename T> void minimize(const Matrix<1,N,T>& x) { size_t i=N; while(i!=0) { --i; if(x.data[i] < data[i]) data[i]=x.data[i]; } }
		template<typename T> void minimize(const SubVector<N,T>& x) { size_t i=N; while(i!=0) { --i; if(x.data[i] < data[i]) data[i]=x.data[i]; } }
		template<typename T> void maximize(const Matrix<1,N,T>& x) { size_t i=N; while(i!=0) { --i; if(data[i] < x.data[i]) data[i]=x.data[i]; } }
		template<typename T> void maximize(const SubVector<N,T>& x) { size_t i=N; while(i!=0) { --i; if(data[i] < x.data[i]) data[i]=x.data[i]; } }
		
		inline Column<N,R> transpose() const ATTR_must_check {
			Column<N,R> ans=fmat_internal::NoInit();
			fmat_internal::tmemcpy<R>(ans.data,data,N);
			return ans;
		}

		inline std::string fmt(std::string const &numberFormat=defaultNumberFormat, 
			std::string const &firstLineStart="[",
			std::string const &nextLineStart="",
			std::string const &lastLineEnd="]",
			std::string const &rowBegin="",
			std::string const &elementSep=" ",
			std::string const &rowEnd="",
			std::string const &rowSep="") const
		{
			return fullfmt(&data[0],1,N,numberFormat,firstLineStart,nextLineStart,lastLineEnd,rowBegin,elementSep,rowEnd,rowSep);
		}

	protected:
		using Matrix<1,N,R>::data;
	};
	
	
	extern const Column<2> ZERO2; //!< a length 2 column vector with all zeros (could also just use Column<2>(), but this is more semantic)
	extern const Column<3> ZERO3; //!< a length 3 column vector with all zeros (could also just use Column<3>(), but this is more semantic)
	extern const Column<4> ZEROH; //!< a length 4 column vector representing zero in homogenous coordinates (0,0,0,1)
	
	extern const Column<2> UNIT2_X; //!< a length 2 column with '1' in the first dimension
	extern const Column<2> UNIT2_Y; //!< a length 2 column with '1' in the second dimension
	
	extern const Column<3> UNIT3_X; //!< a length 3 column with '1' in the first dimension
	extern const Column<3> UNIT3_Y; //!< a length 3 column with '1' in the second dimension
	extern const Column<3> UNIT3_Z; //!< a length 3 column with '1' in the third dimension
	
	//! generic packing of N values into a Column<N> (note assumes fmatReal, see packT()
	inline Column<2> pack(fmatReal x, fmatReal y) { Column<2> v=fmat_internal::NoInit(); v.data[0]=x; v.data[1]=y; return v; }
	inline Column<3> pack(fmatReal x, fmatReal y, fmatReal z) { Column<3> v=fmat_internal::NoInit(); v.data[0]=x; v.data[1]=y; v.data[2]=z; return v; }
	inline Column<4> pack(fmatReal x, fmatReal y, fmatReal z, fmatReal d) { Column<4> v=fmat_internal::NoInit(); v.data[0]=x; v.data[1]=y; v.data[2]=z; v.data[3]=d; return v; }
	
	// alternative implementation, tail recursion faster?
	/*template<typename R> Column<2,R> pack(R x, R y) { R data[] = {x,y}; return Column<2,R>(data); }
	template<typename R> Column<3,R> pack(R x, R y, R z) { R data[] = {x,y,z}; return Column<3,R>(data); }
	template<typename R> Column<4,R> pack(R x, R y, R z, R d) { R data[] = {x,y,z,d}; return Column<4,R>(data); }*/
	
	//! templated version to pack non fmatReal type, using a different name 'packT' so it needs to be explicitly called, e.g. pack(0,0,1) is generally intended as fmatReal, not int
	template<typename R> inline Column<2,R> packT(R x, R y) { Column<2,R> v=fmat_internal::NoInit(); v.data[0]=x; v.data[1]=y; return v; }
	template<typename R> inline Column<3,R> packT(R x, R y, R z) { Column<3,R> v=fmat_internal::NoInit(); v.data[0]=x; v.data[1]=y; v.data[2]=z; return v; }
	template<typename R> inline Column<4,R> packT(R x, R y, R z, R d) { Column<4,R> v=fmat_internal::NoInit(); v.data[0]=x; v.data[1]=y; v.data[2]=z; v.data[3]=d; return v; }
	
	//! packing columns: appending an element or concatenating two columns
	template<template<size_t N, typename R> class T, size_t N, typename R1, typename R2>
	Column<N+1,typename fmat_internal::unconst<R1>::type>
	pack(const T<N,R1>& x, R2 y) {
		Column<N+1,typename fmat_internal::unconst<R1>::type> data(x); data[N]=y; return data;
	}
	template<template<size_t N, typename R> class T, size_t N, typename R1, typename R2>
	Column<N+1,typename fmat_internal::unconst<R2>::type>
	pack(R1 x, const T<N,R2>& y) {
		Column<N+1,typename fmat_internal::unconst<R2>::type> data=fmat_internal::NoInit(); data[0]=x; SubVector<N>(data,1)=y; return data;
	}
	template<template<size_t N, typename R> class T1, template<size_t N, typename R> class T2, size_t N1, size_t N2, typename R1, typename R2>
	Column<N1+N2,typename fmat_internal::promotion_trait<R1,R2>::type>
	pack(const T1<N1,R1>& x, const T2<N2,R2>& y) {
		Column<N1+N2,typename fmat_internal::promotion_trait<R1,R2>::type> data(x); SubVector<N2>(data,N1)=y; return data;
	}
	
	// this is just to pick up subclasses
	template<size_t N, typename R1, typename R2>
	Column<N+1,typename fmat_internal::unconst<R1>::type> pack(const Column<N,R1>& x, R2 y) {
		Column<N+1,typename fmat_internal::unconst<R1>::type> data(x); data[N]=y; return data;
	}
	template<size_t N, typename R1, typename R2>
	Column<N+1,typename fmat_internal::unconst<R2>::type> pack(R1 x, const Column<N,R2>& y) {
		Column<N+1,typename fmat_internal::unconst<R2>::type> data=fmat_internal::NoInit(); data[0]=x; SubVector<N,R2>(data,1)=y; return data;
	}
	template<size_t N1, size_t N2, typename R>
	Column<N1+N2,typename fmat_internal::unconst<R>::type> pack(const Column<N1,R>& x, const Column<N2,R>& y) {
		Column<N1+N2,typename fmat_internal::unconst<R>::type> data(x); SubVector<N2>(data,N1)=y; return data;
	}

	// override: packing a row should return a row
	template<size_t N, typename R1, typename R2>
	Row<N+1,typename fmat_internal::unconst<R1>::type> pack(const Row<N,R1>& x, R2 y) {
		Row<N+1,typename fmat_internal::unconst<R1>::type> data(x); data[N]=y; return data;
	}
	template<size_t N, typename R1, typename R2>
	Row<N+1,typename fmat_internal::unconst<R2>::type> pack(R1 x, const Row<N,R2>& y) {
		Row<N+1,typename fmat_internal::unconst<R2>::type> data=fmat_internal::NoInit(); data[0]=x; SubVector<N,R2>(data,1)=y; return data;
	}
	template<size_t N1, size_t N2, typename R>
	Row<N1+N2,typename fmat_internal::unconst<R>::type> pack(const Row<N1,R>& x, const Row<N2,R>& y) {
		Row<N1+N2,typename fmat_internal::unconst<R>::type> data(x); SubVector<N2>(data,N1)=y; return data;
	}
	
	
	template<size_t N, typename R> inline std::ostream& operator<<(std::ostream& os, const SubVector<N,R>& x) {
		return os << x.fmt(); }

	template<size_t H, size_t W, typename R> inline std::ostream& operator<<(std::ostream& os, const SubMatrix<H,W,R>& x) {
		return os << x.fmt(); }

	template<size_t H, size_t W, typename R> inline std::ostream& operator<<(std::ostream& os, const Matrix<H,W,R>& x) {
		return os << x.fmt();	}

	template<size_t N, typename R> inline std::ostream& operator<<(std::ostream& os, const Column<N,R>& x) {
		return os << x.fmt(); }

	template<size_t N, typename R> inline std::ostream& operator<<(std::ostream& os, const Row<N,R>& x) {
		return os << x.fmt(); }


	
	template<template<size_t H, size_t W, typename R> class T1, template<size_t H, size_t W, typename R> class T2, size_t H, size_t N, size_t W, typename R1, typename R2>
	inline Matrix<H,W,typename fmat_internal::promotion_trait<R1,R2>::type>
	operator*(const T1<H,N,R1>& a, const T2<N,W,R2>& b) {
		return fmat_internal::mmops<T1<H,N,R1>,T2<N,W,R2> >::multiply(a,b);
	}
	
	/*template<template<size_t H, size_t W, typename R> class T1, template<size_t H, size_t W, typename R> class T2, size_t D, typename R1, typename R2>
	inline Matrix<D,D,typename fmat_internal::promotion_trait<R1,R2>::type>
	operator*(const T1<D,D,R1>& a, const T2<D,D,R2>& b) {
		return fmat_internal::mmops<T1<D,D,R1>,T2<D,D,R2> >::multiply(a,b);
	}*/
	
	template<template<size_t H, size_t W, typename R> class T1, template<size_t H, size_t W, typename R> class T2, size_t D, typename R1, typename R2>
	inline T1<D,D,R1>&
	operator*=(T1<D,D,R1>& a, const T2<D,D,R2>& b) {
		a=fmat_internal::mmops<T1<D,D,R1>,T2<D,D,R2> >::multiply(a,b);
		return a;
	}
	
	template<template<size_t H, size_t W, typename R> class T2, size_t D, typename R1, typename R2>
	inline const SubMatrix<D,D,R1>&
	operator*=(const SubMatrix<D,D,R1>& a, const T2<D,D,R2>& b) {
		a=fmat_internal::mmops<SubMatrix<D,D,R1>,T2<D,D,R2> >::multiply(a,b);
		return a;
	}
	
	template<template<size_t H, size_t W, typename R> class T1, template<size_t N, typename R> class T2, size_t H, size_t N, typename R1, typename R2>
	inline Column<H,typename fmat_internal::promotion_trait<R1,R2>::type>
	operator*(const T1<H,N,R1>& a, const T2<N,R2>& b) {
		return fmat_internal::mvops<T1<H,N,R1>,T2<N,R2> >::multiply(a,b);
	}
	
	template<template<size_t W, typename R> class T1, template<size_t H, size_t W, typename R> class T2, size_t N, size_t W, typename R1, typename R2>
	inline Row<W,typename fmat_internal::promotion_trait<R1,R2>::type>
	operator*(const T1<N,R1>& a, const T2<N,W,R2>& b) {
		return fmat_internal::mvops<T2<N,W,R2>,T1<N,R1> >::multiply(a,b);
	}
	
	template<template<size_t N, typename R> class T, size_t N, typename R>
	inline fmat::Column<N,typename fmat_internal::unconst<R>::type>
	abs(const T<N,R>& v) {
		fmat::Column<N,typename fmat_internal::unconst<R>::type> ans=fmat_internal::NoInit();
		for(size_t i=N; i!=0; ) {
			--i;
			ans[i]=std::abs(v[i]);
		}
		return ans;
	}

	template<template<size_t H, size_t W, typename R> class T, size_t H, size_t W, typename R>
	inline fmat::Matrix<H,W,typename fmat_internal::unconst<R>::type>
	abs(const T<H,W,R>& m) {
		typedef typename fmat_internal::unconst<R>::type out_t;
		fmat::Matrix<H,W,out_t> ans=fmat_internal::NoInit();
		out_t* ad=&ans(0,0);
		const R* md=&m(0,0);
		for(size_t i=H*W; i!=0; ) {
			--i;
			ad[i]=std::abs(md[i]);
		}
		return ans;
	}
	
	template<typename R> inline R atan(const Column<2,R>& v) { return std::atan2(v[1],v[0]); }
	template<typename R> inline R atan(const Row<2,R>& v) { return std::atan2(v[1],v[0]); }
	template<typename R> inline R atan(const SubVector<2,R>& v) { return std::atan2(v[1],v[0]); }
	
	template<template<size_t N, typename R> class T1, template<size_t N, typename R> class T2, size_t N, typename R1, typename R2>
	typename fmat_internal::promotion_trait<R1,R2>::type
	dotProduct(const T1<N,R1>& a, const T2<N,R2>& b) {
		typename fmat_internal::promotion_trait<R1,R2>::type ans=0;
		size_t i=N;
		while(i--!=0)
			ans+=a[i]*b[i];
		return ans; 
	}
	// this version of dotProduct() gets picked up when using subclasses, which don't match the template forms above
	template<class Ta, class Tb>
	inline typename fmat_internal::promotion_trait<typename Ta::storage_t,typename Tb::storage_t>::type
	dotProduct(const Ta& a, const Tb& b) {
		(void)sizeof(fmat_internal::CompileTimeAssert<Length_mismatch_or_non_vector,Ta::CAP==Tb::CAP && Ta::HEIGHT==Ta::CAP && Tb::HEIGHT==Tb::CAP>);
		typename fmat_internal::promotion_trait<typename Ta::storage_t,typename Tb::storage_t>::type ans=0;
		size_t i=Ta::CAP;
		while(i--!=0)
			ans+=a[i]*b[i];
		return ans; 
	}
	
	template<class Ta, class Tb>
	inline Column<3,typename fmat_internal::promotion_trait<typename Ta::storage_t,typename Tb::storage_t>::type>
	crossProduct(const Ta& a, const Tb& b) {
		return packT<typename fmat_internal::promotion_trait<typename Ta::storage_t,typename Tb::storage_t>::type>(
			a[1] * b[2] - a[2] * b[1],
			a[2] * b[0] - a[0] * b[2],
			a[0] * b[1] - a[1] * b[0]
		);
	}
	
	template<template<size_t H, size_t W, typename R> class M, typename R>
	typename fmat_internal::unconst<R>::type
	determinant(const M<2,2,R>& m) {
		return m(0,0) * m(1,1) - m(0,1) * m(1,0);
	}
	
	template<template<size_t H, size_t W, typename R> class M, typename R>
	typename fmat_internal::unconst<R>::type
	determinant(const M<3,3,R>& m) {
		return m(0,0) * m(1,1) * m(2,2) + m(0,1) * m(1,2) * m(2,0) + m(0,2) * m(1,0) * m(2,1)
		- m(0,0) * m(1,2) * m(2,1) - m(0,1) * m(1,0) * m(2,2) - m(0,2) * m(1,1) * m(2,0);
	}
	
	//! Computes and returns the inverse of a square matrix using Gauss-Jordan elimination
	/*! The computation is done with column-wise operations for computational efficiency.
	 *  You can think of this as doing the inverse in transposed space: (mᵀ)⁻¹ == (m⁻¹)ᵀ */
	template<typename M>
	Matrix<M::HEIGHT,M::WIDTH,typename fmat_internal::unconst<typename M::storage_t>::type >
	invert(const M& m) {
		(void)sizeof(fmat_internal::CompileTimeAssert<NonSquare_Matrix,M::WIDTH==M::HEIGHT>);
		typedef typename fmat_internal::unconst<typename M::storage_t>::type R;
		const size_t DIM=M::HEIGHT;
		
		Matrix<DIM*2,DIM,R> x(m);
		for(unsigned int c=0; c<DIM; ++c)
			x(DIM+c,c)=1;
		
		for(unsigned int c=0; c<DIM; ++c) {
			// find pivot
			R mx=std::abs(x(c,c));
			unsigned int mxi=c;
			for(unsigned int r=c+1; r<DIM; ++r) {
				const R v=std::abs(x(c,r));
				if(v>mx) {
					mx=v;
					mxi=r;
				}
			}
			if(mx<std::numeric_limits<float>::epsilon())
				throw std::underflow_error("low rank matrix, non-invertable");
			
			// swap to put pivot in position
			Column<DIM*2,R> tmp = x.column(c);
			x.column(c) = x.column(mxi);
			x.column(mxi) = tmp;
			
			{
				const R p=x(c,c);
				for(unsigned int c2=c+1; c2<DIM*2; ++c2)
					x(c2,c)/=p;
				x(c,c)=1;
			}
			
			for(unsigned int r=c+1; r<DIM; ++r) {
				const R p = x(c,r);
				for(unsigned int c2=c+1; c2<DIM*2; ++c2)
					x(c2,r)-=x(c2,c)*p;
				x(c,r)=0;
			}
		}
		for(unsigned int c=0; c<DIM; ++c) {
			for(unsigned int r=0; r<c; ++r) {
				const R p = x(c,r);
				for(unsigned int c2=c+1; c2<DIM*2; ++c2)
					x(c2,r)-=x(c2,c)*p;
				x(c,r)=0;
			}
		}
		return SubMatrix<DIM,DIM,R>(x,DIM,0);
	}
	
	//! returns the determinant for 2x2 matrices
	template<template<size_t H, size_t W, typename R> class T, typename R>
	inline R det(const T<2,2,R>& m) {
		return m(0,0)*m(1,1) - m(1,0)*m(0,1);
	}
	
	//! returns the determinant for 3x3 matrices
	template<template<size_t H, size_t W, typename R> class T, typename R>
	inline R det(const T<3,3,R>& m) {
		const R cofactor0 = m(1,1)*m(2,2) - m(1,2)*m(2,1);
		const R cofactor1 = m(1,2)*m(2,0) - m(1,0)*m(2,2);
		const R cofactor2 = m(1,0)*m(2,1) - m(1,1)*m(2,0);
        return m(0,0)*cofactor0 + m(0,1)*cofactor1 + m(0,2)*cofactor2;
	}
	// todo: det() for higher dimensions
	
	
	// Performance specializations:
	template<template<size_t N, typename R> class V1, template<size_t N, typename R> class V2, typename R1, typename R2>
	inline typename fmat_internal::promotion_trait<R1,R2>::type dotProduct(const V1<2,R1>& a, const V2<2,R2>& b) { return a[0]*b[0] + a[1]*b[1]; }
	template<template<size_t N, typename R> class V1, template<size_t N, typename R> class V2, typename R1, typename R2>
	inline typename fmat_internal::promotion_trait<R1,R2>::type dotProduct(const V1<3,R1>& a, const V2<3,R2>& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
	template<template<size_t N, typename R> class V1, template<size_t N, typename R> class V2, typename R1, typename R2>
	inline typename fmat_internal::promotion_trait<R1,R2>::type dotProduct(const V1<4,R1>& a, const V2<4,R2>& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]; }
	
	namespace fmat_internal {
		inline float hypot(float a, float b) ATTR_pure ATTR_always_inline;
		inline float hypot(float a, float b) { return ::hypotf(a,b); }
		inline double hypot(double a, double b) ATTR_pure ATTR_always_inline;
		inline double hypot(double a, double b) { return ::hypot(a,b); }
#ifndef PLATFORM_APERIOS
		inline long double hypot(long double a, long double b) ATTR_pure ATTR_always_inline;
		inline long double hypot(long double a, long double b) { return ::hypotl(a,b); }
#endif
		template<class T> T hypot(T a, T b) { return std::sqrt(a*a,b*b); }
	}
	
	template<> inline fmatReal Column<2,fmatReal>::norm() const { return fmat_internal::hypot(data[0],data[1]); }
	template<> inline fmatReal Column<3,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal Column<4,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }
	template<> inline fmatReal SubVector<2,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1]); }
	template<> inline fmatReal SubVector<3,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal SubVector<4,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }
	template<> inline fmatReal SubVector<2,const fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1]); }
	template<> inline fmatReal SubVector<3,const fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal SubVector<4,const fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }
	template<> inline fmatReal Row<2,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1]); }
	template<> inline fmatReal Row<3,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal Row<4,fmatReal>::norm() const { return std::sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }

	template<> inline fmatReal Column<2,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1]); }
	template<> inline fmatReal Column<3,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal Column<4,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }
	template<> inline fmatReal SubVector<2,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1]); }
	template<> inline fmatReal SubVector<3,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal SubVector<4,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }
	template<> inline fmatReal SubVector<2,const fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1]); }
	template<> inline fmatReal SubVector<3,const fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal SubVector<4,const fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }
	template<> inline fmatReal Row<2,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1]); }
	template<> inline fmatReal Row<3,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2]); }
	template<> inline fmatReal Row<4,fmatReal>::sumSq() const { return (data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); }
	

	template<> inline Column<2,fmatReal>& Column<2,fmatReal>::operator+=(const SubVector<2,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; return *this; }
	template<> inline Column<3,fmatReal>& Column<3,fmatReal>::operator+=(const SubVector<3,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; data[2]+=x.data[2]; return *this; }
	template<> inline Column<4,fmatReal>& Column<4,fmatReal>::operator+=(const SubVector<4,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; data[2]+=x.data[2]; data[3]+=x.data[3]; return *this; }
	template<> inline Matrix<2,1,fmatReal>& Matrix<2,1,fmatReal>::operator+=(const Matrix<2,1,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; return *this; }
	template<> inline Matrix<3,1,fmatReal>& Matrix<3,1,fmatReal>::operator+=(const Matrix<3,1,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; data[2]+=x.data[2]; return *this; }
	template<> inline Matrix<4,1,fmatReal>& Matrix<4,1,fmatReal>::operator+=(const Matrix<4,1,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; data[2]+=x.data[2]; data[3]+=x.data[3]; return *this; }
	template<> inline Row<2,fmatReal>& Row<2,fmatReal>::operator+=(const SubVector<2,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; return *this; }
	template<> inline Row<3,fmatReal>& Row<3,fmatReal>::operator+=(const SubVector<3,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; data[2]+=x.data[2]; return *this; }
	template<> inline Row<4,fmatReal>& Row<4,fmatReal>::operator+=(const SubVector<4,fmatReal>& x) { data[0]+=x.data[0]; data[1]+=x.data[1]; data[2]+=x.data[2]; data[3]+=x.data[3]; return *this; }

	template<> inline Column<2,fmatReal>& Column<2,fmatReal>::operator-=(const SubVector<2,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; return *this; }
	template<> inline Column<3,fmatReal>& Column<3,fmatReal>::operator-=(const SubVector<3,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; data[2]-=x.data[2]; return *this; }
	template<> inline Column<4,fmatReal>& Column<4,fmatReal>::operator-=(const SubVector<4,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; data[2]-=x.data[2]; data[3]-=x.data[3]; return *this; }
	template<> inline Matrix<2,1,fmatReal>& Matrix<2,1,fmatReal>::operator-=(const Matrix<2,1,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; return *this; }
	template<> inline Matrix<3,1,fmatReal>& Matrix<3,1,fmatReal>::operator-=(const Matrix<3,1,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; data[2]-=x.data[2]; return *this; }
	template<> inline Matrix<4,1,fmatReal>& Matrix<4,1,fmatReal>::operator-=(const Matrix<4,1,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; data[2]-=x.data[2]; data[3]-=x.data[3]; return *this; }
	template<> inline Row<2,fmatReal>& Row<2,fmatReal>::operator-=(const SubVector<2,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; return *this; }
	template<> inline Row<3,fmatReal>& Row<3,fmatReal>::operator-=(const SubVector<3,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; data[2]-=x.data[2]; return *this; }
	template<> inline Row<4,fmatReal>& Row<4,fmatReal>::operator-=(const SubVector<4,fmatReal>& x) { data[0]-=x.data[0]; data[1]-=x.data[1]; data[2]-=x.data[2]; data[3]-=x.data[3]; return *this; }
	
	template<> inline const SubVector<2,fmatReal>& SubVector<2,fmatReal>::operator*=(fmatReal x) const { data[0]*=x; data[1]*=x; return *this; }
	template<> inline const SubVector<3,fmatReal>& SubVector<3,fmatReal>::operator*=(fmatReal x) const { data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline const SubVector<4,fmatReal>& SubVector<4,fmatReal>::operator*=(fmatReal x) const { data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }
	template<> inline Matrix<2,1,fmatReal>& Matrix<2,1,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; return *this; }
	template<> inline Matrix<3,1,fmatReal>& Matrix<3,1,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline Matrix<4,1,fmatReal>& Matrix<4,1,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }
	template<> inline Column<2,fmatReal>& Column<2,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; return *this; }
	template<> inline Column<3,fmatReal>& Column<3,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline Column<4,fmatReal>& Column<4,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }
	template<> inline Row<2,fmatReal>& Row<2,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; return *this; }
	template<> inline Row<3,fmatReal>& Row<3,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline Row<4,fmatReal>& Row<4,fmatReal>::operator*=(fmatReal x) { data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }

	template<> inline const SubVector<2,fmatReal>& SubVector<2,fmatReal>::operator/=(fmatReal x) const { x=1/x; data[0]*=x; data[1]*=x; return *this; }
	template<> inline const SubVector<3,fmatReal>& SubVector<3,fmatReal>::operator/=(fmatReal x) const { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline const SubVector<4,fmatReal>& SubVector<4,fmatReal>::operator/=(fmatReal x) const { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }
	template<> inline Matrix<2,1,fmatReal>& Matrix<2,1,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; return *this; }
	template<> inline Matrix<3,1,fmatReal>& Matrix<3,1,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline Matrix<4,1,fmatReal>& Matrix<4,1,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }
	template<> inline Column<2,fmatReal>& Column<2,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; return *this; }
	template<> inline Column<3,fmatReal>& Column<3,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline Column<4,fmatReal>& Column<4,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }
	template<> inline Row<2,fmatReal>& Row<2,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; return *this; }
	template<> inline Row<3,fmatReal>& Row<3,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; return *this; }
	template<> inline Row<4,fmatReal>& Row<4,fmatReal>::operator/=(fmatReal x) { x=1/x; data[0]*=x; data[1]*=x; data[2]*=x; data[3]*=x; return *this; }

	namespace fmat_internal {
		
		template<template<size_t H, size_t W, typename R> class T1, template<size_t N, typename R> class T2, typename R1, typename R2>
		struct mvops<T1<2,2,R1>,T2<2,R2> > {
			typedef typename fmat_internal::promotion_trait<R1,R2>::type R;
			inline static Column<2,R> multiply(const T1<2,2,R1>& ma, const T2<2,R2>& b) {
				Column<2,R> ans=fmat_internal::NoInit();
				const R1 *a=&ma(0,0);
				ans[0] = a[0*2+0]*b[0] + a[1*2+0]*b[1];
				ans[1] = a[0*2+1]*b[0] + a[1*2+1]*b[1];
				return ans;
			}
			inline static Row<2,R> multiply(const T2<2,R2>& b, const T1<2,2,R1>& ma) {
				Row<2,R> ans=fmat_internal::NoInit();
				const R1 *a=&ma(0,0);
				ans[0] = a[0*2+0]*b[0] + a[0*2+1]*b[1];
				ans[1] = a[1*2+0]*b[0] + a[1*2+1]*b[1];
				return ans;
			}
		};
		template<template<size_t H, size_t W, typename R> class T1, template<size_t N, typename R> class T2, typename R1, typename R2>
		struct mvops<T1<3,3,R1>,T2<3,R2> > {
			typedef typename fmat_internal::promotion_trait<R1,R2>::type R;
			inline static Column<3,R> multiply(const T1<3,3,R1>& ma, const T2<3,R2>& b) {
				Column<3,R> ans=fmat_internal::NoInit();
				const R *a=&ma(0,0);
				ans[0] = a[0*3+0]*b[0] + a[1*3+0]*b[1] + a[2*3+0]*b[2];
				ans[1] = a[0*3+1]*b[0] + a[1*3+1]*b[1] + a[2*3+1]*b[2];
				ans[2] = a[0*3+2]*b[0] + a[1*3+2]*b[1] + a[2*3+2]*b[2];
				return ans;
			}
			inline static Row<3,R> multiply(const T2<3,R2>& b, const T1<3,3,R1>& ma) {
				Row<3,R> ans=fmat_internal::NoInit();
				const R *a=&ma(0,0);
				ans[0] = a[0*3+0]*b[0] + a[0*3+1]*b[1] + a[0*3+2]*b[2];
				ans[1] = a[1*3+0]*b[0] + a[1*3+1]*b[1] + a[1*3+2]*b[2];
				ans[2] = a[2*3+0]*b[0] + a[2*3+1]*b[1] + a[2*3+2]*b[2];
				return ans;
			}
		};
		template<template<size_t H, size_t W, typename R> class T1, template<size_t N, typename R> class T2, typename R1, typename R2>
		struct mvops<T1<4,4,R1>,T2<4,R2> > {
			typedef typename fmat_internal::promotion_trait<R1,R2>::type R;
			inline static Column<4,R> multiply(const T1<4,4,R1>& ma, const T2<4,R2>& b) {
				Column<4,R> ans=fmat_internal::NoInit();
				const R *a=&ma(0,0);
				ans[0] = a[0*4+0]*b[0] + a[1*4+0]*b[1] + a[2*4+0]*b[2] + a[3*4+0]*b[3];
				ans[1] = a[0*4+1]*b[0] + a[1*4+1]*b[1] + a[2*4+1]*b[2] + a[3*4+1]*b[3];
				ans[2] = a[0*4+2]*b[0] + a[1*4+2]*b[1] + a[2*4+2]*b[2] + a[3*4+2]*b[3];
				ans[3] = a[0*4+3]*b[0] + a[1*4+3]*b[1] + a[2*4+3]*b[2] + a[3*4+3]*b[3];
				return ans;
			}
			inline static Row<4,R> multiply(const T2<4,R2>& b, const T1<4,4,R1>& ma) {
				Row<4,R> ans=fmat_internal::NoInit();
				const R *a=&ma(0,0);
				ans[0] = a[0*4+0]*b[0] + a[0*4+1]*b[1] + a[0*4+2]*b[2] + a[0*4+3]*b[3];
				ans[1] = a[1*4+0]*b[0] + a[1*4+1]*b[1] + a[1*4+2]*b[2] + a[1*4+3]*b[3];
				ans[2] = a[2*4+0]*b[0] + a[2*4+1]*b[1] + a[2*4+2]*b[2] + a[2*4+3]*b[3];
				ans[3] = a[3*4+0]*b[0] + a[3*4+1]*b[1] + a[3*4+2]*b[2] + a[3*4+3]*b[3];
				return ans;
			}
		};

		template<template<size_t H, size_t W, typename R> class T1, template<size_t H, size_t W, typename R> class T2, typename R1, typename R2>
		struct mmops<T1<2,2,R1>,T2<2,2,R2> > {
			typedef typename fmat_internal::promotion_trait<R1,R2>::type R;
			inline static Matrix<2,2,R> multiply(const T1<2,2,R1>& ma, const T2<2,2,R2>& mb) {
				Matrix<2,2,R> mans=fmat_internal::NoInit();
				R *ans=&mans(0,0);
				const R1 *a=&ma(0,0);
				const R2 *b=&mb(0,0);
				ans[0+2*0] = a[0+2*0]*b[0+2*0] + a[0+2*1]*b[1+2*0];
				ans[1+2*0] = a[1+2*0]*b[0+2*0] + a[1+2*1]*b[1+2*0];
				ans[0+2*1] = a[0+2*0]*b[0+2*1] + a[0+2*1]*b[1+2*1];
				ans[1+2*1] = a[1+2*0]*b[0+2*1] + a[1+2*1]*b[1+2*1];
				return mans;
			}
		};
		template<template<size_t H, size_t W, typename R> class T1, template<size_t H, size_t W, typename R> class T2, typename R1, typename R2>
		struct mmops<T1<3,3,R1>,T2<3,3,R2> > {
			typedef typename fmat_internal::promotion_trait<R1,R2>::type R;
			inline static Matrix<3,3,R> multiply(const T1<3,3,R1>& ma, const T2<3,3,R2>& mb) {
				Matrix<3,3,R> mans=fmat_internal::NoInit();
				R *ans=&mans(0,0);
				const R1 *a=&ma(0,0);
				const R2 *b=&mb(0,0);
				ans[0+3*0] = a[0+3*0]*b[0+3*0] + a[0+3*1]*b[1+3*0] + a[0+3*2]*b[2+3*0];
				ans[1+3*0] = a[1+3*0]*b[0+3*0] + a[1+3*1]*b[1+3*0] + a[1+3*2]*b[2+3*0];
				ans[2+3*0] = a[2+3*0]*b[0+3*0] + a[2+3*1]*b[1+3*0] + a[2+3*2]*b[2+3*0];
				ans[0+3*1] = a[0+3*0]*b[0+3*1] + a[0+3*1]*b[1+3*1] + a[0+3*2]*b[2+3*1];
				ans[1+3*1] = a[1+3*0]*b[0+3*1] + a[1+3*1]*b[1+3*1] + a[1+3*2]*b[2+3*1];
				ans[2+3*1] = a[2+3*0]*b[0+3*1] + a[2+3*1]*b[1+3*1] + a[2+3*2]*b[2+3*1];
				ans[0+3*2] = a[0+3*0]*b[0+3*2] + a[0+3*1]*b[1+3*2] + a[0+3*2]*b[2+3*2];
				ans[1+3*2] = a[1+3*0]*b[0+3*2] + a[1+3*1]*b[1+3*2] + a[1+3*2]*b[2+3*2];
				ans[2+3*2] = a[2+3*0]*b[0+3*2] + a[2+3*1]*b[1+3*2] + a[2+3*2]*b[2+3*2];
				return mans;
			}
		};
		template<template<size_t H, size_t W, typename R> class T1, template<size_t H, size_t W, typename R> class T2, typename R1, typename R2>
		struct mmops<T1<4,4,R1>,T2<4,4,R2> > {
			typedef typename fmat_internal::promotion_trait<R1,R2>::type R;
			inline static Matrix<4,4,R> multiply(const T1<4,4,R1>& ma, const T2<4,4,R2>& mb) {
				Matrix<4,4,R> mans=fmat_internal::NoInit();
				R *ans=&mans(0,0);
				const R1 *a=&ma(0,0);
				const R2 *b=&mb(0,0);
				ans[0+4*0] = a[0+4*0]*b[0+4*0] + a[0+4*1]*b[1+4*0] + a[0+4*2]*b[2+4*0] + a[0+4*3]*b[3+4*0];
				ans[1+4*0] = a[1+4*0]*b[0+4*0] + a[1+4*1]*b[1+4*0] + a[1+4*2]*b[2+4*0] + a[1+4*3]*b[3+4*0];
				ans[2+4*0] = a[2+4*0]*b[0+4*0] + a[2+4*1]*b[1+4*0] + a[2+4*2]*b[2+4*0] + a[2+4*3]*b[3+4*0];
				ans[3+4*0] = a[3+4*0]*b[0+4*0] + a[3+4*1]*b[1+4*0] + a[3+4*2]*b[2+4*0] + a[3+4*3]*b[3+4*0];
				ans[0+4*1] = a[0+4*0]*b[0+4*1] + a[0+4*1]*b[1+4*1] + a[0+4*2]*b[2+4*1] + a[0+4*3]*b[3+4*1];
				ans[1+4*1] = a[1+4*0]*b[0+4*1] + a[1+4*1]*b[1+4*1] + a[1+4*2]*b[2+4*1] + a[1+4*3]*b[3+4*1];
				ans[2+4*1] = a[2+4*0]*b[0+4*1] + a[2+4*1]*b[1+4*1] + a[2+4*2]*b[2+4*1] + a[2+4*3]*b[3+4*1];
				ans[3+4*1] = a[3+4*0]*b[0+4*1] + a[3+4*1]*b[1+4*1] + a[3+4*2]*b[2+4*1] + a[3+4*3]*b[3+4*1];
				ans[0+4*2] = a[0+4*0]*b[0+4*2] + a[0+4*1]*b[1+4*2] + a[0+4*2]*b[2+4*2] + a[0+4*3]*b[3+4*2];
				ans[1+4*2] = a[1+4*0]*b[0+4*2] + a[1+4*1]*b[1+4*2] + a[1+4*2]*b[2+4*2] + a[1+4*3]*b[3+4*2];
				ans[2+4*2] = a[2+4*0]*b[0+4*2] + a[2+4*1]*b[1+4*2] + a[2+4*2]*b[2+4*2] + a[2+4*3]*b[3+4*2];
				ans[3+4*2] = a[3+4*0]*b[0+4*2] + a[3+4*1]*b[1+4*2] + a[3+4*2]*b[2+4*2] + a[3+4*3]*b[3+4*2];
				ans[0+4*3] = a[0+4*0]*b[0+4*3] + a[0+4*1]*b[1+4*3] + a[0+4*2]*b[2+4*3] + a[0+4*3]*b[3+4*3];
				ans[1+4*3] = a[1+4*0]*b[0+4*3] + a[1+4*1]*b[1+4*3] + a[1+4*2]*b[2+4*3] + a[1+4*3]*b[3+4*3];
				ans[2+4*3] = a[2+4*0]*b[0+4*3] + a[2+4*1]*b[1+4*3] + a[2+4*2]*b[2+4*3] + a[2+4*3]*b[3+4*3];
				ans[3+4*3] = a[3+4*0]*b[0+4*3] + a[3+4*1]*b[1+4*3] + a[3+4*2]*b[2+4*3] + a[3+4*3]*b[3+4*3];
				return mans;
			}
		};
	}

	extern template class SubVector<3,fmatReal>;
	extern template class Column<3,fmatReal>;
	extern template class SubMatrix<3,3,fmatReal>;
	extern template class Matrix<3,4,fmatReal>;

} // namespace

#endif

/*! @file
 * @brief Defines fmat namespace, for fixed-dimension matrix computation
 * @author Ethan Tira-Thompson (ejt) (Creator)
 */
