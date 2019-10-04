#include "fmat.h"

namespace fmat {
	std::string defaultNumberFormat="%g";
	
	template class SubVector<3,fmatReal>;
	template class Column<3,fmatReal>;
	template class SubMatrix<3,3,fmatReal>;
	template class Matrix<3,4,fmatReal>;

	const Column<2> ZERO2;
	const Column<3> ZERO3;
	const Column<4> ZEROH(fmat::pack(0,0,0,1));
	
	const Column<2> UNIT2_X(fmat::pack(1,0));
	const Column<2> UNIT2_Y(fmat::pack(0,1));
	
	const Column<3> UNIT3_X(fmat::pack(1,0,0));
	const Column<3> UNIT3_Y(fmat::pack(0,1,0));
	const Column<3> UNIT3_Z(fmat::pack(0,0,1));
	
	template<typename T> const QuaternionT<T> QuaternionT<T>::IDENTITY;
	template class QuaternionT<float>;
	template class QuaternionT<double>;
	
	template<typename T> const TransformT<T> TransformT<T>::IDENTITY;
	template class TransformT<float>;
	template class TransformT<double>;
}
