#include <iostream>
#include "Shared/newmat/newmat.h"
#include "Shared/newmat/newmatio.h"
#include "Shared/fmatSpatial.h"
#include "Shared/TimeET.h"

using namespace std;


//ostream& operator<<(ostream& s, const NEWMAT::BaseMatrix& X);

void testConstSub(const fmat::SubVector<4,const float>& v) {
	std::cout << "Const: " << v << std::endl;
}

void testSubCall(const fmat::SubVector<3,const float>& v) {
	fmat::Column<3> x(v);
	x+=1;
}

void testColCall(const fmat::Column<3>& v) {
	fmat::Column<3> x(v);
	x+=1;
}

int main(int argc, char ** argv) {
	const int TESTS = argc>1 ? atoi(argv[1]) : 40000000;
	float v1[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0, 0, 0, 1 };
	float v2[16] = { 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 0, 0, 0, 1 };
	
	{
		float ang=20*M_PI/180;
		fmat::Quaternion q = fmat::Quaternion::fromAxisAngle(fmat::pack(1,1,3), ang);
		fmat::Matrix<3,3> m1 = q * fmat::Matrix<3,3>::identity();
		fmat::Transform m2 = q * fmat::Transform::identity();
		fmat::Column<3> v1 = q * fmat::pack(1,1,3);
		fmat::Column<3> v2 = q * fmat::pack(0,0,1);
		//fmat::Column<4> v3 = q * fmat::pack(1,1,3,1);
		//fmat::Column<4> v4 = q * fmat::pack(0,0,1,1);
		std::cout << m2 << '\n' << v1 << '\n' << v2 << /*'\n' << v3 << '\n' << v4 <<*/ std::endl;
		std::cout << m1*fmat::pack(1,1,3) << std::endl;
		std::cout << std::endl;
	}
	
	
	/*Without optimization:
	Passing:        testColCall:    testSubCall:
	SubVector       10.96           7.40
	Column          6.37            7.42

	With -O2:
	Passing:        testColCall:    testSubCall:
	SubVector       0.430           0.352
	Column          0.307           0.352

	So, I'm a little split; it'd be nice to get the ~15% savings by passing and receiving Columns everywhere, but that's at the risk of incurring an equally significant penalty if we need to make a temp Column from a SubVector (or other source, e.g. Row or float*).  So it's all kind of a wash performancewise.

	{
		fmat::Column<3> v;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			testColCall(v);
		cout << "column function call: " << t.Age() << endl;
	}
	{
		fmat::Column<3> v;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			testSubCall(v);
		cout << "subvector function call: " << t.Age() << endl;
	}
	{
		fmat::Column<3> v;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			testColCall(v);
		cout << "column function call: " << t.Age() << endl;
	}
	{
		fmat::Column<3> v;
		fmat::SubVector<3> s(v);
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			testColCall(s);
		cout << "subvector to column function call: " << t.Age() << endl;
	}
	{
		fmat::Column<3> v;
		fmat::SubVector<3> s(v);
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			testSubCall(s);
		cout << "subvector to subvector function call: " << t.Age() << endl;
	}
	{
		fmat::Row<3> v;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			//testColCall(fmat::SubVector<3>(v));
			testColCall(v.transpose());
		cout << "row to column function call: " << t.Age() << endl;
	}
	{
		fmat::Column<3> v;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			testSubCall(v);
		cout << "row to subvector function call: " << t.Age() << endl;
	}*/
	
	{
		fmat::Column<4> p_b = fmat::pack(0,1,1,3);
		fmat::Column<3> r_j = fmat::pack(0,numeric_limits<float>::epsilon(),1);
		fmat::Transform tr(fmat::rotationX<3,float>(45*M_PI/180),fmat::pack(0,0,10));
		std::cout << "transform:\n" << tr << std::endl;
		
		p_b/=fmat::SubVector<3>(p_b).norm();
		fmat::Column<3> pn_b(p_b);
		fmat::Column<3> ro_b = tr.translation();
		fmat::Column<3> rv_b = tr.rotation() * r_j;
		float dist = p_b[3] - fmat::dotProduct(ro_b,pn_b);
		float align = fmat::dotProduct(rv_b,pn_b);
		fmat::Column<4> hit;
		// testing conversions...
		fmat::Row<3> hitr = fmat::SubVector<3>(hit);
		fmat::Column<3> hitc((fmat::SubVector<4>)hit); (void)hitc;
		//fmat::Row<3> hitr2(hitc); (void)hitr2;
		
		if(std::abs(align)>numeric_limits<float>::epsilon())
			hit = fmat::pack(rv_b*(dist/align)+ro_b,1.f);
		else if(align!=0 && dist!=0 && std::signbit(align)!=std::signbit(dist))
			hit = fmat::pack(-rv_b,std::abs(align));
		else
			hit = fmat::pack(rv_b,std::abs(align));
		//hit = fmat::pack(rv_b,align);
		std::cout << "plane: " << p_b << std::endl;
		std::cout << "ray: " << rv_b << " @ " << ro_b << std::endl;
		std::cout << "dist = " << dist << ", align = " << align << std::endl;
		std::cout << "hit: " << hit << std::endl;
		
		testConstSub(p_b);
		
		float v3[9] = { 1, 2, 3, 6, 5, 4, 7, 2, 7 };
		fmat::Matrix<3,3> inv(v3);
		std::cout <<"Original:\n" << inv << std::endl;
		std::cout << "Inverted:\n" << invert(inv) << std::endl;
		std::cout << fmat::invert(fmat::Transform::identity()) << std::endl;
		
		NEWMAT::Matrix inv2(3,3);
		inv2 << v3;
		inv2=inv2.t();
		std::cout << "Check:\n" << inv2.i();
	}
	
	
	{
		std::cout << "\nfmat::TRANSFORMS" << std::endl;
		fmat::Transform tr(fmat::rotationY<3,float>(M_PI/4), fmat::pack(1,2,4));
		fmat::Transform tr2(fmat::rotationZ<3,float>(M_PI/6), fmat::pack(-2,-3,-4));
		fmat::Transform tr3 = tr*tr2;
		cout << tr3 << '\n' << tr*tr2*fmat::pack(5,6,7) << '\n' << tr3*fmat::pack(5,6,7,1) << std::endl;
	}
	
	{
		std::cout << "\nfmat::MATRIX<4,4>" << std::endl;
		fmat::Matrix<4,4> tr;
		(fmat::SubMatrix<3,3>)tr = fmat::rotationY<3,float>(M_PI/4);
		tr.column(3) = fmat::pack(1,2,4,1);
		fmat::Matrix<4,4> tr2(fmat::rotationZ<3,float>(M_PI/6));
		tr2.column(3) = fmat::pack(-2,-3,-4,1);
		fmat::Matrix<4,4> tr3 = tr*tr2;
		cout << tr3 << '\n' << tr*tr2*fmat::pack(5,6,7,1) << '\n' << tr3*fmat::pack(5,6,7,1) << std::endl;
	}
	
	{
		std::cout << "\nNEWMAT" << std::endl;
		NEWMAT::Matrix tr(4,4);
		fmat::Matrix<3,3> r = fmat::rotationY<3,float>(M_PI/4).transpose();
		tr=0;
		tr.SubMatrix(1,3,1,3) << &r(0,0);
		tr(1,4) = 1;
		tr(2,4) = 2;
		tr(3,4) = 4;
		tr(4,4) = 1;
		NEWMAT::Matrix tr2(4,4);
		r = fmat::rotationZ<3,float>(M_PI/6).transpose();
		tr2=0;
		tr2.SubMatrix(1,3,1,3) << &r(0,0);
		tr2(1,4) = -2;
		tr2(2,4) = -3;
		tr2(3,4) = -4;
		tr2(4,4) = 1;
		NEWMAT::Matrix tr3 = tr*tr2;
		NEWMAT::ColumnVector x(4);
		x(1) = 5;
		x(2) = 6;
		x(3) = 7;
		x(4) = 1;
		cout << tr3 << '\n' << tr*tr2*x << std::endl;
	}
	


	{
		fmat::Matrix<4,4> a(fmat::Matrix<4,4>(v1).transpose());
		fmat::Matrix<4,4> b(fmat::Matrix<4,4>::identity());
		fmat::Column<4> x = fmat::pack(1.01f,1.02f,1.03f,1.f);
		fmat::Column<4> d;
		TimeET t;
		for(int i=0; i<TESTS; ++i) {
			a*=b;
			d+=a*x;
		}
		cout << "fmat 4x4: " << t.Age() << endl;
		cout << a << '\n';
		cout << d << '\n' << endl;
	}
	
	{
		fmat::Transform a(fmat::Matrix<4,4>(v1).transpose());
		fmat::Transform b(fmat::Transform::identity());
		fmat::Column<3> x = fmat::pack(1.01f,1.02f,1.03f);
		fmat::Column<3> d;
		TimeET t;
		for(int i=0; i<TESTS; ++i) {
			a*=b;
			d+=a*x;
		}
		cout << "fmat Transform: " << t.Age() << endl;
		cout << a << '\n';
		cout << d << '\n' << endl;
	}
	
	

	// test matrix based Transform vs. quaternion operations
	double angInc = 1*M_PI/180;
	fmat::Quaternion idealQ = fmat::Quaternion::fromAxisAngle(fmat::pack(1,2,3),angInc*fmod(TESTS,2*M_PI/angInc));
	cout << endl;
	cout << "IDEAL:" << idealQ << endl;
	fmat::Transform idealT;
	idealT.translation() = fmat::pack(4,3,2);
	idealT = idealQ * idealT;
	cout << idealT << endl;
	
	{
		fmat::Quaternion qInc = fmat::Quaternion::fromAxisAngle(fmat::pack(1,2,3),1*M_PI/180);
		fmat::Transform tInc = qInc * fmat::Transform::identity();
		fmat::Transform acc;
		acc.translation() = fmat::pack(4,3,2);
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			acc=tInc*acc;
		cout << "fmat Transform: " << t.Age() << endl;
		cout << acc << endl;
		cout << "norms:\n";
		cout << acc.column(0).norm() << ' ' << acc.column(1).norm() << ' ' << acc.column(2).norm() << endl;
		fmat::Matrix<3,3> r2 = acc.rotation().transpose();
		cout << r2.column(0).norm() << ' ' << r2.column(1).norm() << ' ' << r2.column(2).norm() << endl;
		cout << "error:\n";
		cout << acc-idealT << endl;
	}

	{
		fmat::Quaternion qInc = fmat::Quaternion::fromAxisAngle(fmat::pack(1,2,3),1*M_PI/180);
		fmat::Quaternion accQ;
		fmat::Column<3> accP = fmat::pack(4,3,2);
		TimeET t;
		const int RENORM=TESTS>10000 ? 10000 : TESTS;
		for(int i=0; i<TESTS/RENORM; ++i) {
			for(int j=0; j<RENORM; ++j) {
				accQ=qInc*accQ;
				accP=qInc*accP;
			}
			accQ.normalize();
		}
		cout << "fmat Quaternion: " << t.Age() << endl;
		if((accQ.getW()<0) != (idealQ.getW()<0)) {
			accQ=fmat::Quaternion(-accQ.getW(),-accQ.getX(),-accQ.getY(),-accQ.getZ());
		}
		cout << accQ << ' ' << accP << endl;
		cout << "norm: " << accQ.norm() << endl;
		cout << "q err: " << accQ.exportTo<fmat::Column<4> >()-idealQ.exportTo<fmat::Column<4> >() << endl;
		cout << "p err: " << accP-idealT.translation() << endl;
	}

	cout << endl;
	/*{
		NEWMAT::Matrix a(4,4), b(4,4), c(4,4);
		a << v1;
		b << v2;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			c+=a*b;
		cout << "Newmat: " << t.Age() << endl;
		cout << c << endl;
	}*/
	
	{
		fmat::Matrix<4,4> a(fmat::Matrix<4,4>(v1).transpose());
		fmat::Matrix<4,4> b(fmat::Matrix<4,4>(v2).transpose());
		fmat::Matrix<4,4> c;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			c+=a*b;
		cout << "fmat 4x4: " << t.Age() << endl;
		cout << c << '\n' << endl;
	}
	
	{
		fmat::Transform a(fmat::Matrix<4,4>(v1).transpose());
		fmat::Transform b(fmat::Matrix<4,4>(v2).transpose());
		fmat::Transform c;
		TimeET t;
		for(int i=0; i<TESTS; ++i)
			c+=a*b;
		cout << "fmat Transform: " << t.Age() << endl;
		cout << c << '\n' << endl;
	}
	
	{
		float a[16], b[16], c[16];
		for(unsigned int c=0; c<4; ++c) {
			for(unsigned int r=0; r<4; ++r) {
				a[r*4+c] = v1[c*4+r];
			}
		}
		for(unsigned int c=0; c<4; ++c) {
			for(unsigned int r=0; r<4; ++r) {
				b[r*4+c] = v2[c*4+r];
			}
		}
		memset(c,0,sizeof(c));
		TimeET t;
		for(int i=0; i<TESTS; ++i) {
			c[ 0] += a[0]*b[ 0] + a[4]*b[ 1] + a[ 8]*b[ 2] + a[12]*b[ 3];
			c[ 1] += a[1]*b[ 0] + a[5]*b[ 1] + a[ 9]*b[ 2] + a[13]*b[ 3];
			c[ 2] += a[2]*b[ 0] + a[6]*b[ 1] + a[10]*b[ 2] + a[14]*b[ 3];
			c[ 3] += a[3]*b[ 0] + a[7]*b[ 1] + a[11]*b[ 2] + a[15]*b[ 3];
			c[ 4] += a[0]*b[ 4] + a[4]*b[ 5] + a[ 8]*b[ 6] + a[12]*b[ 7];
			c[ 5] += a[1]*b[ 4] + a[5]*b[ 5] + a[ 9]*b[ 6] + a[13]*b[ 7];
			c[ 6] += a[2]*b[ 4] + a[6]*b[ 5] + a[10]*b[ 6] + a[14]*b[ 7];
			c[ 7] += a[3]*b[ 4] + a[7]*b[ 5] + a[11]*b[ 6] + a[15]*b[ 7];
			c[ 8] += a[0]*b[ 8] + a[4]*b[ 9] + a[ 8]*b[10] + a[12]*b[11];
			c[ 9] += a[1]*b[ 8] + a[5]*b[ 9] + a[ 9]*b[10] + a[13]*b[11];
			c[10] += a[2]*b[ 8] + a[6]*b[ 9] + a[10]*b[10] + a[14]*b[11];
			c[11] += a[3]*b[ 8] + a[7]*b[ 9] + a[11]*b[10] + a[15]*b[11];
			c[12] += a[0]*b[12] + a[4]*b[13] + a[ 8]*b[14] + a[12]*b[15];
			c[13] += a[1]*b[12] + a[5]*b[13] + a[ 9]*b[14] + a[13]*b[15];
			c[14] += a[2]*b[12] + a[6]*b[13] + a[10]*b[14] + a[14]*b[15];
			c[15] += a[3]*b[12] + a[7]*b[13] + a[11]*b[14] + a[15]*b[15];
		}
		cout << "Unrolled: " << t.Age() << endl;
		for(int j=0; j<4; ++j) {
			for(int i=0; i<4; ++i) {
				cout << c[i*4+j] << ' ';
			}
			cout << endl;
		}
	}
	
	
	return 0;
}
