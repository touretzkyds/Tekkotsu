#include "Shared/fmat.h"
#include "Shared/TimeET.h"
#include <iostream>
#include <cstdlib>

using namespace fmat;
using namespace std;

int main(int argc, const char* argv[]) {
	fmat::defaultNumberFormat="%.4f";
	
	Column<3> a1 = fmat::pack(.9,.4,0.05);
	Column<3> a2 = fmat::pack(.3,-.8,.2);
	Quaternion q1 = Quaternion::fromAxis(a1);
	Quaternion q2 = Quaternion::fromAxis(a2);
	
	Column<3> a3 = crossProduct(a1,a2);
	Quaternion q3 = crossProduct(q1,q2);
	cout << q1 << '\n' << q2 << '\n';
	//cout << "Axis cross " << a3 << " norm " << a3/a3.norm() << '\n';
	cout << "Diff " << q3 << " axis " << q3.axis() << '\n';
	cout << "q2 to q1 " << q3.inverse()*q2 << '\n';
	cout << "q1 to q2 " << q3*q1 << '\n';
	
	//Quaternion qe = Quaternion::aboutZ(a2[2]*M_PI) * Quaternion::aboutY(a2[1]*M_PI) * Quaternion::aboutX(a2[0]*M_PI);
	Quaternion qe = Quaternion::aboutX(a2[0]*M_PI) * Quaternion::aboutY(a2[1]*M_PI) * Quaternion::aboutZ(a2[2]*M_PI);
	cout << "qe " << qe << " qeV " << qe.axis() << endl;
	cout << "Times identity:\n";
	cout << qe*Matrix<3,3>::identity() << endl;
	cout << "To matrix:\n";
	cout << qe.toMatrix() << endl;
	cout << "Back to quaternion:\n";
	cout << Quaternion::fromMatrix(qe.toMatrix()) << endl;
	
	
	/*Quaternion offset = q2;
	Column<3> transAxis = crossProduct(offset.axis(),fmat::pack(0,0,1));
	transAxis/=transAxis.norm();
	//double ta = M_PI;
	for(fmatReal ta=-2*M_PI; ta<2*M_PI; ta+=.5/180*M_PI) {
		Column<3> jointAxis = Quaternion::fromAxisAngle(transAxis,ta)*offset.axis();
		double minErr=numeric_limits<double>::infinity();
		double minErrAng=0;
		for(fmatReal ja=-2*M_PI; ja<2*M_PI; ja+=.5/180*M_PI) {
			Quaternion jointQ = Quaternion::fromAxisAngle(jointAxis,ja);
			Quaternion errQ = offset.inverse() * jointQ;
			double err = errQ.angle();
			if(err<0)
				err+=2*M_PI;
			if(err<minErr) {
				minErr=err;
				minErrAng=ja;
			}
			std::cout << ja << ' ' << err << std::endl;
		}
		double align = dotProduct(offset.axis(),jointAxis);
		if(align>1) align=1;
		if(align<-1) align=-1;
		double pred = offset.angle()*asin(align);
		//std::cout << ta << ' ' << minErrAng << ' ' << pred << std::endl;
	}*/
	
	/*cout << Quaternion::aboutZ(.2) << endl;
	cout << Quaternion::fromAxisAngle(pack(0,0,1),.2) << endl;
	Column<3> p = Quaternion::fromAxisAngle(pack(0,0,1),.2)*pack(1,0,0);
	cout << p << endl;
	cout << atan2(p[1],p[0]) << endl;*/
	
	cout << "45°X " << Quaternion::aboutX(M_PI_2/2) << ' ' << Quaternion::fromMatrix(rotationX(M_PI_2/2)) << endl;
	cout << "45°Y " << Quaternion::aboutY(M_PI_2/2) << ' ' << Quaternion::fromMatrix(rotationY(M_PI_2/2)) << endl;
	cout << "45°Z " << Quaternion::aboutZ(M_PI_2/2) << ' ' << Quaternion::fromMatrix(rotationZ(M_PI_2/2)) << endl;
	cout << "-45°X " << Quaternion::aboutX(-M_PI_2/2) << ' ' << Quaternion::fromMatrix(rotationX(-M_PI_2/2)) << endl;
	cout << "-45°Y " << Quaternion::aboutY(-M_PI_2/2) << ' ' << Quaternion::fromMatrix(rotationY(-M_PI_2/2)) << endl;
	cout << "-45°Z " << Quaternion::aboutZ(-M_PI_2/2) << ' ' << Quaternion::fromMatrix(rotationZ(-M_PI_2/2)) << endl;
	cout << "90°X " << Quaternion::aboutX(M_PI_2) << ' ' << Quaternion::fromMatrix(rotationX(M_PI_2)) << endl;
	cout << "90°Y " << Quaternion::aboutY(M_PI_2) << ' ' << Quaternion::fromMatrix(rotationY(M_PI_2)) << endl;
	cout << "90°Z " << Quaternion::aboutZ(M_PI_2) << ' ' << Quaternion::fromMatrix(rotationZ(M_PI_2)) << endl;
	cout << "90°XZ " << Quaternion::aboutZ(M_PI_2)*Quaternion::aboutX(M_PI_2) << endl;
	cout << Quaternion::aboutZ(M_PI_2)*Quaternion::aboutX(M_PI_2).toMatrix() << endl;
	
	Matrix<3,3> m = fmat::rotationZ(M_PI_2)*fmat::rotationX(M_PI_2);
	cout << m << '\n' << Quaternion::fromMatrix(m) << endl;
	
	//srandom(1231);
	TimeET startTime;
	cout << "Running random tests" << endl;
	for(size_t i=0; i<200000; ++i) {
		fmatReal y=random()/float((1<<31)-1) * M_PI * 2 - M_PI;
		fmatReal p=random()/float((1<<31)-1) * M_PI - M_PI/2;
		fmatReal r=random()/float((1<<31)-1) * M_PI * 2 - M_PI;
		Column<3> yprV=fmat::pack(y,p,r);
		Quaternion q = Quaternion::aboutZ(y)*Quaternion::aboutY(p)*Quaternion::aboutX(r);
		if(std::abs(q.normalize()-1)>0.001)
			cout << "Initial q was non-normalized!" << endl;
		fmat::Matrix<3,3> m = rotationZ(y)*rotationY(p)*rotationX(r);
		if(abs(Column<9>((m-q.toMatrix())(0,0))).max()>0.001) {
			cout << "\nBad q.toMatrix() " << yprV << ":\n";
			cout << q << '\n' << m << '\n' << q.toMatrix() << '\n';
		}
		Quaternion q2 = Quaternion::fromMatrix(m);
		if(std::abs(q2.normalize()-1)>0.001)
			cout << "fromMatrix was non-normalized!" << endl;
		Column<3> t1,t2;
		q.exportTo(t1);
		q2.exportTo(t2);
		if(abs(t1-t2).max()>0.001) {
			cout << "\nBad q.fromMatrix() " << yprV << ":\n";
			cout << q << ' ' << q.axis() << '\n' << m << '\n' << q2 << ' ' << q2.axis() << '\n';
		}
		t1 = ypr(q);
		t2 = ypr(m);
		fmat::Column<3> yprErr = abs(t1-t2);
		for(size_t j=0;j<3;j++) {
			if(yprErr[j]>M_PI)
				yprErr[j]=std::abs(yprErr[j]-M_PI*2);
		}
		if(yprErr.max()>0.02 || t2[2]!=0 && (yprErr.max()>0.001 || abs(t2-yprV).max()>0.001) ) {
			cout << "\nBad ypr() " << yprV << ":\n";
			cout << q << " -> " << t1 << '\n';
			cout << m << "\n -> " << t2 << '\n';
			/*fmat::SubVector<9> x(&m(0,0));
			for(int i=0; i<9; ++i) {
				std::cout << std::asin(x[i]) <<'\n';
				std::cout << std::acos(x[i]) <<'\n';
				for(int j=0; j<9; ++j) {
					std::cout << x[i] << ' ' << x[j] << ' ' << std::atan2(x[i],x[j]) << std::endl;
				}
			}*/
		}
	}
	cout << "@VAR Took " << startTime.Age() << endl;
	
	return EXIT_SUCCESS;
}
