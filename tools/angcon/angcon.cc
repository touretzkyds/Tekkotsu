#include "Shared/fmat.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std;

fmat::fmatReal num(const string& name, int i, int argc, const char* argv[]) {
	if(i>=argc)
		throw runtime_error(string("Missing argument for ")+name);
	stringstream ss(argv[i]);
	fmat::fmatReal ans;
	if(!(ss >> ans))
		throw runtime_error(string("Could not parse ")+argv[i]);
	return ans;
}

int main(int argc, const char* argv[]) {
	if(argc<=1 || strcmp(argv[1],"-h")==0 || strcmp(argv[1],"--help")==0) {
		std::string indent(std::strlen(argv[0])+8,' ');
		std::cerr << "Usage: " << argv[0] << " [(x|y|z) degrees]\n"
		<< indent << "[(xr|yr|zr) radians]\n"
		<< indent << "[-p|--apply x y [z]]\n"
		<< indent << "[-o|--out format]\n" << std::endl;
		std::cerr << "  format can be one of 'array', 'quaternion' (or 'quat'), or 'matrix' (or 'mat')" << std::endl;
		return 2;
	}
	
	fmat::Quaternion q;
	fmat::Column<3> p;
	enum { OUT_MATRIX, OUT_QUAT, OUT_ARRAY, OUT_POINT, OUT_POINTARRAY } output = OUT_ARRAY;
	
	try {
		for(int i=1; i<argc; ++i) {
			const string a(argv[i]);
			if(a=="x") {
				q *= fmat::Quaternion::aboutX(num(a,++i,argc,argv)*M_PI/180);
			} else if(a=="y") {
				q *= fmat::Quaternion::aboutY(num(a,++i,argc,argv)*M_PI/180);
			} else if(a=="z") {
				q *= fmat::Quaternion::aboutZ(num(a,++i,argc,argv)*M_PI/180);
			} else if(a=="xr") {
				q *= fmat::Quaternion::aboutX(num(a,++i,argc,argv));
			} else if(a=="yr") {
				q *= fmat::Quaternion::aboutY(num(a,++i,argc,argv));
			} else if(a=="zr") {
				q *= fmat::Quaternion::aboutZ(num(a,++i,argc,argv));
			} else if(a=="-p" || a=="--apply") {
				p[0] = num(a,++i,argc,argv);
				p[1] = num(a,++i,argc,argv);
				try {
					p[2] = num(a,i+1,argc,argv);
					++i;
				} catch(...) {}
				output = (output==OUT_ARRAY) ? OUT_POINTARRAY : OUT_POINT;
			} else if(a=="-o" || a=="--out") {
				if(++i>=argc)
					throw runtime_error(string("Missing argument for ")+a);
				const string o(argv[i]);
				if(o=="array") {
					output = (output==OUT_POINT) ? OUT_POINTARRAY : OUT_ARRAY;
				} else if(o=="quaternion" || o=="quat") {
					output = OUT_QUAT;
				} else if(o=="matrix" || o=="mat") {
					if(output==OUT_POINTARRAY)
						output = OUT_POINT;
					else if(output!=OUT_POINT)
						output = OUT_MATRIX;
				}
			} else {
				throw runtime_error("Unknown argument "+a);
			}
		}
	} catch(const std::exception& ex) {
		std::cerr << "ERROR: " << ex.what() << std::endl;
		return EXIT_FAILURE;
	}
	
	p = q*p;
	
	switch(output) {
		
		case OUT_ARRAY: {
			q.normalize();
			cout << "<array> <real>" << q.getX() << "</real> <real>" << q.getY() << "</real> <real>" << q.getZ() << "</real> </array>" << endl;
		} break;
		
		case OUT_QUAT: {
			cout << "[ " << q.getW()<<' '<<q.getX()<<' '<<q.getY()<<' '<<q.getZ()<< " ]" << endl;
		} break;
		
		case OUT_MATRIX: {
			cout << q * fmat::Matrix<3,3>::identity() << endl;
		} break;
		
		case OUT_POINT: {
			cout << p.transpose() << endl;
		} break;
		
		case OUT_POINTARRAY: {
			cout << "<array> <real>" << p[0] << "</real> <real>" << p[1] << "</real> <real>" << p[2] << "</real> </array>" << endl;
		} break;
	}
	
	return EXIT_SUCCESS;
}
