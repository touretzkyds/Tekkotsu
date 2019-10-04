#ifndef _HOMOGRAPHY_H_
#define _HOMOGRAPHY_H_

#include "Shared/newmat/newmat.h"
#include "Shared/newmat/newmatap.h"
#include "Shared/fmat.h"

class Homography {
	//! The matrix that holds the correspondences
	fmat::Matrix<9,9> A;
	//! The matrix that holds the computed homography
	fmat::Matrix<3,3> h;
	//! The matrix that holds the inverse of the computed homography
	fmat::Matrix<3,3> hInv;
	
	//! Whether the homography is out of date with how many correspondences have been added
	bool isDirty;
	
public:
	
	Homography(): A(), h(), hInv(), isDirty(true) {
		for(unsigned i = 0; i < 9; i++)
			for(unsigned j = 0; j < 9; j++)
				A(i,j) = 0;
		
		for(unsigned i = 0; i < 3; i++)
			for(unsigned j = 0; j < 3; j++) {
				h(i,j) = 0;
				hInv(i,j) = 0;
			}
	}
	
	//! Projects (x,y) to the corrected values (correctedX, correctedY) based off the homography matrix
	void project(float x, float y, float& correctedX, float& correctedY) {
		fmat::Matrix<3,3>& H = getMatrix();
		
		correctedX = H(0,0)*x + H(0,1)*y + H(0,2);
		correctedY = H(1,0)*x + H(1,1)*y + H(1,2);
		float z =    H(2,0)*x + H(2,1)*y + H(2,2);
		if(z == 0)
			z = 1;
		
		correctedX /= z;
		correctedY /= z;
	}
	
	//! Inverts (correctedX, correctedY) to the original values (x,y) from the homography matrix inverse
	void invertProjection(float correctedX, float correctedY, float& x, float& y) {
		float z;
		fmat::Matrix<3,3>& invH = getMatrixInverse();
		
		x = invH(0,0)*correctedX + invH(0,1)*correctedY + invH(0,2);
		y = invH(1,0)*correctedX + invH(1,1)*correctedY + invH(1,2);
		z = invH(2,0)*correctedX + invH(2,1)*correctedY + invH(2,2);
		
		x /= z;
		y /= z;
	}
	
	//! Returns the homography so far, recomputes H if out of date
	fmat::Matrix<3,3>& getMatrix()
    {
		if(!isDirty)
			return h;
		
        // make symmetric
        for (int i = 0; i < 9; i++)
            for (int j = i+1; j < 9; j++)
                A(j,i) = A(i,j);
		
		NEWMAT::Matrix nmA(9,9), U, V;
		NEWMAT::DiagonalMatrix D;
		
		A.exportToNewmat(nmA);
		
        // compute using singular value decomposition
		NEWMAT::SVD(nmA,D,U,V);
		
	    for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                h(i,j) = V(i*3+j+1, V.ncols());
			}
		}
		
		hInv = invert(h);
		
		isDirty = false;
		
		return h;
    }
	
	fmat::Matrix<3,3>& getMatrixInverse() {
		//Compute new data if dirty
		getMatrix();
		return hInv;
	}
	
	//! Adds a correspondence to the list. actualx and actualy are the corrected values, imagex and imagey are the perceived values
	void addCorrespondence(double actualx, double actualy, double imagex, double imagey)
    {
		isDirty = true;
//		cout<<"("<<imagex<<", "<<imagey<<") -> ("<<actualx<<", "<<actualy<<")"<<endl;
        double a03 = -actualx;
        double a04 = -actualy;
        double a05 = -1;
        double a06 = actualx*imagey;
        double a07 = actualy*imagey;
        double a08 = imagey;
		
        A(3, 3) += a03*a03;
        A(3, 4) += a03*a04;
        A(3, 5) += a03*a05;
        A(3, 6) += a03*a06;
        A(3, 7) += a03*a07;
        A(3, 8) += a03*a08;
        A(4, 4) += a04*a04;
        A(4, 5) += a04*a05;
        A(4, 6) += a04*a06;
        A(4, 7) += a04*a07;
        A(4, 8) += a04*a08;
        A(5, 5) += a05*a05;
        A(5, 6) += a05*a06;
        A(5, 7) += a05*a07;
        A(5, 8) += a05*a08;
        A(6, 6) += a06*a06;
        A(6, 7) += a06*a07;
        A(6, 8) += a06*a08;
        A(7, 7) += a07*a07;
        A(7, 8) += a07*a08;
        A(8, 8) += a08*a08;
		
        double a10 = actualx;
        double a11 = actualy;
        double a12 = 1;
        double a16 = -actualx*imagex;
        double a17 = -actualy*imagex;
        double a18 = -imagex;
		
        A(0, 0) += a10*a10;
        A(0, 1) += a10*a11;
        A(0, 2) += a10*a12;
        A(0, 6) += a10*a16;
        A(0, 7) += a10*a17;
        A(0, 8) += a10*a18;
        A(1, 1) += a11*a11;
        A(1, 2) += a11*a12;
        A(1, 6) += a11*a16;
        A(1, 7) += a11*a17;
        A(1, 8) += a11*a18;
        A(2, 2) += a12*a12;
        A(2, 6) += a12*a16;
		A(2, 7) += a12*a17;
        A(2, 8) += a12*a18;
        A(6, 6) += a16*a16;
        A(6, 7) += a16*a17;
        A(6, 8) += a16*a18;
        A(7, 7) += a17*a17;
        A(7, 8) += a17*a18;
        A(8, 8) += a18*a18;
		
        double a20 = -actualx*imagey;
        double a21 = -actualy*imagey;
        double a22 = -imagey;
        double a23 = actualx*imagex;
        double a24 = actualy*imagex;
        double a25 = imagex;
		
        A(0, 0) += a20*a20;
        A(0, 1) += a20*a21;
        A(0, 2) += a20*a22;
        A(0, 3) += a20*a23;
        A(0, 4) += a20*a24;
        A(0, 5) += a20*a25;
        A(1, 1) += a21*a21;
        A(1, 2) += a21*a22;
        A(1, 3) += a21*a23;
        A(1, 4) += a21*a24;
        A(1, 5) += a21*a25;
        A(2, 2) += a22*a22;
        A(2, 3) += a22*a23;
        A(2, 4) += a22*a24;
        A(2, 5) += a22*a25;
        A(3, 3) += a23*a23;
        A(3, 4) += a23*a24;
        A(3, 5) += a23*a25;
        A(4, 4) += a24*a24;
        A(4, 5) += a24*a25;
        A(5, 5) += a25*a25;
    }
	
};






#endif
