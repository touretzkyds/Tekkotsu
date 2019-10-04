#include "TagDetection.h"
#include "MathUtil.h"
#include "Shared/mathutils.h" // for isnan fix
using namespace std;
namespace AprilTags {

TagDetection::TagDetection() 
  : good(false), obsCode(), code(), id(), hammingDistance(), rotation(), p(),
    cxy(), observedPerimeter(), homography(), hxy() {}

TagDetection::TagDetection(int _id)
  : good(false), obsCode(), code(), id(_id), hammingDistance(), rotation(), p(),
    cxy(), observedPerimeter(), homography(), hxy() {}

AngSignPi TagDetection::getXYOrientation() const {
	// Original code said:
  //   Because the order of segments in a quad is arbitrary, so is the
  //   homography's rotation, so we can't determine orientation directly
  //   from the homography.  Instead, use the homography to find two
  //   bottom corners of a properly oriented tag in pixel coordinates,
  //   and then compute orientation from that.
  //   std::pair<float,float> p0 = interpolate(-1,-1);   // lower left corner of tag
  //   std::pair<float,float> p1 = interpolate(1,-1);    // lower right corner of tag
	// But we're not computing orientation until the tag is decoded, so
	// the points ARE in the right order.  Using the points instead of the
	// homography avoids occasional buggy behavior with the homography.  -- DST
	const std::pair<float,float> &p0 = p[0];
	const std::pair<float,float> &p1 = p[1];
  AngSignPi orient = std::atan2(p1.second - p0.second, p1.first - p0.first);
  return ! std::isnan(float(orient)) ? orient : AngSignPi(0);
}

fmat::Matrix<4,4> TagDetection::getRotMatrix() const {
	// This code was taken in April 2014 from the latest AprilTags
	// Java source, but it does not appear to work correctly.  -- DST
	float s = (7*25.4) / 1000;
	float cx = 640/2, cy = 480/2;
	float fx = 500, fy = 500;

	fmat::Matrix<4,4> M;
	M(0,0) = (homography(0,0)-cx*homography(2,0)) / fx;
	M(0,1) = (homography(0,1)-cx*homography(2,1)) / fx;
	M(0,3) = (homography(0,2)-cx*homography(2,2)) / fx;
	M(1,0) = (homography(1,0)-cy*homography(2,0)) / fy;
	M(1,1) = (homography(1,1)-cy*homography(2,1)) / fy;
	M(1,3) = (homography(1,2)-cy*homography(2,2)) / fy;
	M(2,0) = s * homography(2,0);
	M(2,1) = s * homography(2,1);
	M(2,3) = s * homography(2,2);

	float scale0 = sqrt(M(0,0)*M(0,0) + M(1,0)*M(1,0) + M(2,0)*M(2,0));
	float scale1 = sqrt(M(0,1)*M(0,1) + M(1,1)*M(1,1) + M(2,1)*M(2,1));
	float scale = sqrt(scale0 * scale1);

	M = M / scale;
	if ( M(2,3) > 0 )
		M = M * -1.0;

	M(3,0) = M(3,1) = M(3,2) = 0;
	M(3,3) = 1;

	// Recover third rotation vector by cross product.
	fmat::Column<3> a = fmat::pack(M(0,0), M(1,0), M(2,0));
	fmat::Column<3> b = fmat::pack(M(0,1), M(1,1), M(2,1));
	fmat::Column<3> ab = fmat::crossProduct(a,b);
	M(0,2) = ab[0];
	M(1,2) = ab[1];
	M(2,2) = ab[2];

	// Pull out just the rotation component so we can normalize it.
	fmat::Matrix<3,3> R = fmat::SubMatrix<3,3>(M,0,0);

	// Polar decomposition, R = (UV')(VSV')
	NEWMAT::Matrix tm(3,3), tmu(3,3), tmv(3,3);
	NEWMAT::DiagonalMatrix tmd;
	R.exportToNewmat(tm);
	NEWMAT::SVD(tm, tmd, tmu, tmv, true, true);
	NEWMAT::Matrix MR = tmu * tmv.t(); 
	R.importFromNewmat(MR);
	fmat::SubMatrix<3,3>(M,0,0) = R;
	return M;
}

/*
			// ***** OLD CODE HERE

      double tagSize = 0.200;
      double f = 500.0;

      // cout << "homography: " << endl << homography.fmt() << endl;

//     double F[][] = LinAlg.identity(3);
//     F[1][1] = -1;
//     F[2][2] = -1;
        
        fmat::Matrix<3,3> identMatrix(fmat::Matrix<3,3>::identity());
        identMatrix(1, 1) = -1;
        identMatrix(2, 2) = -1;
        // cout << identMatrix.fmt() << endl;
        
//     h = LinAlg.matrixAB(F, h);
        fmat::Matrix<3,3> h = identMatrix * homography;
        // cout << h.fmt() << endl;

//     double M[][] = new double[4][4];
//     M[0][0] =  h[0][0] / f;
//     M[0][1] =  h[0][1] / f;
//     M[0][3] =  h[0][2] / f;
//     M[1][0] =  h[1][0] / f;
//     M[1][1] =  h[1][1] / f;
//     M[1][3] =  h[1][2] / f;
//     M[2][0] =  h[2][0];
//     M[2][1] =  h[2][1];
//     M[2][3] =  h[2][2];

        fmat::Matrix<4,4> m;

        // cout << m.fmt() << endl;
        
        m(0,0) = h(0,0) / f;
        m(0,1) = h(0,1) / f;
        m(0,3) = h(0,2) / f;
        m(1,0) = h(1,0) / f;
        m(1,1) = h(1,1) / f;
        m(1,3) = h(1,2) / f;
        m(2,0) = h(2,0);
        m(2,1) = h(2,1);
        m(2,3) = h(2,2);

        // cout << m.fmt() << endl;

//     // Compute the scale. The columns of M should be made to be
//     // unit vectors. This is over-determined, so we take the
//     // geometric average.
//     double scale0 = Math.sqrt(sq(M[0][0]) + sq(M[1][0]) + sq(M[2][0]));
//     double scale1 = Math.sqrt(sq(M[0][1]) + sq(M[1][1]) + sq(M[2][1]));
//     double scale = Math.sqrt(scale0*scale1);

        double scale0 = sqrt(MathUtil::square(m(0,0)) + MathUtil::square(m(1,0)) + MathUtil::square(m(2,0)));
        double scale1 = sqrt(MathUtil::square(m(0,1)) + MathUtil::square(m(1,1)) + MathUtil::square(m(2,1)));
        double scale = sqrt(scale0 * scale1);

//     M = LinAlg.scale(M, 1.0/scale);

        m = m * (1.0/scale);

        // cout << "Scaled" << m.fmt() << endl;

	// recover sign of scale factor by noting that observations must occur in front of the camera.
//     if (M[2][3] > 0)
//         M = LinAlg.scale(M, -1);

        if (m(2,3) > 0)
            m = m * -1;

        //     // The bottom row should always be [0 0 0 1].  We reset the
//     // first three elements, even though they must be zero, in
//     // order to make sure that they are +0. (We could have -0 due
//     // to the sign flip above. This is theoretically harmless but
//     // annoying in practice.)
//     M[3][0] = 0;
//     M[3][1] = 0;
//     M[3][2] = 0;
//     M[3][3] = 1;

        m(3,0) = 0;
        m(3,1) = 0;
        m(3,2) = 0;
        m(3,3) = 1;

        //     // recover third rotation vector by crossproduct of the other two rotation vectors.
//     double a[] = new double[] { M[0][0], M[1][0], M[2][0] };
//     double b[] = new double[] { M[0][1], M[1][1], M[2][1] };
//     double ab[] = LinAlg.crossProduct(a,b);

//     M[0][2] = ab[0];
//     M[1][2] = ab[1];
//     M[2][2] = ab[2];

        fmat::Column<3> a = fmat::pack(m(0,0), m(1,0), m(2,0));
        fmat::Column<3> b = fmat::pack(m(0,1), m(1,1), m(2,1));
        fmat::Column<3> ab = fmat::crossProduct(a,b);

        // cout << ab.fmt() << endl;

        m(0,2) = ab[0];
        m(1,2) = ab[1];
        m(2,2) = ab[2];
        
        //     // pull out just the rotation component so we can normalize it.
//     double R[][] = new double[3][3];
//     for (int i = 0; i < 3; i++)
//         for (int j = 0; j < 3; j++)
//             R[i][j] = M[i][j];

        fmat::Matrix<3,3> m2;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m2(i,j) = m(i,j);

        // cout << m2 << endl;

        //     SingularValueDecomposition svd = new SingularValueDecomposition(new Matrix(R));
//     // polar decomposition, R = (UV')(VSV')
//     Matrix MR = svd.getU().times(svd.getV().transpose());
//     for (int i = 0; i < 3; i++)
//         for (int j = 0; j < 3; j++)
//             M[i][j] = MR.get(i,j);

        NEWMAT::Matrix tm(3,3);
        m2.exportToNewmat(tm);
        NEWMAT::Matrix tmu(3,3);
        NEWMAT::Matrix tmv(3,3);
        NEWMAT::DiagonalMatrix tmd;
        NEWMAT::SVD(tm, tmd, tmu, tmv, true, true);
        NEWMAT::Matrix tm2 = tmu * tmv.t();
        fmat::Matrix<3,3> m3;
        m3.importFromNewmat(tm2);

        // cout << "imported" << endl << m3.fmt() << endl;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m(i,j) = m3(i,j);
        

	// Scale the results based on the scale in the homography. The
	// homography assumes that tags span from -1 to +1, i.e., that
	// they are two units wide (and tall).
//     for (int i = 0; i < 3; i++)
//         M[i][3] *= tagSize / 2;
        for (int i = 0; i < 3; i++)
             m(i,3) *= tagSize / 2;

        // cout << "final" << endl << m << endl;
        return m;
    }

*/

std::pair<float,float> TagDetection::interpolate(float x, float y) const {
  float z = homography(2,0)*x + homography(2,1)*y + homography(2,2);
  if ( z == 0 )
    return std::pair<float,float>(0,0);  // prevents returning a pair with a -NaN, for which gcc 4.4 flubs isnan
  float newx = (homography(0,0)*x + homography(0,1)*y + homography(0,2))/z + hxy.first;
  float newy = (homography(1,0)*x + homography(1,1)*y + homography(1,2))/z + hxy.second;
  return std::pair<float,float>(newx,newy);
}

bool TagDetection::overlapsTooMuch(const TagDetection &other) const {
  // Compute a sort of "radius" of the two targets. We'll do this by
  // computing the average length of the edges of the quads (in
  // pixels).
  float radius =
    ( MathUtil::distance2D(p[0], p[1]) +
      MathUtil::distance2D(p[1], p[2]) +
      MathUtil::distance2D(p[2], p[3]) +
      MathUtil::distance2D(p[3], p[0]) +
      MathUtil::distance2D(other.p[0], other.p[1]) +
      MathUtil::distance2D(other.p[1], other.p[2]) +
      MathUtil::distance2D(other.p[2], other.p[3]) +
      MathUtil::distance2D(other.p[3], other.p[0]) ) / 16.0f;

  // distance (in pixels) between two tag centers
  float dist = MathUtil::distance2D(cxy, other.cxy);

  // reject pairs where the distance between centroids is smaller than
  // the "radius" of one of the tags.
  if ( dist < radius )
    std::cout << "AprilTags: rejecting AprilTag with dist=" << dist << "  radius=" << radius << std::endl;
  return ( dist < radius );
}

} // namespace
