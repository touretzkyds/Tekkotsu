#if defined(TGT_IS_MANTIS) && defined(TGT_HAS_HEAD)

#include "IKMantis.h"
#include "IKGradientSolver.h"
#include "Kinematics.h"

//const float IKMantis::EPSILON = 1e-3;
const std::string IKMantis::autoRegisterIKMantis = IKSolver::getRegistry().registerType<IKMantis>("IKMantis");

IKMantis::IKMantis() { //const KinematicJoint* j = kine->getKinematicJoint(CameraFrameOffset);
    // qOffset = kine->getKinematicJoint(HeadPanOffset)->qOffset;
    // cameraOffset = kine->getKinematicJoint(CameraFrameOffset)->qOffset;
    // baseToHead = kine->getKinematicJoint(HeadPanOffset)->getTo().inverse();
}

bool IKMantis::solve(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float posPri, const Orientation& oriTgt, float oriPri) const {
	
	bool solved;
	if(j.outputOffset == CameraFrameOffset) 
		solved = solveHead(j,pTgt);
	else if(j.outputOffset >= FootFrameOffset+LMdLegOrder && j.outputOffset <= FootFrameOffset+RBkLegOrder)
		solved = solvePosLeg(j,pTgt);
	else
		solved = solveFrontLeg(j,pTgt); 
	return solved;
}

bool IKMantis::solveHead(KinematicJoint& j, const Position& pTgt) const {       
	// translate target point from BaseFrame to the frame of the Pan Servo
	const Point& pTgtPoint = dynamic_cast<const Point&>(pTgt);
	KinematicJoint *roll = j.getParent();
	KinematicJoint *tilt = roll->getParent();
	KinematicJoint *pan = tilt->getParent();
	roll->tryQ(0.f);
	tilt->tryQ(0.f);
	//std::cout << "qPan is :" << qPan*180/M_PI << "  targ1 = " << targ1 << std::endl; 
	//std::cout << kine->linkToBase(HeadOffset+PanOffset) << std::endl;
	
	//correcting the camera angle
	//fmat::Column<3> start = pan->getWorldPosition();
	float radius =  kine->linkToLink(CameraFrameOffset, HeadOffset+PanOffset)(1,3);
	std::cout << " radius = " << radius << std::endl;
	fmat::Column<3> targ1 = pan->getFullInvT() * pTgtPoint;
	targ1[2] = 0; // We're working in the x-y plane; want norm to ignore z distance
	float d = targ1.norm();
	std::cout << " distance d = " << d << std::endl;
	float R = radius/d;
	float X = targ1[0] / d;
	float Y = targ1[1] / d;
	float rsq = sqrt(1 - R*R);
	float a = R*X - Y*rsq;
	float b = R*Y + X*rsq;
	float theta = atan2(-a,b);
	float qPan = pan->getQ() + theta; 
	pan->tryQ(qPan);
	//std::cout << "Camera to pan: " << std::endl << kine->linkToLink(CameraFrameOffset, HeadOffset+PanOffset) << std::endl;

	// tilt
	radius =  kine->linkToLink(CameraFrameOffset, HeadOffset+TiltOffset)(1,3);
	fmat::Column<3> targ2 = tilt->getFullInvT() * pTgtPoint;
	targ2[2] = 0; // We're working in the x-y plane; want norm to ignore z distance
	d = targ2.norm();
	//std::cout << " distance d = " << d << std::endl;
	R = radius/d;
	X = targ2[0] / d;
	Y = targ2[1] / d;
	rsq = sqrt(1 - R*R);
	a = R*X - Y*rsq;
	b = R*Y + X*rsq;
	theta = atan2(-a,b);
	float qTilt = tilt->getQ() + theta; 
	tilt->tryQ(qTilt);

	//fmat::Column<3> targ2 = tilt->getFullInvT() * pTgtPoint;
	//float qTilt = std::atan2(targ2[1],targ2[0]) + tilt->getQ();
	//std::cout << "qTilt is :" << qTilt*180/M_PI << std::endl;
	//std::cout<< kine->linkToBase(HeadOffset+TiltOffset) << "  targ2 = " << targ2 << std::endl;
	std::cout << "Pan to tilt:" << kine->linkToLink(HeadOffset+PanOffset, HeadOffset+TiltOffset) << std::endl;
	return true;
}

bool IKMantis::solvePosLeg(KinematicJoint& j, const Position& pTgt) const {	
	const Point& pTgtPoint = dynamic_cast<const Point&>(pTgt);
	KinematicJoint *knee = j.getParent()->getParent();
	KinematicJoint *elevator = knee->getParent();
	KinematicJoint *rotor = elevator->getParent();
	KinematicJoint *sweep = rotor->getParent();
	//std::cout << "sweep at " << sweep->getWorldPosition() << std::endl;
	//std::cout << "rotor at " << rotor->getWorldPosition() << std::endl;
	//std::cout << "elev  at " << elevator->getWorldPosition() << std::endl;
	//std::cout << "knee  at " << knee->getWorldPosition() << std::endl;
	rotor->tryQ(0.f);
    
	bool solved; 
	// sweep
	fmat::Column<3> targ1 = sweep->getFullInvT() * pTgtPoint;
	//std::cout << " sweepInv : " << sweep->getFullInvT() << std::endl; // transformation matrix from base to link(sweep) frame 
	float qsweep = atan2(targ1[1],targ1[0]) + sweep->getQ(); 
	sweep->tryQ(qsweep);
	//std::cout << "qsweep is :" << qsweep*180/M_PI << std::endl /*<< "  targ1 = " << targ1 << std::endl*/;
	
	// 2-link IK
	fmat::Column<3> targ2 = elevator->getFullInvT() * pTgtPoint; // target point in the frame of elevator
	//unsigned int legLink = LegOffset + (j.outputOffset - FootFrameOffset)*JointsPerPosLeg + FrontLegExtra;
	
	fmat::Column<3> elvtrPos = elevator->getWorldPosition();
	fmat::Column<3> len1 = knee->getFullInvT() * elvtrPos;
	fmat::Column<3> footframe = j.getWorldPosition();
	fmat::Column<3> len2 = knee->getFullInvT() * footframe;

	//fmat::Column<3> len1 = kine->linkToLink(legLink+PosElevatorOffset, legLink+PosKneeOffset).translation();
	//fmat::Column<3> len2 = kine->linkToLink(j.outputOffset, legLink+PosKneeOffset).translation();
	float L1 = hypotf(len1[0],len1[1]);
	float L2 = hypotf(len2[0],len2[1]);
	float qknee;
	float qelevtr;
	//std::cout << "Translation vector for length1 = " << len1 << std::endl << "Length1 normalization vector = " << L1 << std::endl;
	//std::cout << "Translation vector for length2 = " << len2 << std::endl << "Length2 normalization vector = " << L2 << std::endl;
	//std::cout << " targ2 = " << targ2 << std::endl;
	float t2angle = atan2(targ2[1],targ2[0]);
	float k_cos = (targ2[0]*targ2[0] + targ2[1]*targ2[1] - L1*L1 - L2*L2) / (2*L1*L2); // using law of cosines
	float k_sin = std::sqrt(1 - k_cos*k_cos) * (t2angle > 0.0 ? 1 : -1);
	float k1 = L1 + k_cos*L2;
	float k2 = k_sin*L2;

	// The tibia is bent, so we must calculate a correction angle alpha.
	float alpha = atan2(len2[1],len2[0]);
	//std::cout << " alpha = " << alpha << std::endl;
	
	if (L1 + L2 <= hypotf(targ2[0],targ2[1])) {
	// point too far away
		qknee = -alpha;
		qelevtr = t2angle + elevator->getQ();
		solved = false;
		//std::cout << " Point too far away " << std::endl;
	} else {
		qknee = atan2(k_sin,k_cos) - alpha /*+ knee->getQ()*/; 
		qelevtr = t2angle - atan2(k2,k1) + elevator->getQ(); 
		//std::cout << "qelevtr atan2t=" << atan2(targ2[1],targ2[0]) << "  atan2k=" << atan2(k2,k1) << "  getQ=" << elevator->getQ() << std::endl;
		solved = true;
	}
	//std::cout << "K1 = " << k1 << std::endl << "K2 = " << k2 << std::endl;	
	//std::cout << "cos = " << k_cos << std::endl << "sin = " << k_sin << std::endl;	
	//std::cout << "x = " << targ2[0] << std::endl << "y = " << targ2[1] << std::endl;
	//std::cout << "Elevator angle = " << qelevtr*180/M_PI << std::endl << "Knee angle = " << qknee*180/M_PI << std::endl << std::endl;
	elevator->tryQ(qelevtr);
	knee->tryQ(qknee);

	return solved;	
}

bool IKMantis::solveFrontLeg(KinematicJoint& j, const Position& pTgt) const {
	const Point& pTgtPoint = dynamic_cast<const Point&>(pTgt);
	bool solved;
	KinematicJoint *wrist = j.getParent()->getParent();
	KinematicJoint *twist2 = wrist->getParent();
	KinematicJoint *elbow = twist2->getParent();
	KinematicJoint *twist1 = elbow->getParent();
	KinematicJoint *elevator = twist1->getParent();
	KinematicJoint *sweep = elevator->getParent();
	//std::cout << "Initial joint values: " << sweep->getQ() << " " << elevator->getQ() << " " << elbow->getQ() << " " << twist2->getQ() << " " << wrist->getQ() << std::endl;
	twist1->tryQ(0);
   
	//unsigned int legLink = LegOffset + (j.outputOffset - FootFrameOffset)*JointsPerFrLeg;
	//fmat::Column<3> footToWrist = kine->linkToLink(j.outputOffset, legLink+FrWristOffset).translation();
    
    	fmat::Column<3> footframe = j.getWorldPosition();
    	fmat::Column<3> footToWrist = wrist->getFullInvT() * footframe;
    	float len_tibia = hypotf(footToWrist[0],footToWrist[1]);
    	//std::cout << "FootToWrist translation vector = " << footToWrist << std::endl << "Length of the tibia = " << len_tibia << std::endl;
    	const Point pTgtWrist = pTgtPoint;
    	pTgtWrist.z = pTgtPoint.z + len_tibia;
    	//std::cout << "Target position of the wrist servo = " << pTgtWrist << std::endl;
    
    	// sweep
    	fmat::Column<3> targ1 = sweep->getFullInvT() * pTgtWrist; // target wrist position in the frame of sweep servo
    	//std::cout << "sweepInv : " << sweep->getFullInvT() << std::endl; // transformation matrix from base to link(sweep) frame
    	//std::cout << "baseToLink " << kine->baseToLink(legLink+FrSweepOffset) << std::endl;
    	float qsweep = atan2(targ1[1],targ1[0]) + sweep->getQ();
    	sweep->tryQ(qsweep);
	    //std::cout << "qsweep is :" << qsweep*180/M_PI << std::endl << "targ1 = " << targ1 << std::endl;

    	// 2-link IK
    	fmat::Column<3> targ2 = elevator->getFullInvT() * pTgtWrist; // target wrist position in the frame of elevator

    	fmat::Column<3> elvtrPos = elevator->getWorldPosition();
    	fmat::Column<3> len1 = elbow->getFullInvT() * elvtrPos;
    	fmat::Column<3> wristPos = wrist->getWorldPosition();
    	fmat::Column<3> len2 = elbow->getFullInvT() * wristPos;

    	//fmat::Column<3> len1 = kine->linkToLink(legLink+FrElevatorOffset, legLink+FrElbowOffset).translation();
    	//fmat::Column<3> len2 = kine->linkToLink(legLink+FrWristOffset, legLink+FrElbowOffset).translation();
    	float L1 = hypotf(len1[0],len1[1]);
    	float L2 = hypotf(len2[0],len2[1]);
    	float qelevtr;
    	float qelbow;
    	//std::cout << "Translation vector for length1 = " << len1 << std::endl << "Length1 normalization vector = " << L1 << std::endl;
    	//std::cout << "Translation vector for length2 = " << len2 << std::endl << "Length2 normalization vector = " << L2 << std::endl;
    	//std::cout << "targ2 = " << targ2 << std::endl;
    	float k_cos = (targ2[0]*targ2[0] + targ2[1]*targ2[1] - L1*L1 - L2*L2) / (2*L1*L2); // using law of cosines
    	float k_sin = std::sqrt(1 - k_cos*k_cos);
        if (j.outputOffset == FootFrameOffset+LFrLegOrder) k_sin = -k_sin;  //! hack..fix it later
    	float k1 = L1 + k_cos*L2;
    	float k2 = k_sin*L2;
    
    	if (L1 + L2 <= hypotf(targ2[0],targ2[1])) {
    		// point too far away
        	qelbow = 0;
        	qelevtr = atan2(targ2[1],targ2[0]) + elevator->getQ();
        	std::cout << "Point too far away " << std::endl;
        	solved = false; 
    	} else {
        	qelbow = atan2(k_sin,k_cos) /*+ elbow->getQ()*/;
        	qelevtr = atan2(targ2[1],targ2[0]) - atan2(k2,k1) + elevator->getQ();
        	solved = true;
    	}
    	//std::cout << "K1 = " << k1 << std::endl << "K2 = " << k2 << std::endl;
    	//std::cout << "cos = " << k_cos << std::endl << "sin = " << k_sin << std::endl;
    	//std::cout << "x = " << targ2[0] << std::endl << "y = " << targ2[1] << std::endl;
    	//std::cout << "Elevator angle = " << qelevtr*180/M_PI << std::endl << "Elbow angle = " << qelbow*180/M_PI << std::endl;
    	elevator->tryQ(qelevtr);
    	elbow->tryQ(qelbow);
  
   	// Put leg vertically on the ground

/*  	//fmat::Column<3> vec = fmat::pack(0, 1, 0); 
    	//fmat::Column<3>  tvec = twist2->getFullInvT() * vec;
    	//std::cout << "vec in twist2 frame = " << tvec << std::endl;
    	fmat::Matrix<3,3> rot =  kine->linkToBase(legLink+FrTwist2Offset).rotation();
    	std::cout << "Orientation of twist2 in base coordinates = " << rot << std::endl;
    	fmat::Quaternion q = twist2->getWorldQuaternion();
    	std::cout << "q = " << q << std::endl;
    	float qtwist2 = -atan2(2*(q.getW()*q.getZ() + q.getX()*q.getY()), 1 - 2*(q.getY()*q.getY() + q.getZ()*q.getZ())) - M_PI/2;
    	std::cout << " qtwist2 = " << qtwist2 << std::endl;
    	twist2->tryQ(0);
*/  
    	fmat::Column<3> targ3 = twist2->getFullInvT() * pTgtPoint; //target point in the frame of twrist2 servo
        float qtwist2;
    	//std::cout << "target point in twist2 frame = " << targ3 << std::endl;
        if (j.outputOffset == FootFrameOffset+LFrLegOrder) qtwist2 = -atan2(-targ3[0], -targ3[1]) + twist2->getQ();  //! hack..fix it later
    	else qtwist2 = -atan2(targ3[0], targ3[1]) + twist2->getQ();
        //std::cout << "atan2 of targ3 = " << atan2(targ3[0],targ3[1]) << std::endl;
    	//std::cout << "Twist2 angle (in degrees) = " << qtwist2*180/M_PI << std::endl; 
    	twist2->tryQ(qtwist2);

     
    	// The tibia is bent, so we must calculate a correction angle beta.
    	float beta = atan2(footToWrist[1],footToWrist[0]); 
    	//std::cout << "beta  = " << beta*180/M_PI << std::endl;
    	fmat::Column<3> targ4 = wrist->getFullInvT() * pTgtPoint; //target point in the frame of wrist servo
    	//std::cout << "target point in wrist frame = " << targ4 << std::endl;
    	float qwrist = atan2(targ4[1],targ4[0]) - beta + wrist->getQ();
    	//std::cout << "atan2 of targ4 = " << atan2(targ4[1],targ4[0]) << std::endl;
    	wrist->tryQ(qwrist);
    	//std::cout << "Wrist angle (in degrees) = " << qwrist*180/M_PI << std::endl; 

    	//fmat::Column<3> legT = j.getWorldPosition();
    	//std::cout << " The position of toe is :  " << std::endl << legT << std::endl;

    	return solved;
}

IKSolver::StepResult_t IKMantis::step(const Point& pEff, const Rotation& oriEff, KinematicJoint& j, const Position& pTgt, float pDist, float posPri, const Orientation& oriTgt, float oriDist, float oriPri) const {
    // find the closest target point
	fmat::Transform Tj = j.getFullT();
	fmat::Column<3> pEffBase(Tj*pEff);
	fmat::Quaternion qj = j.getWorldQuaternion(); //cout << "WorldQ: " << qj;
	Rotation oEffBase = qj*oriEff;
	Point de;
	pTgt.computeErrorGradient(pEffBase,oEffBase,de);

	StepResult_t res=SUCCESS;
	if(de.norm()>pDist) {
		de*=pDist/de.norm();
		res = PROGRESS;
	}
	Point pTgtP = pEffBase+de;
	if(solve(pEff,oriEff,j,pTgtP,posPri,oriTgt,oriPri))
		return res;
	return LIMITS;
}

#endif
