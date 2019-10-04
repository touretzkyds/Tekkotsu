package org.tekkotsu.aibo3d;

import javax.vecmath.*;

public class Kinematics {
  public static final int FRONT_LEFT=0;
  public static final int FRONT_RIGHT=1;
  public static final int BACK_LEFT=2;
  public static final int BACK_RIGHT=3;
 
  static final Vector3d f_body_to_shoulder=
                                   new Vector3d( 59.50, 59.20,   0.00); 
  static final Vector3d f_leg_shoulder_to_knee=
                                   new Vector3d( 12.80,  0.50, -64.00);
  static final Vector3d f_leg_knee_to_ball=
                                   new Vector3d(-18.00,  0.00, -67.23);
  static final Vector3d h_body_to_shoulder=
                                   new Vector3d(-59.50, 59.20,   0.00); 
  static final Vector3d h_leg_shoulder_to_knee=
                                   new Vector3d(-12.80,  0.50, -64.00);
  static final Vector3d h_leg_knee_to_ball=
                                   new Vector3d( 18.00,  0.00, -74.87);
  static final Vector3d f_upper=f_leg_shoulder_to_knee;
  static final Vector3d f_lower=f_leg_knee_to_ball;
  static final Vector3d h_upper=h_leg_shoulder_to_knee;
  static final Vector3d h_lower=h_leg_knee_to_ball;

  static final double rotator_min  = RAD(-117.0);
  static final double rotator_max  = RAD( 117.0);
  static final double shoulder_min = RAD( -11.0);
  static final double shoulder_max = RAD(  97.0);
  static final double knee_max     = RAD( 147.0);
  static final double knee_min     = RAD( -27.0);

  static final double rotator_kmin  = RAD(-90.0);
  static final double rotator_kmax  = RAD( 90.0);
  static final double shoulder_kmin = shoulder_min;
  static final double shoulder_kmax = RAD( 90.0);
  static final double knee_kmax     = knee_max;
  static final double f_knee_kmin   = 0.2616;
  static final double h_knee_kmin   = 0.2316;

  public Kinematics () {
  }

  public static void main(String args[]) {
    Vector3d target=getForward(0, new LegConfiguration(0.0,0.0,0.0));
    System.out.println(target);
    target.y=target.y+20.0;
    LegConfiguration angles=getInverse(0, target);
    System.out.println(angles);
    System.out.println(getForward(0, angles));
  }

  final static double RAD(double deg) {
    return (deg*Math.PI)/180.0;
  }

  public static LegConfiguration getInverse(int leg, Vector3d target) {
    Vector3d pos=new Vector3d();
    Vector3d targ=new Vector3d(target);
    Matrix3d rot=new Matrix3d();
    double rotator=0.0, shoulder=0.0, knee=0.0;
    double a, b, d, dist;

    if (leg%2!=0) targ.y = -targ.y;

    if (leg<2) {
      targ.sub(f_body_to_shoulder);
      dist=targ.lengthSquared();

      a=-2*(f_upper.x*f_lower.z - f_upper.z*f_lower.x);
      b= 2*(f_upper.x*f_lower.x + f_upper.z*f_lower.z);
      d=(dist-f_upper.lengthSquared()-f_lower.lengthSquared()
         -2*f_upper.y*f_lower.y);
      knee=GetTrigAngle(a,b,d,f_knee_kmin,knee_kmax,true);

      rot.rotY(-knee);
      rot.transform(f_leg_knee_to_ball, pos);
      pos.add(f_leg_shoulder_to_knee);
      shoulder=GetTrigAngle(-pos.z,pos.y,targ.y,shoulder_kmin,
                            shoulder_kmax,false);

      pos.z=Math.sin(shoulder)*pos.y + Math.cos(shoulder)*pos.z;
      rotator=GetTrigAngle(-pos.z, pos.x, targ.x, rotator_min, rotator_max,
                           targ.z>0.0);
    } else {
      targ.sub(h_body_to_shoulder);
      dist=targ.lengthSquared();

      a= 2*(h_upper.x*h_lower.z - h_upper.z*h_lower.x);
      b= 2*(h_upper.x*h_lower.x + h_upper.z*h_lower.z);
      d=(dist-h_upper.lengthSquared()-h_lower.lengthSquared()
         -2*h_upper.y*h_lower.y);
      knee=GetTrigAngle(a,b,d,h_knee_kmin,knee_kmax,true);

      rot.rotY(knee);
      rot.transform(h_leg_knee_to_ball, pos);
      pos.add(h_leg_shoulder_to_knee);
      shoulder=GetTrigAngle(-pos.z,pos.y,targ.y,shoulder_kmin,
                            shoulder_kmax,false);

      pos.z=Math.sin(shoulder)*pos.y + Math.cos(shoulder)*pos.z;
      rotator=GetTrigAngle(-pos.z, -pos.x, -targ.x, rotator_min, rotator_max,
                           targ.z>0.0);

    }
    
    return new LegConfiguration (rotator, shoulder, knee);
  }

  static double GetTrigAngle(double a, double b, double d, double min, double max,
                     boolean add) {
    double theta;
    double t, f, c;

    f=d/Math.sqrt(a*a + b*b);
    if (Math.abs(f) > 1.0) f=(f > 0.0) ? 1.0 : -1.0;

    t=Math.atan2(a,b);
    c=Math.acos(f);

    theta=add?(t+c):(t-c);
    if (theta<min) return min;
    if (theta>max) return max;
    return theta;
  }
 
  public static Vector3d getForward(int leg, LegConfiguration angles) {
    Vector3d pos=new Vector3d();
    Matrix3d rot1=new Matrix3d();
    Matrix3d rot2=new Matrix3d();

    if (leg < 2) {
      rot1.rotY(-angles.knee);
      rot1.transform(f_leg_knee_to_ball,pos);

      pos.add(f_leg_shoulder_to_knee);
      rot1.rotX(angles.shoulder);
      rot2.rotY(-angles.rotator);
      rot1.mul(rot2);
      rot1.transform(pos);

      pos.add(f_body_to_shoulder);
    } else {
      rot1.rotY(angles.knee);
      rot1.transform(h_leg_knee_to_ball,pos);

      pos.add(h_leg_shoulder_to_knee);
      rot1.rotX(angles.shoulder);
      rot2.rotY(angles.rotator);
      rot1.mul(rot2);
      rot1.transform(pos);

      pos.add(h_body_to_shoulder);
    }
    if (leg%2!=0) {
      pos.y=-pos.y;  
    }
    return pos; 
  }
}
