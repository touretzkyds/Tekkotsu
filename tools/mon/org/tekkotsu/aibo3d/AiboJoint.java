package org.tekkotsu.aibo3d;

import javax.media.j3d.*;
import javax.vecmath.*;
import java.util.*;
import java.lang.*;

public class AiboJoint {
	public static final int RADIANS=0;
	public static final int DEGREES=1;
	
	public String name;
	public TransformGroup obj;
	public Matrix3f rotPreOffset,rotPostOffset;
	public Matrix3f mul;
	Matrix3f rot;
	public float minX;
	public float maxX;
	public float minY;
	public float maxY;
	public float minZ;
	public float maxZ;
	public float x;
	public float y;
	public float z;
	
	float bound(float n, float l, float h) {
		return Math.max(Math.min(h, n),l);
	}
	
	public void setX(float X) {
		x=bound(X, minX, maxX);
		setJoints();
	}
	
	public void setY(float Y) {
		y=bound(Y, minY, maxY);
		setJoints();
	}
	
	public void setZ(float Z) {
		z=bound(Z, minZ, maxZ);
		setJoints();
	}
	
	public void setX(double X) {
		setX((float)X);
	}
	
	public void setY(double Y) {
		setY((float)Y);
	}
	
	public void setZ(double Z) {
		setZ((float)Z);
	}
	
	public void setXDiff(int X) {
		setX(x+((float)X*0.03f));
	}
	
	public void setYDiff(int Y) {
		setY(y+((float)Y*0.03f));
	}
	
	public void setZDiff(int Z) {
		setZ(z+((float)Z*0.03f));
	}
	
	public void set(float X, float Y, float Z) {
		x=bound(X, minX, maxX);
		y=bound(Y, minY, maxY);
		z=bound(Z, minZ, maxZ);
		setJoints();
	}
	
	
	public AiboJoint mulX(float X) {
		x=bound(X, minX, maxX);
		rot.rotX(x);
		mul.mul(rot);
		return this;
	}
	public AiboJoint mulY(float Y) {
		y=bound(Y, minY, maxY);
		rot.rotY(y);
		mul.mul(rot);
		return this;
	}
	public AiboJoint mulZ(float Z) {
		z=bound(Z, minZ, maxZ);
		rot.rotZ(z);
		mul.mul(rot);
		return this;
	}
	public void apply() {
		mul.mul(rotPostOffset);
		Transform3D t3d=new Transform3D();
		obj.getTransform(t3d);
		t3d.setRotation(mul);
		obj.setTransform(t3d);
		mul.set(rotPreOffset);
	}
	
	void setJoints() {
		Matrix3f accum=new Matrix3f(rotPreOffset);
		rot.rotX(x);
		accum.mul(rot);
		rot.rotY(y);
		accum.mul(rot);
		rot.rotZ(z);
		accum.mul(rot);
		accum.mul(rotPostOffset);
		Transform3D t3d=new Transform3D();
		obj.getTransform(t3d);
		t3d.setRotation(accum);
		obj.setTransform(t3d);
		mul.set(rotPreOffset);
	}
	
	public AiboJoint (TransformGroup tg, String n, float minx, float maxx,float miny, float maxy, float minz, float maxz) {
		this(tg, n, minx, maxx, miny, maxy, minz, maxz, DEGREES);
	}
	
	public AiboJoint (TransformGroup tg, String n, float minx, float maxx, float miny, float maxy, float minz, float maxz, int units) {
		if (units==RADIANS) {
			obj=tg;
			minX=minx;
			maxX=maxx;
			minY=miny;
			maxY=maxy;
			minZ=minz;
			maxZ=maxz;
		} else if (units==DEGREES) {
			obj=tg;
			minX=deg2rad(minx);
			maxX=deg2rad(maxx);
			minY=deg2rad(miny);
			maxY=deg2rad(maxy);
			minZ=deg2rad(minz);
			maxZ=deg2rad(maxz);
		} else {
			System.out.println("error: unknown units");
			System.exit(1);
		}
		x=0.0f;
		y=0.0f;
		z=0.0f;
		name=n;
		rotPreOffset = new Matrix3f();
		rotPreOffset.setIdentity();
		rotPostOffset = new Matrix3f();
		rotPostOffset.setIdentity();
		mul = new Matrix3f();
		mul.setIdentity();
		rot=new Matrix3f();
	}
	
	public void setRotationalPreOffset(float x, float y, float z) {
		rotPreOffset.rotX(x);
		Matrix3f rot=new Matrix3f();
		rot.rotY(y);
		rotPreOffset.mul(rot);
		rot.rotZ(z);
		rotPreOffset.mul(rot);
		mul.set(rotPreOffset);
	}
	
	public void setRotationalPostOffset(float x, float y, float z) {
		rotPostOffset.rotX(x);
		Matrix3f rot=new Matrix3f();
		rot.rotY(y);
		rotPostOffset.mul(rot);
		rot.rotZ(z);
		rotPostOffset.mul(rot);
	}
	
	public float deg2rad (float deg) {
		return (deg*(float)Math.PI)/180.0f;
	}
	
	public String toString () {
		return name;
	}
}
