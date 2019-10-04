package org.tekkotsu.sketch;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Graphics;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import java.awt.Rectangle;
import java.awt.geom.*;
import java.awt.BasicStroke;
import javax.media.j3d.*;
import javax.vecmath.*;
import com.sun.j3d.utils.geometry.Sphere;


// stores info for a EllipseShape
// note that ellipse center is same as centroid
public class EllipseShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"ellipse.png");
    static Icon inv_icon = new ImageIcon(icon_path+"ellipseinv.png");
    float semimajor, semiminor; // length of semimajor axes
    float orientation; // orientation of principal axis
    
    public EllipseShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			    float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			    float _semimajor, float _semiminor,
			    float _orientation) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	semimajor = _semimajor;
	semiminor = _semiminor;
	orientation = _orientation;
    }
    
    public float getTop() {
	double t = Math.atan(semiminor/Math.tan(orientation)/semimajor)
	    + Math.PI;
	double r = centroidy + semiminor*Math.sin(t)*Math.cos(orientation)
	    + semimajor*Math.cos(t)*Math.sin(orientation);
	return (float)r;
    }

    public float getBottom() {
	double t = Math.atan(semiminor/Math.tan(orientation)/semimajor);
	double r = centroidy + semiminor*Math.sin(t)*Math.cos(orientation)
	    + semimajor*Math.cos(t)*Math.sin(orientation);
	return (float)r;
    }

    public float getLeft() {
	double t = Math.atan((-semiminor*Math.tan(orientation)) / semimajor) + Math.PI;
	double r = centroidx + semimajor*Math.cos(t)*Math.cos(orientation)
	    - semiminor*Math.sin(t)*Math.sin(orientation);
	return (float)r;
    }
    
    public float getRight() {
	double t = Math.atan(( -semiminor*Math.tan(orientation)) / semimajor);
	double r = centroidx + semimajor*Math.cos(t)*Math.cos(orientation)
	    - semiminor*Math.sin(t)*Math.sin(orientation);
	return (float)r;
    }
    
    public String toString() {
	return (super.toString() + " center=(" + centroidx + " " + centroidy + " " + centroidz + ")"
		+ " smaj=" + semimajor 
		+ ", smin=" + semiminor 
		+ ", orient=" + orientation);
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	return icon; 
    }
    
    public void renderTo(Graphics2D graphics, float scaling) {
	graphics.setStroke(new BasicStroke(1.0f/scaling));
	
	graphics.transform(AffineTransform.getRotateInstance(getOrientation(), getCentroidX(), getCentroidY())); // fwd rotation
	// for proper orientation, rotate the coordinate because we cannot rotate the oval itself
	graphics.draw(new Ellipse2D.Float(getCentroidX()-semimajor, getCentroidY()-semiminor, 2 * semimajor, 2 * semiminor));
	graphics.transform(AffineTransform.getRotateInstance(-getOrientation(), getCentroidX(), getCentroidY())); // back rotation
    }
    
    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);

        Sphere ellipse = new Sphere(1.0f, ap);
        Matrix3f m = new Matrix3f(semimajor/scaling, 0.0f, 0.0f, 0.0f, semiminor/scaling, 0.0f, 0.0f, 0.0f, 0.009f);
        
        TransformGroup newTrans = new TransformGroup();
        Transform3D t3d = new Transform3D(m, new Vector3f(centroidx/scaling, centroidy/scaling, 1), 1.0f);
        newTrans.setTransform(t3d);
        newTrans.addChild(ellipse);
        vinfoBranch.addChild(newTrans);
        vinfoGroup.addChild(vinfoBranch);
    }
    
    public float getSemiMajor() { return semimajor; }
    public float getSemiMinor() { return semiminor; }
    public float getOrientation() { return orientation; }
}
