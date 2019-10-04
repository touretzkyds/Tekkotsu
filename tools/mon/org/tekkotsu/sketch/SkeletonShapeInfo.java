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

// stores info for a SkeletonShape
public class SkeletonShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"skeleton.png");
    static Icon inv_icon = new ImageIcon(icon_path+"skeletoninv.png");


    SkeletonJoint joints[];
    float p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y;
    

    public SkeletonShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			     float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			     SkeletonJoint _joints[]) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	joints = _joints;
    }

    public float getLeft() { 
	if ( space == SketchGUI.Space.cam )
	    return java.lang.Math.min(java.lang.Math.min(p0x,p1x),java.lang.Math.min(p2x,p3x));
	else
	    return centroidx;
    }

    public float getRight() {
	if ( space == SketchGUI.Space.cam )
	    return java.lang.Math.max(java.lang.Math.max(p0x,p1x),java.lang.Math.max(p2x,p3x));
	else
	    return centroidx;
    }

    public float getTop() {
	if ( space == SketchGUI.Space.cam )
	    return java.lang.Math.min(java.lang.Math.min(p0y,p1y),java.lang.Math.min(p2y,p3y));
	else
	    return centroidy;
    }

    public float getBottom() {
	if ( space == SketchGUI.Space.cam )
	    return java.lang.Math.max(java.lang.Math.max(p0y,p1y),java.lang.Math.max(p2y,p3y));
	else
	    return centroidy;
    }

    public String toString() {
	String tellCenter = "center=(" + centroidx + " " + centroidy + " " + centroidz + ")";
	int numJoints = 0;
	for (int i=0; i < joints.length; i++)
	    if ( joints[i] != null ) ++numJoints;
	return super.toString() + " numJoints=" + numJoints;
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	int on = inverted ? 0 : 200;
	int off = 200 - on;
 
	if ( space == SketchGUI.Space.cam ) {
	    graphics.setStroke(new BasicStroke(2.0f/scaling));

	    graphics.setColor(new Color(off, on, off)); // green
	    graphics.draw(new Line2D.Float(p0x,p0y,p1x,p1y));
	    
	    graphics.setColor(new Color(off, off, on)); // blue
	    graphics.draw(new Line2D.Float(p1x,p1y,p2x,p2y));
	    graphics.draw(new Line2D.Float(p2x,p2y,p3x,p3y));
	    
	    graphics.setColor(new Color(on, off, off)); // red
	    graphics.draw(new Line2D.Float(p3x,p3y,p0x,p0y));
	}
	else {
	    graphics.setStroke(new BasicStroke(1.0f/scaling));

	    float side = 5.0f/scaling;
	    graphics.setColor(new Color(off, on, off)); // green
	    graphics.draw(new Line2D.Float(centroidx-side, centroidy-side, centroidx-side, centroidy+side));

	    graphics.setColor(new Color(off, off, on)); // blue
	    graphics.draw(new Line2D.Float(centroidx-side, centroidy-side, centroidx+side, centroidy-side));
	    graphics.draw(new Line2D.Float(centroidx+side, centroidy-side, centroidx+side, centroidy+side));

	    graphics.setColor(new Color(on, off, off)); // red
	    graphics.draw(new Line2D.Float(centroidx-side, centroidy+side, centroidx+side, centroidy+side));
	}
    }
}
