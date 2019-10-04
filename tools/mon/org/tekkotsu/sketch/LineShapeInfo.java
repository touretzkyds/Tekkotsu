package org.tekkotsu.sketch;

import java.awt.Graphics2D;
import java.awt.Graphics;
import java.awt.Color;
import java.awt.Font;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import java.awt.Rectangle;
import java.awt.geom.*;
import java.awt.BasicStroke;
import javax.media.j3d.*;
import javax.vecmath.*;
import com.sun.j3d.utils.geometry.Box;

// stores info for a LineShape
public class LineShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"line.png");
    static Icon inv_icon = new ImageIcon(icon_path+"lineinv.png");
    // x/y coordinates of endpoints, with variances
    float e1x, e1y, e1v;
    float e2x, e2y, e2v;
    float r, theta;
    boolean end1_valid, end1_active, end2_valid, end2_active;
    
    public LineShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			 float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			 float _e1x, float _e1y, float _e1v, 
			 float _e2x, float _e2y, float _e2v,
			 float _r, float _theta, 
			 boolean _end1_valid, boolean _end1_active,
			 boolean _end2_valid, boolean _end2_active) {
		
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	e1x = _e1x;
	e1y = _e1y;
	e1v = _e1v;
	e2x = _e2x;
	e2y = _e2y;
	e2v = _e2v;
	r = _r;
	theta = _theta;
	end1_valid = _end1_valid;
	end1_active = _end1_active;
	end2_valid = _end2_valid;
	end2_active = _end2_active;
    }
	
    // returns left-most coordinate of object
    public float getLeft() { return java.lang.Math.min(e1x,e2x); }
    // returns right-most coordinate of object
    public float getRight() { return java.lang.Math.max(e1x,e2x); }
    // returns top-most coordinate of object
    public float getTop() { return java.lang.Math.min(e1y,e2y); }
    // returns bottom-most coordinate of object
    public float getBottom() { return java.lang.Math.max(e1y,e2y); }

    public String toString() {
	double len = Math.floor(Math.sqrt((e1x-e2x)*(e1x-e2x) + (e1y-e2y)*(e1y-e2y))*10) / 10;
	return (super.toString()
		+ " len=" + len + " (" + e1x + " " + e1y + " " + e1v + ")---("
		+ e2x + " " + e2y + " " + e2v + ")");
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
        final float big_slope = (float)1e3;
        // Swap points if point 1 is to the right of point 2
        if ( e1x > e2x ) {
            boolean t_valid = end1_valid;
            boolean t_active = end1_active;
            end1_valid = end2_valid;
            end1_active = end2_active;
            end2_valid = t_valid;
            end2_active = t_active;
            float tx=e1x;
            float ty=e1y;
            e1x=e2x;
            e1y=e2y;
            e2x=tx;
            e2y=ty;
        }
        float m, b;
        if ( e2x-e1x > 1/big_slope )
            m = (e2y-e1y) / (e2x-e1x);
        else
            m = big_slope;
        b = e1y - e1x*m;
        float e1xi, e1yi, e2xi, e2yi;
        SketchPanel p = gui.sketchPanel;
        if (end1_active) {
            e1xi = e1x; e1yi = e1y;
        } else {
            e1xi = p.leftBound;
            e1yi = e1xi*m + b;
            // due to a bug in OpenJDK drawLine, we can't let
            // abs(e1yi) exceed ~ 16300, but we still want to
            // maintain the proper slope, so recalculate e1xi if
            // necessary.
            if (e1yi < p.topBound) {
                e1yi = p.topBound;
                e1xi = (e1yi-b) / m;
            } else if (e1yi > p.bottomBound) {
                e1yi = p.bottomBound;
                e1xi = (e1yi-b) / m;
            }
        };
        if (end2_active) {
            e2xi = e2x; e2yi = e2y;
        } else {
            // guard against OpenJDK drawLine bug
            e2xi = p.rightBound;
            e2yi = e2xi*m + b;
            if (e2yi < p.topBound) {
                e2yi = p.topBound;
                e2xi = (e2yi-b) / m;
            } else if (e2yi > p.bottomBound) {
                e2yi = p.bottomBound;
                e2xi = (e2yi-b) / m;
            }
        };
        graphics.setStroke(new BasicStroke(1.0f/scaling));
        graphics.draw(new Line2D.Float(e1xi,e1yi,e2xi,e2yi));

        // draw endcaps if endpoints are valid
        if(end1_valid || end2_valid) {
            final float dl = 5/scaling;
            final float xo = dl*(float)Math.cos(theta);
            final float yo = dl*(float)Math.sin(theta);
            if (end1_valid)
                graphics.draw(new Line2D.Float((e1x+xo), (e1y+yo), (e1x-xo), (e1y-yo)));
            if (end2_valid)
                graphics.draw(new Line2D.Float((e2x+xo), (e2y+yo), (e2x-xo), (e2y-yo)));
        }
	
	// draw circles for variance
	// graphics.drawOval((int)e1x,(int)e1y,(int)(e1v/2),(int)(e1v/2));
	// graphics.drawOval((int)e2x,(int)e2y,(int)(e2v/2),(int)(e2v/2));
    }

    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);

        // There is a more efficient way of doing this, however the alternative will only render 2d lines. Consider the second method if efficiency is an issue. (writte the second method)
        float distance = (float)Math.sqrt(Math.pow(e2x - e1x, 2) + Math.pow(e2y - e1y, 2));
        Box lineBox = new Box(distance / (2*scaling), 0.005f, 0.008f, ap);
        float rot = (float)Math.atan2((e2y - e1y), (e2x - e1x));
        float midPointx = (e2x + e1x) / 2;
        float midPointy = (e2y + e1y) / 2;
        TransformGroup lineTrans = new TransformGroup();
        Transform3D l3d = new Transform3D();
        l3d.rotZ(rot);
        l3d.setTranslation(new Vector3f(midPointx / scaling, midPointy / scaling, 0.008f));
        lineTrans.setTransform(l3d);
        lineTrans.addChild(lineBox);
        vinfoBranch.addChild(lineTrans);
        vinfoGroup.addChild(vinfoBranch);
    }

    public float getE1X() { return e1x; }
    public float getE1Y() { return e1y; }
    public float getE1V() { return e1v; }
    public float getE2X() { return e2x; }
    public float getE2Y() { return e2y; }
    public float getE2V() { return e2v; }
    public float getR() { return r; }
    public float getTheta() { return theta; }

    public float getIdX() { return getE1X(); }
    public float getIdY() { return getE1Y(); }
    
}

