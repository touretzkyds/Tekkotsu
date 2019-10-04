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

// stores info for a CrossShape
public class CrossShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"cross.png");
    static Icon inv_icon = new ImageIcon(icon_path+"crossinv.png");
    // x/y coordinates of endpoints
    float e1x, e1y, e2x, e2y, e3x, e3y, e4x, e4y;
    float armWidth;
   
    public CrossShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			  float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			  float _e1x, float _e1y, float _e2x, float _e2y,
			  float _e3x, float _e3y, float _e4x, float _e4y,
			  float _armWidth) {
		
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	e1x = _e1x;
	e1y = _e1y;
	e2x = _e2x;
	e2y = _e2y;
	e3x = _e3x;
	e3y = _e3y;
	e4x = _e4x;
	e4y = _e4y;
	armWidth = _armWidth;
    }
	
    // returns left-most coordinate of object
    public float getLeft() { return java.lang.Math.min(java.lang.Math.min(e1x,e2x),java.lang.Math.min(e3x,e4x)); }
    // returns right-most coordinate of object
    public float getRight() { return java.lang.Math.max(java.lang.Math.max(e1x,e2x),java.lang.Math.max(e3x,e4x)); }
    // returns top-most coordinate of object
    public float getTop() { return java.lang.Math.min(java.lang.Math.min(e1y,e2y),java.lang.Math.min(e3y,e4y)); }
    // returns bottom-most coordinate of object
    public float getBottom() { return java.lang.Math.max(java.lang.Math.max(e1y,e2y),java.lang.Math.max(e3y,e4y)); }

    public String toString() {
	return (super.toString()
		+ " (" + centroidx + "," + centroidy + "," + centroidz + ")");
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
        graphics.setStroke(new BasicStroke(1.0f/scaling));
        graphics.draw(new Line2D.Float(e1x,e1y,e2x,e2y));
        graphics.draw(new Line2D.Float(e3x,e3y,e4x,e4y));
    }

    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
	/*

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
	*/
    }

    public float getE1X() { return e1x; }
    public float getE1Y() { return e1y; }
    public float getE2X() { return e2x; }
    public float getE2Y() { return e2y; }

    public float getIdX() { return getE1X(); }
    public float getIdY() { return getE1Y(); }
    
}

