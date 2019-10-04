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

// stores info for a BrickShape
public class BrickShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"brick.png");
    static Icon inv_icon = new ImageIcon(icon_path+"brickinv.png");
    
    float GFLx, GFLy;
    float GFRx, GFRy;
    float GBLx, GBLy;
    float GBRx, GBRy;
    float TFLx, TFLy;
    float TFRx, TFRy;
    float TBLx, TBLy;
    float TBRx, TBRy;
    
    public BrickShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			  float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			  float _GFLx, float _GFLy,
			  float _GFRx, float _GFRy,
			  float _GBLx, float _GBLy,
			  float _GBRx, float _GBRy,
			  float _TFLx, float _TFLy,
			  float _TFRx, float _TFRy,
			  float _TBLx, float _TBLy,
			  float _TBRx, float _TBRy) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	
	GFLx = _GFLx; GFLy = _GFLy;
	GFRx = _GFRx; GFRy = _GFRy;
	GBLx = _GBLx; GBLy = _GBLy;
	GBRx = _GBRx; GBRy = _GBRy;
	TFLx = _TFLx; TFLy = _TFLy;
	TFRx = _TFRx; TFRy = _TFRy;
	TBLx = _TBLx; TBLy = _TBLy;
	TBRx = _TBRx; TBRy = _TBRy;
    }
	
    // returns left-most coordinate of object
    public float getLeft() { return Math.min(Math.min(GFLx,GFRx), Math.min(GBLx,GBRx)); }
    // returns right-most coordinate of object
    public float getRight() { return Math.max(Math.max(GFLx,GFRx), Math.max(GBLx,GBRx)); }
    // returns top-most coordinate of object
    public float getTop() { return Math.min(Math.min(GFLy,GFRy), Math.min(GBLy,GBRy)); }
    // returns bottom-most coordinate of object
    public float getBottom() { return Math.max(Math.max(GFLy,GFRy), Math.max(GBLy,GBRy)); }
    
    public String toString() {
	String _brick = "brick=(" + centroidx + " " + centroidy + " " + centroidz + ")";
	return (super.toString() + " " + _brick); }

    public String superToString() { // called by subclasses such as Domino
	return super.toString();
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	Rectangle bounds = graphics.getClipBounds();
	graphics.drawLine((int)GFLx, (int)GFLy, (int)GFRx, (int)GFRy);
	graphics.drawLine((int)GBLx, (int)GBLy, (int)GBRx, (int)GBRy);
	graphics.drawLine((int)TFLx, (int)TFLy, (int)TFRx, (int)TFRy);
	graphics.drawLine((int)TBLx, (int)TBLy, (int)TBRx, (int)TBRy);
	graphics.drawLine((int)GFLx, (int)GFLy, (int)GBLx, (int)GBLy);
	graphics.drawLine((int)GFRx, (int)GFRy, (int)GBRx, (int)GBRy);
	graphics.drawLine((int)TFLx, (int)TFLy, (int)TBLx, (int)TBLy);
	graphics.drawLine((int)TFRx, (int)TFRy, (int)TBRx, (int)TBRy);
	graphics.drawLine((int)GFLx, (int)GFLy, (int)TFLx, (int)TFLy);
	graphics.drawLine((int)GFRx, (int)GFRy, (int)TFRx, (int)TFRy);
	graphics.drawLine((int)GBLx, (int)GBLy, (int)TBLx, (int)TBLy);
	graphics.drawLine((int)GBRx, (int)GBRy, (int)TBRx, (int)TBRy);
    }

}

