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

// stores info for a PyramidShape
public class PyramidShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"pyramid.png");
    static Icon inv_icon = new ImageIcon(icon_path+"pyramidinv.png");
    
    float FLx, FLy;
    float FRx, FRy;
    float BLx, BLy;
    float BRx, BRy;
    float Topx, Topy;
    
    public PyramidShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			    float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			    float _FLx, float _FLy,
			    float _FRx, float _FRy,
			    float _BLx, float _BLy,
			    float _BRx, float _BRy,
			    float _Topx, float _Topy) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	
	FLx = _FLx; FLy = _FLy;
	FRx = _FRx; FRy = _FRy;
	BLx = _BLx; BLy = _BLy;
	BRx = _BRx; BRy = _BRy;
	Topx = _Topx; Topy = _Topy;
    }
	
    // returns left-most coordinate of object
    public float getLeft() { return centroidx; }
    // returns right-most coordinate of object
    public float getRight() { return centroidx; }
    // returns top-most coordinate of object
    public float getTop() { return centroidy; }
    // returns bottom-most coordinate of object
    public float getBottom() { return centroidy; }
    
    public String toString() {
	String _pyramid = "pyramid=(" + centroidx + " " + centroidy + " " + centroidz + ")";
	return (super.toString() + " " + _pyramid); }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	Rectangle bounds = graphics.getClipBounds();
	graphics.drawLine((int)FLx, (int)FLy, (int)FRx, (int)FRy);
	graphics.drawLine((int)BLx, (int)BLy, (int)BRx, (int)BRy);
	graphics.drawLine((int)FLx, (int)FLy, (int)BLx, (int)BLy);
	graphics.drawLine((int)FRx, (int)FRy, (int)BRx, (int)BRy);
	graphics.drawLine((int)Topx, (int)Topy, (int)BLx, (int)BLy);
	graphics.drawLine((int)Topx, (int)Topy, (int)BRx, (int)BRy);
	graphics.drawLine((int)Topx, (int)Topy, (int)FLx, (int)FLy);
	graphics.drawLine((int)Topx, (int)Topy, (int)FRx, (int)FRy);
    }

}

