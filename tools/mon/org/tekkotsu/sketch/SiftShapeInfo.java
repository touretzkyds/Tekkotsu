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

// stores info for a SiftShape
public class SiftShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"sift.png");
    static Icon inv_icon = new ImageIcon(icon_path+"siftinv.png");

    String objectName;
    String modelName;
    int objectID;
    float topLeft_x, topLeft_y, topLeft_z,
	topRight_x, topRight_y, topRight_z,
	bottomLeft_x, bottomLeft_y, bottomLeft_z,
	bottomRight_x, bottomRight_y, bottomRight_z;

    
    public SiftShapeInfo(SketchGUI _gui, int _id, int _parentId, int _objectID, 
			 String _name, String _objectName, String _modelName, Color _color,
			 float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			 float _topLeft_x, float _topLeft_y, float _topLeft_z,
			 float _topRight_x, float _topRight_y, float _topRight_z,
			 float _bottomLeft_x, float _bottomLeft_y, float _bottomLeft_z,
			 float _bottomRight_x, float _bottomRight_y, float _bottomRight_z) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	objectName = _objectName;
	modelName = _modelName;
	objectID = _objectID;
	topLeft_x = _topLeft_x;
	topLeft_y = _topLeft_y;
	topLeft_z = _topLeft_z;
	topRight_x = _topRight_x;
	topRight_y = _topRight_y;
	topRight_z = _topRight_z;
	bottomLeft_x = _bottomLeft_x;
	bottomLeft_y = _bottomLeft_y;
	bottomLeft_z = _bottomLeft_z;
	bottomRight_x = _bottomRight_x;
	bottomRight_y = _bottomRight_y;
	bottomRight_z = _bottomRight_z;
    }

    // should adjust these to take into account the crossbar height for pillar/poster sifts in worldspace
    public float getLeft() { return java.lang.Math.min(topLeft_x,bottomLeft_x); }
    public float getRight() { return java.lang.Math.max(topRight_x,bottomRight_x); }
    public float getTop() { return  java.lang.Math.min(topLeft_y,topRight_y); }
    public float getBottom() { return java.lang.Math.max(bottomLeft_y,bottomRight_y); }


    public String toString() {
	String _center = "center=(" + centroidx + " " + centroidy + " " + centroidz + ")";
	return super.toString() + " " + objectName+ "(" + modelName + ") " + _center;
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	graphics.setColor(new Color(200, 0, 0));
	graphics.setStroke(new BasicStroke(3.0f/scaling));
        graphics.drawLine((int)topLeft_x,(int)topLeft_y,(int)topRight_x,(int)topRight_y);
	graphics.drawLine((int)topRight_x,(int)topRight_y,(int)bottomRight_x,(int)bottomRight_y);
	graphics.drawLine((int)bottomRight_x,(int)bottomRight_y,(int)bottomLeft_x,(int)bottomLeft_y);
	graphics.drawLine((int)bottomLeft_x,(int)bottomLeft_y,(int)topLeft_x,(int)topLeft_y);
    }
}
