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

// stores info for a BlobShape
public class BlobShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"blob.png");
    static Icon inv_icon = new ImageIcon(icon_path+"blobinv.png");

    String colorname;
    float area;
    int orient;
    float topLeft_x, topLeft_y, topLeft_z,
	topRight_x, topRight_y, topRight_z,
	bottomLeft_x, bottomLeft_y, bottomLeft_z,
	bottomRight_x, bottomRight_y, bottomRight_z;
    boolean topValid, bottomValid, leftValid, rightValid;

    public BlobShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			 String _colorname,
			 float _centroidx, float _centroidy, float _centroidz,
			 boolean _obstacle, boolean _landmark,
			 float _area, int _orient,
			 float _topLeft_x, float _topLeft_y, float _topLeft_z,
			 float _topRight_x, float _topRight_y, float _topRight_z,
			 float _bottomLeft_x, float _bottomLeft_y, float _bottomLeft_z,
			 float _bottomRight_x, float _bottomRight_y, float _bottomRight_z,
			 boolean _topValid, boolean _bottomValid, boolean _leftValid, boolean _rightValid) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	colorname = _colorname;
	area = _area;
	orient = _orient;
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
	topValid = _topValid;
	bottomValid = _bottomValid;
	leftValid = _leftValid;
	rightValid = _rightValid;
    }

    // should adjust these to take into account the crossbar height for pillar/poster blobs in worldspace
    public float getLeft() { return java.lang.Math.min(topLeft_x,bottomLeft_x); }
    public float getRight() { return java.lang.Math.max(topRight_x,bottomRight_x); }
    public float getTop() { return  java.lang.Math.min(topLeft_y,topRight_y); }
    public float getBottom() { return java.lang.Math.max(bottomLeft_y,bottomRight_y); }


    public String toString() {
	String _orient = (orient==0) ? "groundplane" : (orient==1) ? "pillar" : "poster";
	String _center = "center=(" + centroidx + " " + centroidy + " " + centroidz + ")";
	return super.toString() + " " + colorname + " area=" + area + " " + _center + " " + _orient;
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	float dash1[] = {5.0f/scaling};
    	BasicStroke dashed = new BasicStroke((2.0f/scaling), BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
					     (5.0f/scaling), dash1, (0.0f/scaling)); 

	if ( orient == 0 ) {
	    //check for a valid top
	    if (!topValid) {
		graphics.setStroke(dashed);
	    }
	    else {
	    	graphics.setStroke(new BasicStroke(0.5f/scaling));
	    }
	    graphics.drawLine((int)topLeft_x,(int)topLeft_y,(int)topRight_x,(int)topRight_y);

	    //check for a valid bottom
	    if (!bottomValid) {
		graphics.setStroke(dashed);
	    }
	    else {
	    	graphics.setStroke(new BasicStroke(0.5f/scaling));
	    }
	    graphics.drawLine((int)bottomLeft_x,(int)bottomLeft_y,(int)bottomRight_x,(int)bottomRight_y);

	    //check for a valid left side
	    if (!leftValid) {
		graphics.setStroke(dashed);
	    }
	    else {
	    	graphics.setStroke(new BasicStroke(0.5f/scaling));
	    }
	    graphics.drawLine((int)topLeft_x,(int)topLeft_y,(int)bottomLeft_x,(int)bottomLeft_y);		

	    //check for a valid right side
	    if (!rightValid) {
		graphics.setStroke(dashed);
	    }
	    else {
	    	graphics.setStroke(new BasicStroke(0.5f/scaling));
	    }
	    graphics.drawLine((int)topRight_x,(int)topRight_y,(int)bottomRight_x,(int)bottomRight_y);

	}
	else {
	    if ((!topValid) || (!bottomValid) || (!leftValid) || (!rightValid)) 			{
		graphics.setStroke(dashed);
	    }
	    else {
		graphics.setStroke(new BasicStroke(3.0f/scaling));
	    }	
	    graphics.drawLine((int)bottomLeft_x,(int)bottomLeft_y,(int)bottomRight_x,(int)bottomRight_y);
	    int w = java.lang.Math.max(12,(int)java.lang.Math.abs(bottomRight_y - bottomLeft_y));
	    int midx = (int)((bottomLeft_x + bottomRight_x) / 2);
	    int midy = (int)((bottomLeft_y + bottomRight_y) / 2);
	    graphics.drawLine(midx-w/4,midy,midx+w/4,midy); 
	
	}

    }
}
