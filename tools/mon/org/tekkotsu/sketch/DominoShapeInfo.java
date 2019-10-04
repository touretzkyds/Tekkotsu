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

// Domino extends Brick

public class DominoShapeInfo extends BrickShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"domino.png");
    static Icon inv_icon = new ImageIcon(icon_path+"dominoinv.png");

    int lowValue, highValue;

    public DominoShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			   float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			   float _GFLx, float _GFLy,
			   float _GFRx, float _GFRy,
			   float _GBLx, float _GBLy,
			   float _GBRx, float _GBRy,
			   float _TFLx, float _TFLy,
			   float _TFRx, float _TFRy,
			   float _TBLx, float _TBLy,
			   float _TBRx, float _TBRy,
			   int _lowValue, int _highValue) {
	super(_gui, _id, _parentId, _name, _color,
	      _centroidx, _centroidy, _centroidz, _obstacle, _landmark,
	      _GFLx, _GFLy, _GFRx, _GFRy, _GBLx, _GBLy,_GBRx,_GBRy,
	      _TFLx, _TFLy, _TFRx, _TFRy, _TBLx, _TBLy,_TBRx,_TBRy);
	lowValue = _lowValue;
	highValue = _highValue;
    }

    public String toString() {
	String domino = "[" + lowValue + "|" + highValue + "] at (" +
	    centroidx + " " + centroidy + " " + centroidz + ")";
	return superToString() + " " + domino;
    }

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	else
	    return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	super.renderTo(graphics, scaling);
	float Fmidx = (TFLx + TFRx)/2;
	float Fmidy = (TFLy + TFRy)/2;
	float Bmidx = (TBLx + TBRx)/2;
	float Bmidy = (TBLy + TBRy)/2;
	float fblensq = (Fmidx-Bmidx)*(Fmidx-Bmidx) + (Fmidy-Bmidy)*(Fmidy-Bmidy);
	float Lmidx = (TFLx + TBLx)/2;
	float Lmidy = (TFLy + TBLy)/2;
	float Rmidx = (TFRx + TBRx)/2;
	float Rmidy = (TFRy + TBRy)/2;
	float lrlensq = (Lmidx-Rmidx)*(Lmidx-Rmidx) + (Lmidy-Rmidy)*(Lmidy-Rmidy);
	if ( fblensq < lrlensq )
	    graphics.drawLine((int)Fmidx, (int)Fmidy, (int)Bmidx, (int)Bmidy);
	else
	    graphics.drawLine((int)Lmidx, (int)Lmidy, (int)Rmidx, (int)Rmidy);
    }

}
