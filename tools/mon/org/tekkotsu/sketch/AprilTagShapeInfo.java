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
import com.sun.j3d.utils.geometry.Box;
import com.sun.j3d.utils.geometry.Text2D;
import javax.vecmath.*;

// stores info for a AprilTagShape
public class AprilTagShapeInfo extends ShapeInfo {
    static Icon icon = new ImageIcon(icon_path+"apriltag.png");
    static Icon inv_icon = new ImageIcon(icon_path+"apriltaginv.png");

    int tagID;
    float orientation;
    int hammingDistance;
    float p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y;
    Quat4f q;

    
    public AprilTagShapeInfo(SketchGUI _gui, int _id, int _parentId, String _name, Color _color,
			     float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			     int _tagID, float _orientation, float _q[], int _hammingDistance, float p[][]) {
	super(_gui, _id, _parentId, _name, _color, _centroidx, _centroidy, _centroidz, _obstacle, _landmark);
	tagID = _tagID;
	orientation = _orientation;
	hammingDistance = _hammingDistance;
	p0x = p[0][0];
	p0y = p[0][1];
	p1x = p[1][0];
	p1y = p[1][1];
	p2x = p[2][0];
	p2y = p[2][1];
	p3x = p[3][0];
	p3y = p[3][1];
        q = new Quat4f(_q[1], _q[2], _q[3], _q[0]);
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
	return super.toString() + " tagID=" + tagID + " " + tellCenter + " orient=" + orientation + " hamming=" + hammingDistance;
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
	    //side = side*7;

	    float o = orientation; // - (float)Math.PI;
	    float c = side * (float)(Math.cos(o));
	    float s = side * (float)(Math.sin(o));

	    float blx = centroidx + s + c/2;
	    float bly = centroidy - c + s/2;

	    float brx = centroidx - s + c/2;
	    float bry = centroidy + c + s/2;

	    float tlx = centroidx - c/2 + s/2;
	    float tly = centroidy - c - s;

	    float trx  = centroidx - s - c/2;
	    float try_ = centroidy + c - s/2;

	    //float trx = centroidx + s - c/2;
	    //float try_ = centroidy + c;

	    graphics.setColor(new Color(off, on, off)); // green
	    graphics.draw(new Line2D.Float(blx, bly, brx, bry));

	    graphics.setColor(new Color(off, off, on)); // blue
	    graphics.draw(new Line2D.Float(brx, bry, trx, try_));
	    graphics.draw(new Line2D.Float(trx, try_, tlx, tly));
	    graphics.setColor(new Color(on, off, off)); // red
	    graphics.draw(new Line2D.Float(blx, bly, tlx, tly));

	    //graphics.setColor(new Color(off, off, off)); // black
	    //graphics.draw(new Ellipse2D.Float(centroidx, centroidy, 15, 15));
	    //graphics.draw(new Ellipse2D.Float(tlx, tly, 15, 15));
	}
    }

    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
        BranchGroup vinfoBranch = new BranchGroup();
        vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);

        // Letter sized tag shape (size will change accordingly to



























        // pixel coordinates in the future)
        Box tagBox = new Box((8.5f * 2.54f) / scaling, (11.0f * 2.54f) / scaling, 0.001f, ap);

        // TagID in form of text
        Text2D tagNum = new Text2D("" + tagID, new Color3f(Color.white), "SansSerif", 12, Font.BOLD);
        PolygonAttributes polyAttrib = new PolygonAttributes();
        polyAttrib.setCullFace(PolygonAttributes.CULL_NONE);
        polyAttrib.setBackFaceNormalFlip(true);
        tagNum.getAppearance().setPolygonAttributes(polyAttrib);

        // Tag position
        Vector3f pos = new Vector3f(centroidx / scaling, centroidy / scaling, centroidz / scaling);
        
        // Tag and text transform groups
        TransformGroup tagShapeTG = new TransformGroup(); 
        TransformGroup tagTextTG = new TransformGroup();
        tagTextTG.setCapability(TransformGroup.ALLOW_TRANSFORM_WRITE);
        // TransformGroup that holds both text and tag shape objects
        // and apply a translation and a quaternion defined rotation
        TransformGroup tagTG = new TransformGroup();
        tagTG.setCapability(TransformGroup.ALLOW_CHILDREN_WRITE);

        // Rotate the tag shape to a convinient (default) position
        Transform3D tagShape3d = new Transform3D();
        Transform3D temp3d = new Transform3D();
        tagShape3d.rotZ(-90 * Math.PI/180);
        temp3d.rotX(90 * Math.PI/180);
        tagShape3d.mul(temp3d); // multiple rotations by multiplication of
                         // transforms (quaternion or rot matrix makes
                         // this cleaner?)
//        tagShape3d.setTranslation(pos);
        tagShapeTG.setTransform(tagShape3d);
        tagShapeTG.addChild(tagBox);

        // Because of different default centroids between the tag
        // shape and the text we need to move the text to the lower
        // left corner of the tag shape
        Transform3D textShape3d = new Transform3D(tagShape3d);
        textShape3d.setTranslation(new Vector3f(- tagBox.getZdimension() * 2,tagBox.getXdimension(),- tagBox.getYdimension()));
        tagTextTG.setTransform(textShape3d);
        tagTextTG.addChild(tagNum);
       
        // Translate and rotate both transform groups
        Transform3D tag3d = new Transform3D(q, pos, 1);
        
        tagTG.setTransform(tag3d);
        tagTG.addChild(tagTextTG);
        tagTG.addChild(tagShapeTG);;
        
        vinfoBranch.addChild(tagTG);
        vinfoGroup.addChild(vinfoBranch);
    }
}
