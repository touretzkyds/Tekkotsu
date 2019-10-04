package org.tekkotsu.sketch;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Graphics;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import java.awt.Rectangle;
import java.awt.geom.*;
import java.awt.FontMetrics;
import java.awt.BasicStroke;
import java.util.*;
import javax.media.j3d.*;
import com.sun.j3d.utils.geometry.Cone;
import com.sun.j3d.utils.geometry.Box;
import com.sun.j3d.utils.geometry.Sphere;
import com.sun.j3d.utils.geometry.Cylinder;
import javax.vecmath.*;


public class GraphicsShapeInfo extends ShapeInfo {

    public enum GraphicsElementType {
	lineGType, 
	    polygonGType,
	    ellipseGType,
	    textGType,
	    locParticleGType,
            axisAngleGType,
	    pointGType,
            boundingGType
	    }
    
    public static class GraphicsElement extends ShapeInfo {

	GraphicsElementType type;
	GraphicsShapeInfo owner;
	
	public GraphicsElement(GraphicsElementType _type, String _name, Color _color, GraphicsShapeInfo _owner ) {
	    super(_owner.gui, -1, _owner.parentId, _name, _color, 0, 0, 0, false, false);
	    color = _color;	   
	    type = _type;
	    owner = _owner; 

	}

	public GraphicsElementType getType() {return type; }
	public float getLeft() {return 0;}
	public float getRight() {return 0;}
	public float getTop() {return 0;}
	public float getBottom() {return 0;}

	public void renderTo(Graphics2D graphics, float scaling) {}
        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {}

	public Color getElementColor() {
	    if (owner.inverted)
		return new Color(255-color.getRed(),
				 255-color.getGreen(),
				 255-color.getBlue());
	    else
		return color; 
	}
    }

    public static class LineElement extends GraphicsElement {
	static Icon icon = new ImageIcon(icon_path+"line.png");
   	static Icon inv_icon = new ImageIcon(icon_path+"lineinv.png");
	float p1x, p1y, p2x, p2y;
	public LineElement(String _name, float _p1x, float _p1y, float _p2x, float _p2y,
			   Color _color, GraphicsShapeInfo _owner) {
	    super(GraphicsElementType.lineGType, _name, _color, _owner);
	    p1x = _p1x;
	    p1y = _p1y;
	    p2x = _p2x;
	    p2y = _p2y;

	}

	public String toString() {
	    String _line = name + " endPoints= (" + p1x + " " + p1y + ")---(" + p2x + " " + p2y + ")";	
	    return _line;
	}

   	public Icon getIcon() { 
	    if (inverted)
		return inv_icon;
	    return icon; 
	}

        public void renderTo(Graphics2D graphics, float scaling) {
	    graphics.setColor(getElementColor());
	    graphics.setStroke(new BasicStroke(1.0f/scaling));
	    graphics.draw(new Line2D.Float(p1x,p1y,p2x,p2y));
	}
        
        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            BranchGroup vinfoBranch = new BranchGroup();
            vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
            float distance = (float)Math.sqrt(Math.pow(p2x - p1x, 2) + Math.pow(p2y - p1y, 2));
            Box lineBox = new Box(distance / (scaling * 2), 0.005f, 0.008f, ap);
            float rot = (float)Math.atan2((p2y - p1y), (p2x - p1x));
            float midPointx = (p2x + p1x) / 2;
            float midPointy = (p2y + p1y) / 2;
            TransformGroup lineTG = new TransformGroup();
            Transform3D lineT3d = new Transform3D();
            lineT3d.rotZ(rot);
            lineT3d.setTranslation(new Vector3f(midPointx / scaling, midPointy / scaling, 0.008f));
            lineTG.setTransform(lineT3d);
            lineTG.addChild(lineBox);
            vinfoBranch.addChild(lineTG);
            vinfoGroup.addChild(vinfoBranch);
        }
        
	public float getLeft() { return java.lang.Math.min(p1x,p2x); }
	public float getRight() { return java.lang.Math.max(p1x,p2x); }
	public float getTop() { return java.lang.Math.min(p1y,p2y); }
	public float getBottom() { return java.lang.Math.max(p1y,p2y); }
    }
    public static class PolygonElement extends GraphicsElement {
	static Icon icon = new ImageIcon(icon_path+"poly.png");
	static Icon inv_icon = new ImageIcon(icon_path+"polyinv.png");
	int numvtx;
	float[][] vertices;
	boolean closed;

	public PolygonElement(String _name, int _numvtx, float[][] _vertices, boolean _closed, Color _color, GraphicsShapeInfo _owner) {
	    super(GraphicsElementType.polygonGType, _name, _color, _owner);
	    numvtx = _numvtx;
	    vertices = _vertices;
	    closed = _closed;
	}

        public String toString() {
	    String vtx_coords = name + " vertices = (" + vertices[0][0] + "," + vertices[0][1] + ")";
	    for (int i = 1; i < numvtx; i++)
		vtx_coords += "->(" + vertices[i][0] + "," + vertices[i][1] + ")";
	    return vtx_coords;
        }    

   	public Icon getIcon() { 
	    if (inverted)
		return inv_icon;
	    return icon; 
	}

	// returns left-most coordinate of object
	public float getLeft() { 
	    float min_x = vertices[0][0];
	    for (int i = 1; i < numvtx; i++)
		if (min_x > vertices[i][0])
		    min_x = vertices[i][0];
	    return min_x; 
	}
	// returns right-most coordinate of object
	public float getRight() {
	    float max_x = vertices[0][0];
	    for (int i = 1; i < numvtx; i++)
		if (max_x < vertices[i][0])
		    max_x = vertices[i][0];
	    return max_x; 
	}
	// returns top-most coordinate of object
	public float getTop() { 
	    float min_y = vertices[0][1];
	    for (int i = 1; i < numvtx; i++)
		if (min_y > vertices[i][1])
		    min_y = vertices[i][1];
	    return min_y;     
	}
	// returns bottom-most coordinate of object
	public float getBottom() { 
	    float max_y = vertices[0][1];
	    for (int i = 1; i < numvtx; i++)
		if (max_y < vertices[i][1])
		    max_y = vertices[i][1];
	    return max_y; 
	}
        
	public void renderTo(Graphics2D graphics, float scaling) {
	    graphics.setColor(getElementColor());
	    graphics.setStroke(new BasicStroke(1.0f/scaling));
	    for (int i = 0; i < numvtx-1; i++)
		graphics.drawLine((int)vertices[i][0],(int)vertices[i][1],(int)vertices[i+1][0],(int)vertices[i+1][1]);
	    if (closed) {
		graphics.drawLine((int)vertices[numvtx-1][0],(int)vertices[numvtx-1][1],(int)vertices[0][0],(int)vertices[0][1]);
	    }
	}
        
        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            // Need to clean this up, it is also very inefficient but it works for now.
            BranchGroup vinfoBranch = new BranchGroup();
            vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
            for (int i = 0; i < numvtx - 1; i++)
            {
                float e1x = vertices[i][0];
                float e2x = vertices[i + 1][0];
                float e1y = vertices[i][1];
                float e2y = vertices[i + 1][1];
                float distance = (float)Math.sqrt(Math.pow(e2x - e1x, 2) + Math.pow(e2y - e1y, 2));
                Box lineBox = new Box(distance/2000, 0.005f, 0.008f, ap);
                float rot = (float)Math.atan2((e2y - e1y), (e2x - e1x));
                float midPointx = (e2x + e1x) / 2;
                float midPointy = (e2y + e1y) / 2;
                TransformGroup lineTG = new TransformGroup();
                Transform3D lineT3d = new Transform3D();
                lineT3d.rotZ(rot);
                lineT3d.setTranslation(new Vector3f(midPointx / scaling, midPointy / scaling, 0.008f));
                lineTG.setTransform(lineT3d);
                lineTG.addChild(lineBox);
                vinfoBranch.addChild(lineTG);
            }
            if (closed) {
                float e1x = vertices[0][0];
                float e2x = vertices[numvtx - 1][0];
                float e1y = vertices[0][1];
                float e2y = vertices[numvtx - 1][1];
                float distance = (float)Math.sqrt(Math.pow(e2x - e1x, 2) + Math.pow(e2y - e1y, 2));
                Box lineBox = new Box(distance/2000, 0.005f, 0.008f, ap);
                float rot = (float)Math.atan2((e2y - e1y), (e2x - e1x));
                float midPointx = (e2x + e1x) / 2;
                float midPointy = (e2y + e1y) / 2;
                TransformGroup lineTG = new TransformGroup();
                Transform3D lineT3d = new Transform3D();
                lineT3d.rotZ(rot);
                lineT3d.setTranslation(new Vector3f(midPointx / scaling, midPointy / scaling, 0.008f));
                lineTG.setTransform(lineT3d);
                lineTG.addChild(lineBox);
                vinfoBranch.addChild(lineTG);
            }
            vinfoGroup.addChild(vinfoBranch);
        }
    }
    // Work on going
    public static class BoundingBoxElement extends GraphicsElement {
        static Icon icon = new ImageIcon(icon_path+"poly.png");
        static Icon inv_icon = new ImageIcon(icon_path+"polyinv.png");
        Vector components = new Vector();
        Quat4f q;
        float cx, cy, cz;
        float w, h, l;
        boolean closed;
        
        public BoundingBoxElement(String _name, float [] _q, float _cx, float _cy, float _cz, float _w, float _h, float _l, Color _color, GraphicsShapeInfo _owner) {
            super(GraphicsElementType.boundingGType, _name, _color, _owner);
            q = new Quat4f(_q[1], _q[2], _q[3], _q[0]);
            cx = _cx;
            cy = _cy;
            cz = _cz;
            w = _w;
            h = _h;
            l = _l;
        }

        public BoundingBoxElement(String _name, Vector _components, Color _color, GraphicsShapeInfo _owner) {
            super(GraphicsElementType.boundingGType, _name, _color, _owner);
            components = _components;
        }
        
        public String toString() {
            if (components.size() <= 0) {
                String centroid = name + " centroid(X = " + cx + ", Y = " + cy +
                    ", Z = " + cz + ")\n";
                String quaternion = name + " quaternion(orientation)(w = " + q.getW() + ", x = " + q.getX() + ", y = " + q.getY() + ", z = " + q.getZ() + ")\n";
                String dimensions = " dimensions(width = " + w + ", height = " + h + ", length = " + l + ")\n";
                String boundingBoxString = centroid + quaternion + dimensions;
                return boundingBoxString;
            }
            else {
                String vertex = "Vertices are: ";
                return vertex;
            }
        }    

        public Icon getIcon() { 
            if (inverted)
        	return inv_icon;
            return icon; 
        }

        // returns left-most coordinate of object
        public float getLeft() {
            return 1;//cx - l;
        }
        // returns right-most coordinate of object
        public float getRight() {
            return 1;//cx + l;
        }
        // returns top-most coordinate of object
        public float getTop() {
            return 1;//cy + w;
        }
        // returns bottom-most coordinate of object
        public float getBottom() {
            return 1;//cy - w;
        }
        
        public void renderTo(Graphics2D graphics, float scaling) {
        }
        
        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            BranchGroup vinfoBranch = new BranchGroup();
            vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
            TransformGroup bbTG = new TransformGroup();

            if (components.size() > 0) {
                //vertex are arranged in this way:
                // top upper right
                // top lower right
                // top lower left
                // top upper left
                // bottom upper right
                // bottom lower right
                // bottom lower left
                // bottom upper left
                Point3f tur = new Point3f();
                Point3f tlr = new Point3f();
                Point3f tll = new Point3f();
                Point3f tul = new Point3f();
                Point3f bur = new Point3f();
                Point3f blr = new Point3f();
                Point3f bll = new Point3f();
                Point3f bul = new Point3f();
                QuadArray boxArray;

                for (int i = 0; i < components.size(); i++) {
                    float [][] vertices = (float[][])components.get(i);
                    boxArray = new QuadArray(24, QuadArray.COORDINATES);
                    
                    for (int j = 0; j < 8; j++) {
                        tur = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                        tlr = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                        tll = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                        tul = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                        bur = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                        blr = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                        bll = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                        bul = new Point3f(vertices[j][0], vertices[j][1], vertices[j][2]);
                    }

                    //top
                    boxArray.setCoordinate(0, tur);
                    boxArray.setCoordinate(1, tlr);
                    boxArray.setCoordinate(2, tll);
                    boxArray.setCoordinate(3, tul);

                    //side a
                    boxArray.setCoordinate(4, tur);
                    boxArray.setCoordinate(5, bur);
                    boxArray.setCoordinate(6, blr);                    
                    boxArray.setCoordinate(7, tlr);

                    //side b
                    boxArray.setCoordinate(8, tlr);
                    boxArray.setCoordinate(9, blr);
                    boxArray.setCoordinate(10, bll);
                    boxArray.setCoordinate(11, tll);

                    //side c
                    boxArray.setCoordinate(12, tul);
                    boxArray.setCoordinate(13, tll);
                    boxArray.setCoordinate(14, bll);
                    boxArray.setCoordinate(15, bul);

                    //side d
                    boxArray.setCoordinate(16, tur);
                    boxArray.setCoordinate(17, tul);
                    boxArray.setCoordinate(18, tll);
                    boxArray.setCoordinate(19, tlr);

                    //bottom
                    boxArray.setCoordinate(20, bur);
                    boxArray.setCoordinate(21, blr);
                    boxArray.setCoordinate(22, bll);
                    boxArray.setCoordinate(23, bul);
                    bbTG.addChild(new Shape3D(boxArray, ap));
                }
            }
            else {
                Appearance app = new Appearance();
                app.setMaterial(ap.getMaterial());
                app.setTransparencyAttributes(new TransparencyAttributes(TransparencyAttributes.FASTEST, 0.7f));
//            app.setPolygonAttributes(new PolygonAttributes(PolygonAttributes.POLYGON_LINE, PolygonAttributes.CULL_BACK, 0));
                Box boundingBox = new Box(l/scaling, w/scaling, h/scaling, app);

                Transform3D bbT3d = new Transform3D(q, new Vector3f(cx/scaling, cy/scaling, cz/scaling), 1);
                bbTG.setTransform(bbT3d);
                bbTG.addChild(boundingBox);
            }
            vinfoBranch.addChild(bbTG);
            vinfoGroup.addChild(vinfoBranch);
        }
    }
	
    public static class PointElement extends GraphicsElement {
    	static Icon icon = new ImageIcon(icon_path+"point.png");
    	static Icon inv_icon = new ImageIcon(icon_path+"pointinv.png");
	float x, y;
	
	public PointElement(String _name, float _x, float _y, Color _color, 
			    GraphicsShapeInfo _owner) {
	    super(GraphicsElementType.pointGType, _name, _color, _owner);
	    x = _x;
	    y = _y;
	}        
	
	public String toString() {
	    String _point = name +" center=(" + x + ", " + y + ")" ;
	    return _point;
	}

   	public Icon getIcon() { 
	    if (inverted)
		return inv_icon;
	    return icon; 
	}

	public float getTop() {
	    return x;
	}

	public float getBottom() {
	    return x;
	}

	public float getLeft() {
	    return y;
	}
    
	public float getRight() {
	    return y;
	}

        public void renderTo(Graphics2D graphics, float scaling) {
	    graphics.setColor(getElementColor());
	    graphics.setStroke(new BasicStroke(1.0f/scaling));
	    graphics.draw(new Ellipse2D.Float(x, y, 10, 10));
	}

        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            BranchGroup vinfoBranch = new BranchGroup();
            vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
            final float radius = 0.015f;
            Sphere point = new Sphere(radius, ap);
            TransformGroup pointTG = new TransformGroup();
            Transform3D pointT3d = new Transform3D();
            pointT3d.setTranslation(new Vector3f(centroidx/scaling, centroidy/scaling, centroidz/scaling + 0));
            pointTG.setTransform(pointT3d);
            pointTG.addChild(point);
            vinfoBranch.addChild(pointTG);
            vinfoGroup.addChild(vinfoBranch);
        }
    } 
    public static class EllipseElement extends GraphicsElement {
    	static Icon icon = new ImageIcon(icon_path+"ellipse.png");
    	static Icon inv_icon = new ImageIcon(icon_path+"ellipseinv.png");
	float x, y;
	float semimajor, semiminor;
	float orientation;
	boolean filled;
	
	public EllipseElement(String _name, float _x, float _y, float _semimajor, 
			      float _semiminor, float _orientation, boolean _filled,
			      Color _color, GraphicsShapeInfo _owner) {
	    super(GraphicsElementType.ellipseGType, _name, _color, _owner);
	    x = _x;
	    y = _y;
	    semimajor = _semimajor;
	    semiminor = _semiminor;
	    orientation = _orientation;
	    filled = _filled;

	}
	public String toString() {
	    String _ellipse = name + " center=(" + x + ", " + y + ")"
		+ " smaj=" + semimajor 
		+ ", smin=" + semiminor 
		+ ", orient=" + orientation;
	    return _ellipse;
	}

   	public Icon getIcon() { 
	    if (inverted)
		return inv_icon;
	    return icon; 
	}

        public void renderTo(Graphics2D graphics, float scaling) {
            Ellipse2D ellipse = new Ellipse2D.Float(x - semimajor, y - semiminor,
                                                    semimajor * 2,
                                                    semiminor * 2);
	    graphics.setColor(getElementColor());
	    graphics.setStroke(new BasicStroke(1.0f/scaling));
	    graphics.transform(AffineTransform.getRotateInstance(orientation, x, y));
	    graphics.draw(ellipse);
	    graphics.transform(AffineTransform.getRotateInstance(-orientation, x, y));
	    if (filled == true)
		graphics.fill(ellipse);
	}

        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            // This is an ellipse and not an ellipsoid so we have no 3D rotation xyz
            BranchGroup vinfoBranch = new BranchGroup();
            vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);

            Sphere ellipse = new Sphere(1f, ap);
            Matrix3f m = new Matrix3f(semimajor/scaling, 0.0f, 0.0f, 0.0f, semiminor/scaling, 0.0f, 0.0f, 0.0f, 0.009f);
        
            TransformGroup ellipseTG = new TransformGroup();
            Transform3D ellipseT3d = new Transform3D(m, new Vector3f(x/scaling, y/scaling, 0), 1.0f);
            ellipseTG.setTransform(ellipseT3d);
            ellipseTG.addChild(ellipse);
            vinfoBranch.addChild(ellipseTG);
            vinfoGroup.addChild(vinfoBranch);
        }
	
	public float getTop() {
	    double t = Math.atan(semiminor/Math.tan(orientation)/semimajor)
		+ Math.PI;
	    double r = y + semiminor*Math.sin(t)*Math.cos(orientation)
		+ semimajor*Math.cos(t)*Math.sin(orientation);
	    return (float)r;
	}

	public float getBottom() {
	    double t = Math.atan(semiminor/Math.tan(orientation)/semimajor);
	    double r = y + semiminor*Math.sin(t)*Math.cos(orientation)
		+ semimajor*Math.cos(t)*Math.sin(orientation);
	    return (float)r;
	}

	public float getLeft() {
	    double t = Math.atan((-semiminor*Math.tan(orientation)) / semimajor) + Math.PI;
	    double r = x + semimajor*Math.cos(t)*Math.cos(orientation)
		- semiminor*Math.sin(t)*Math.sin(orientation);
	    return (float)r;
	}
    
	public float getRight() {
	    double t = Math.atan(( -semiminor*Math.tan(orientation)) / semimajor);
	    double r = x + semimajor*Math.cos(t)*Math.cos(orientation)
		- semiminor*Math.sin(t)*Math.sin(orientation);
	    return (float)r;
	}
    }
    
    public static class TextElement extends GraphicsElement {
	float stx, sty;
	String msg;
	public TextElement(String _name, float _stx, float _sty, String _msg, Color _color, GraphicsShapeInfo _owner) {
	    super (GraphicsElementType.textGType, _name, _color, _owner);
	    stx = _stx;
	    sty = _sty;
	    msg = _msg;
	}
        
        public void renderTo(Graphics2D graphics, float scaling) {
	    graphics.setColor(getElementColor());
	    graphics.setStroke(new BasicStroke(1.0f/scaling));
	    FontMetrics fmet=graphics.getFontMetrics();
	    Font tmpFont = graphics.getFont();
	    Font newFont = new Font(tmpFont.getFontName(), tmpFont.getStyle(), 40);
	    graphics.setFont(newFont);
	    if (owner.gui.space == SketchGUI.Space.cam) 
		graphics.drawString(msg,sty,stx+fmet.getAscent());
	    else {
		graphics.transform(AffineTransform.getRotateInstance((float)Math.PI/2, 0, 0));
		AffineTransform flipTrans = new AffineTransform();
		flipTrans.scale (-1.0, 1);
		graphics.transform (flipTrans);
		graphics.drawString(msg,-sty,-stx-fmet.getAscent());
		graphics.transform (flipTrans);
		graphics.transform(AffineTransform.getRotateInstance(-(float)Math.PI/2, 0, 0));
	    }
	    graphics.setFont(tmpFont);
	}

        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            // look for a way of resizing this
            // Font tagFont = new Font("SansSerif", Font.BOLD, 12);
            // Font(String name, int style, int size)
            // Font3D tagFont = new Font3D(new Font("SansSerif", Font.BOLD, 32), new FontExtrusion());
            // Text3D tagNum = new Text3D(tagFont, "" + 2);
            //Shape3D textShape = new Shape3D(tagNum, ap);
        }
    }
    
    public static class LocalizationParticleElement extends GraphicsElement {

	float  x, y, orientation, weight;
	static Icon icon = new ImageIcon(icon_path+"locpart.png");
   	static Icon inv_icon = new ImageIcon(icon_path+"locpartinv.png");

	public LocalizationParticleElement(String _name, float _x, float _y, float _orientation, float _weight, Color _color, GraphicsShapeInfo _owner) {
	    super (GraphicsElementType.locParticleGType, _name, _color, _owner);
	    x = _x;
	    y = _y;
	    orientation = _orientation;
	    weight = _weight;
	}
	
	public String toString() {
	    String _particle = name + " wt=" + weight + " center=(" + x + ", " + y + ") th=" + orientation;
	    return _particle; 
	}

	public Icon getIcon() { 
	    if (inverted)
		return inv_icon;
	    else
		return icon; 
	}

	// returns left-most coordinate of object
        public float getLeft() { return x; }
	// returns right-most coordinate of object
        public float getRight() { return x; }
	// returns top-most coordinate of object
	public float getTop() { return y; }
	// returns bottom-most coordinate of object
        public float getBottom() { return y; }

        public void renderTo(Graphics2D graphics, float scaling) {
	    Rectangle bounds = graphics.getClipBounds();
	    int radius;
	    if (bounds != null)
		radius= java.lang.Math.max(4,java.lang.Math.max(bounds.width,bounds.height)/100);  // was max(4,...)
	    else
		radius = 4;  // was 4
	    graphics.setColor(getElementColor());
	    graphics.setStroke(new BasicStroke(6.0f));  // was 1.0f
	    graphics.fillOval((int)x-radius/2, (int)y-radius/2, radius, radius);
	    float dx = (float)java.lang.Math.cos(orientation)*radius*2; // was *3
	    float dy = (float)java.lang.Math.sin(orientation)*radius*2; // was *3
	    graphics.drawLine((int)x,(int)y,(int)(x+dx),(int)(y+dy));
	}
        
        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            BranchGroup vinfoBranch = new BranchGroup();
            vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
            Cone particle = new Cone(0.01f, 0.03f);
            particle.setAppearance(ap);
            TransformGroup particleTG = new TransformGroup();
            Transform3D particleT3d = new Transform3D();
            particleT3d.rotZ(orientation - ((90 * Math.PI)/180));
            particleT3d.setTranslation(new Vector3f(x/scaling, y/scaling, 0.01f));
            particleTG.setTransform(particleT3d);
            particleTG.addChild(particle);
            vinfoBranch.addChild(particleTG);
            vinfoGroup.addChild(vinfoBranch);
        }
    } 

    public static class AxisAngleElement extends GraphicsElement {
        float cx, cy, cz;
        Quat4f q;
        public AxisAngleElement(String _name, float _q[], float _cx, float _cy, float _cz, GraphicsShapeInfo _owner) {
            super (GraphicsElementType.axisAngleGType, _name, new Color(0,0,255), _owner);

            cx = _cx;
            cy = _cy;
            cz = _cz;
            
            q = new Quat4f(_q[1], _q[2], _q[3], _q[0]); //(x, y, z, w)... Why couldn't java developers just make this constructor conventionally!?
        }
        
        public TransformGroup createArrow(Color color) {
            final float radius = 0.005f;
            final float size = 0.06f;
            TransformGroup tgg = new TransformGroup();
            TransformGroup tg = new TransformGroup();
            Appearance ap = new Appearance();
            Material mat = new Material();
            mat.setAmbientColor(new Color3f(color.darker()));
            mat.setDiffuseColor(new Color3f(color.brighter()));
            mat.setSpecularColor(new Color3f(color.brighter().brighter()));
            ap.setMaterial(mat);

            //shapes
            Cylinder acyl = new Cylinder(radius, size, ap);
            Transform3D cylt3d = new Transform3D();
            cylt3d.rotX((90 * Math.PI)/180);
            cylt3d.setTranslation(new Vector3f(0, 0, size/2));
            tg = new TransformGroup(cylt3d);
            tg.addChild(acyl);
            tgg.addChild(tg);

            Cone acone = new Cone(1.5f * radius, radius * 3, ap);
            Transform3D conet3d = new Transform3D();
            conet3d.rotX((90 * Math.PI)/180);
            conet3d.setTranslation(new Vector3f(0, 0, size + radius/2));
            tg = new TransformGroup(conet3d);
            tg.addChild(acone);
            tgg.addChild(tg);

            return tgg;
        }

        public TransformGroup createAxisAngle() {
            TransformGroup axisTrans = new TransformGroup();
            Transform3D a3d = new Transform3D();

            // draw each axis
            TransformGroup zAxis = createArrow(Color.blue);
            TransformGroup yAxis = createArrow(Color.green);
            Transform3D y3d = new Transform3D();
            y3d.rotX(-(90 * Math.PI)/180);
            yAxis.setTransform(y3d);
            TransformGroup xAxis = createArrow(Color.red);
            Transform3D x3d = new Transform3D();
            x3d.rotY((90 * Math.PI)/180);
            xAxis.setTransform(x3d);

            // add them to a single transform group
            axisTrans.addChild(zAxis);
            axisTrans.addChild(yAxis);
            axisTrans.addChild(xAxis);
            return axisTrans;
        }
        public float getLeft() { return cx; }
	// returns right-most coordinate of object
        public float getRight() { return cx; }
	// returns top-most coordinate of object
	public float getTop() { return cy; }
	// returns bottom-most coordinate of object
        public float getBottom() { return cy; }

	public void renderTo(Graphics2D graphics, float scaling) {
	    
	}

        public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
            BranchGroup vinfoBranch = new BranchGroup();
            vinfoBranch.setCapability(BranchGroup.ALLOW_DETACH);
            TransformGroup axisAngleTG = createAxisAngle();
            Transform3D axisT3d = new Transform3D(q, new Vector3f(cx/scaling, cy/scaling, cz/scaling), 1);
            axisAngleTG.setTransform(axisT3d);
            vinfoBranch.addChild(axisAngleTG);
            vinfoGroup.addChild(vinfoBranch);
        }
    }
        
	

    // definition of GraphicsShapeInfo

    static Icon icon = new ImageIcon(icon_path+"graphics.png");
    static Icon inv_icon = new ImageIcon(icon_path+"graphicsinv.png");
    GraphicsElement elements[];

    public GraphicsShapeInfo(SketchGUI _gui, int _id, int _parentId,
			     String _name, Color _color, 
			     float _centroidx, float _centroidy, float _centroidz, boolean _obstacle, boolean _landmark,
			     GraphicsElement _elements[]) {
	super(_gui, _id, _parentId, _name, _color, _centroidx,_centroidy, _centroidz, _obstacle, _landmark);
	elements = _elements;	
    }	

    public Icon getIcon() { 
	if (inverted)
	    return inv_icon;
	return icon; 
    }

    public void renderTo(Graphics2D graphics, float scaling) {
	// GraphicsShapes don't render; their individual elements do.
	//for (GraphicsElement elt : elements)
	//  elt.renderTo(graphics,scaling);
    }

    public void renderTo3D(BranchGroup vinfoGroup, Appearance ap, float scaling) {
	// GraphicsShapes don't render; their individual elements do.
	/*
        Color color = new Color(0, 0, 0);
        Color3f ambientColor = new Color3f(Color.GRAY);
        Color3f emissiveColor = new Color3f(Color.BLACK);
        Appearance app = new Appearance();
        for (GraphicsElement elt : elements) {
            //need to write special case AxisAngle to make it more efficient
            if ( ! elt.getElementColor().equals(color) ) {
                app = new Appearance();
                color = elt.getElementColor();
                app.setMaterial(new Material(ambientColor, emissiveColor, 
					     new Color3f(color), new Color3f(color.brighter()), 5f));
                elt.renderTo3D(vinfoGroup, app, scaling);
            }
            else { //no need for new appearance
                elt.renderTo3D(vinfoGroup, app, scaling);
            }
        }
	*/
    }
    
    public float getLeft() {
      	float leftMost = elements.length == 0 ? 0 : elements[0].getLeft();
	for (GraphicsElement elt : elements)
	    leftMost = java.lang.Math.min(leftMost, elt.getLeft());
	return leftMost;
    }
    
    public float getRight() {
	float rightMost = elements.length == 0 ? 0 : elements[0].getRight();
	for (GraphicsElement elt : elements) 
	    rightMost = java.lang.Math.max(rightMost, elt.getRight());
	return rightMost;
    }
    
    public float getTop() {
	float topMost = elements.length == 0 ? 0 : elements[0].getTop();
	for (GraphicsElement elt : elements)
	    topMost = java.lang.Math.min(topMost, elt.getTop());
	return topMost;
    }

    public float getBottom() {
	float bottomMost = elements.length == 0 ? 0 : elements[0].getBottom();
	for (GraphicsElement elt : elements) 
	    bottomMost = java.lang.Math.max(bottomMost, elt.getBottom());
	return bottomMost;
    }
}
