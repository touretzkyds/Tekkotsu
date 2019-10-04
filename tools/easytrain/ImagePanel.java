import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.util.*;
import java.awt.geom.*;
import java.io.*;
import java.util.prefs.Preferences;


public class ImagePanel extends JComponent
{
	BufferedImage image;
	Point2D hint;
	int hintStroke;
	int pointSize;

	public ImagePanel() {
		image=new BufferedImage(1,1,BufferedImage.TYPE_INT_RGB);
		hint=null;
		hintStroke=1;
		pointSize=1;
	}
	
	public void setImage(BufferedImage img) { image=img; repaint(); }
	public BufferedImage getImage() { return image; }
	
	public void setupGraphics(Graphics2D g2d) {
		g2d.setRenderingHint(RenderingHints.KEY_ALPHA_INTERPOLATION,RenderingHints.VALUE_ALPHA_INTERPOLATION_QUALITY);
		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING ,RenderingHints.VALUE_ANTIALIAS_ON );
		g2d.setRenderingHint(RenderingHints.KEY_COLOR_RENDERING ,RenderingHints.VALUE_COLOR_RENDER_QUALITY );
		g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION ,RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR );
		g2d.setRenderingHint(RenderingHints.KEY_RENDERING ,RenderingHints.VALUE_RENDER_QUALITY );
	}
	
	public void paintComponent(Graphics g) {
		Graphics2D g2d=(Graphics2D)g;
		setupGraphics(g2d);

		AffineTransform trans=AffineTransform.getScaleInstance(getWidth()/(double)image.getWidth(),getHeight()/(double)image.getHeight());
		g2d.setColor(getParent().getBackground());
		g2d.fill(g2d.getClip());
		g2d.drawImage(image, trans, null);
		if(hint!=null) {
			g2d.setStroke(new BasicStroke(hintStroke));
			g2d.setColor(invert(getParent().getBackground()));
			Rectangle r=getHintRect();
			//g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING ,RenderingHints.VALUE_ANTIALIAS_OFF );
			//g2d.drawRect(r.x,r.y,r.width,r.height);
			//g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING ,RenderingHints.VALUE_ANTIALIAS_ON );
			if((hintStroke/2)*2==hintStroke) {
				//keep drawing aligned with pixel boundaries (no half pixel drawing)
				Rectangle2D.Float r2=new Rectangle2D.Float(r.x-.5f,r.y-.5f,r.width,r.height);
				g2d.draw(r2);
			} else {
				g2d.drawRect(r.x,r.y,r.width,r.height);
			}
		}
	}
	
	public void setHintStroke(int hintStroke) {
		this.hintStroke=hintStroke;
	}
	public int getHintStroke() {
		return hintStroke;
	}
	public void setHintPointSize(int pointSize) {
		this.pointSize=pointSize;
	}
	public int getHintPointSize() {
		return pointSize;
	}
	
	public void setHint(Point2D hint) {
		if(this.hint!=null)
			repaint(getShapeBounds(getHintRect()));
		if(hint==null) {
			this.hint=null;
			return;
		}
		this.hint=(Point2D)hint.clone();
		repaint(getShapeBounds(getHintRect()));
	}
	
	Rectangle getHintRect() {
		int px=Math.round((float)hint.getX()*(image.getWidth()-pointSize)); //same formula used for plotting
		int py=Math.round((float)hint.getY()*(image.getHeight()-pointSize)); // (res-pointSize so maximal point is visible)
		float xs=getWidth()/(float)image.getWidth();
		float ys=getHeight()/(float)image.getHeight();
		int ul=hintStroke/2;
		int lr=hintStroke-ul;
		int x=Math.round(px*xs)-lr;
		int y=Math.round(py*ys)-lr;
		int width=Math.round((px+pointSize)*xs)-x+ul;
		int height=Math.round((py+pointSize)*ys)-y+ul;
		if(width<hintStroke+1) width=hintStroke+1;
		if(height<hintStroke+1) height=hintStroke+1;
		return new Rectangle(x,y,width,height);
	}
	
	Rectangle getShapeBounds(Shape s) {
		//AffineTransform trans=AffineTransform.getScaleInstance(getWidth(),getHeight());
		//Rectangle2D scrnBnds=trans.createTransformedShape(s).getBounds2D();
		Rectangle2D scrnBnds=s.getBounds();
		int ul=hintStroke/2;
		return new Rectangle((int)(scrnBnds.getX()-ul)-1, (int)(scrnBnds.getY()-ul)-1,(int)(scrnBnds.getWidth()+hintStroke+3),(int)(scrnBnds.getHeight()+hintStroke+3));
	}
	
	static protected Color invert(Color c) {
		//can't invert gray -- if it's close to gray, just use white/black
		int rd=128-c.getRed();
		int gd=128-c.getGreen();
		int bd=128-c.getBlue();
		if(rd*rd+gd*gd+bd*bd<10*10) //within 10 units of gray, snap to black or white
			return ((rd+gd+bd)/3<0) ? Color.BLACK : Color.WHITE;
		return new Color(255-c.getRed(),255-c.getGreen(),255-c.getBlue(),c.getAlpha());
	}

	/*	Rectangle getHintRect() {
		int px=(int)(hint.getX()*image.getWidth());
		int py=(int)(hint.getY()*image.getHeight());
		float xs=getWidth()/(float)image.getWidth();
		float ys=getHeight()/(float)image.getHeight();
		int x=Math.round(px*xs-1);
		int y=Math.round(py*ys-1);
		int width=Math.round((px+1)*xs-x);
		int height=Math.round((py+1)*ys-y);
		if(width<2) width=2;
		if(height<2) height=2;
		return new Rectangle(x,y,width,height);
	}*/
	
/*	Rectangle getShapeBounds(Shape s) {
		//AffineTransform trans=AffineTransform.getScaleInstance(getWidth(),getHeight());
		//Rectangle2D scrnBnds=trans.createTransformedShape(s).getBounds2D();
		Rectangle2D scrnBnds=s.getBounds();
		return new Rectangle((int)(scrnBnds.getX()), (int)(scrnBnds.getY()),(int)(scrnBnds.getWidth()+1),(int)(scrnBnds.getHeight()+1));
	}
	*/
	/*public void Paint(Graphics g)
	{
		/*System.out.println("jsjsjsjs");
		
		
		Dimension sz=getSize();
		Point spot = p.spot;
		
		if (p._image!=null)
		  g.drawImage(p._image, 0, 0, sz.width , sz.height , null);

		g.setColor(Color.white);
		if (p.curArea!=null)
		  ((Graphics2D)g).draw(p.curArea);

		if (p.curPoly!=null)
		  ((Graphics2D)g).draw(p.curPoly);
		
		if(spot != null)
		  g.drawRect(spot.x-2, spot.y-2, 5, 5);
		  
		content.faint(content.getGraphics());		
		
	
	}*/
	
	/*public void faint(Graphics g )
	{
		
		
		//Graphics g = getGraphics();
		
		System.out.println("jfffjsjsjs" + g);
		
		Dimension sz=getSize();
		Point spot = p.spot;
		
		if (p._image!=null)
		  g.drawImage(p._image, 0, 0, sz.width , sz.height , null);

		g.setColor(Color.white);
		if (p.curArea!=null)
		  ((Graphics2D)g).draw(p.curArea);

		if (p.curPoly!=null)
		  ((Graphics2D)g).draw(p.curPoly);
		
		if(spot != null)
		  g.drawRect(spot.x-2, spot.y-2, 5, 5);	
	
	}
	
	*/
	

}
