import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.util.*;
import java.awt.geom.*;
import java.io.*;
import java.util.prefs.Preferences;

public class SpectrumViewer extends ImageViewer {
	final static int renderXRes=(1<<8)*2;
	final static int renderYRes=(1<<8)*2;
	
	BufferedImage fullPlot; //the color spectrum plot of all images
	BufferedImage curPlot; //the current color spectrum plot shown (may be fullplot, or a separate plot from selected regions)
	Graphics2D curPlotGraphics;
	BufferedImage[] compPlots; //an array of plots, one per file (used for quick updates of selected areas) -- drawn together, form curPlot
	boolean inverted; //normally black background, inverted uses a white background	
	final static int[] transTemplate=new int[renderXRes*renderYRes]; //!< an array of zeros for quickly blanking out an image
	//final static DataBufferInt transTemplate=new DataBufferInt(renderXRes*renderYRes);
	
	public SpectrumViewer(String title)
	{
		super(title,20);
		init();
	}
	public SpectrumViewer(String title,int inset)
	{
		super(title,inset);
		init();
	}
	
	protected void init() {
		fullPlot=new BufferedImage(renderXRes,renderYRes,BufferedImage.TYPE_INT_ARGB);
		curPlot=new BufferedImage(renderXRes,renderYRes,BufferedImage.TYPE_INT_ARGB);
		curPlotGraphics=(Graphics2D)curPlot.getGraphics();
		compPlots=new BufferedImage[0];
		//transTemplate=new int[renderXRes*renderYRes];
		setImage(fullPlot);
	}
	
	public void showFullPlot() {
		setImage(fullPlot);
	}
	
	public void showCurPlot() {
		paintCurPlot(); //refresh the image, in case compPlots are all null and fullPlot changed
		setImage(curPlot);
	}
	
	//plots a number of images into the fullplot, which is retained so it doesn't have to be recalcuated when switching modes
	public void plotImages(float[][] spec, BufferedImage[] rgbimgs) 
	{
		fullPlot.getRaster().setDataElements(0,0,renderXRes,renderYRes,transTemplate);
		Graphics2D graphics=fullPlot.createGraphics();
		//graphics.setBackground(getBackground());
		graphics.setColor(new Color(0,0,0,0));
		graphics.fillRect(0,0,renderXRes,renderYRes);
		
		Rectangle2D.Float r=new Rectangle2D.Float();
		int pointSize=disp.getHintPointSize();
		for (int i=0; i<rgbimgs.length; i++) {
			int[] rgb=rgbimgs[i].getRGB(0,0,rgbimgs[i].getWidth(),rgbimgs[i].getHeight(),null,0,rgbimgs[i].getWidth());
			for(int p=0; p<rgb.length; p++) {
				graphics.setColor(new Color(rgb[p]));
				float x=spec[i][3*p]*(renderXRes-pointSize); //res-pointSize so maximal point is visible
				float y=spec[i][3*p+1]*(renderYRes-pointSize);
				r.setRect(Math.round(x),Math.round(y),pointSize,pointSize);
				graphics.fill(r);
			}
		}
		if(compPlots.length!=rgbimgs.length)
			compPlots=new BufferedImage[rgbimgs.length];
		setImage(fullPlot);
	}
	
	public void plotImageAreas(float[][] spec, BufferedImage[] rgbimg, Area[] areas) {
		if(compPlots.length!=areas.length) {
			compPlots=new BufferedImage[areas.length];
			//compPlots elements will be initialized on an as-needed basis
		}
		for(int i=0; i<areas.length; i++) {
			paintImageArea(spec[i],rgbimg[i],areas[i],i);
		}
		paintCurPlot();
		setImage(curPlot);
	}
	
	public void updateImageArea(float[] spec, BufferedImage rgbimg, Area a, int i) {
		paintImageArea(spec,rgbimg,a,i);
		paintCurPlot();
		repaint();
	}
	
	//does computation for compPlot, but doesn't update curPlot
	protected void paintImageArea(float[] spec, BufferedImage rgbimg, Area a, int i) {
		if(a==null || a.isEmpty()) {
			compPlots[i]=null;
			return;
		}
		if(compPlots[i]==null)
			compPlots[i]=new BufferedImage(renderXRes,renderYRes,BufferedImage.TYPE_INT_ARGB);
		compPlots[i].getRaster().setDataElements(0,0,renderXRes,renderYRes,transTemplate);
		Graphics2D graphics=compPlots[i].createGraphics();
		
		Rectangle2D.Float r=new Rectangle2D.Float();
		int pointSize=disp.getHintPointSize();
		int[] rgb=rgbimg.getRGB(0,0,rgbimg.getWidth(),rgbimg.getHeight(),null,0,rgbimg.getWidth());
		int p=0;
		for(int y=0; y<rgbimg.getHeight(); y++) {
			double yc=y/(double)(rgbimg.getHeight()-1)*.999+.0005;
			for(int x=0; x<rgbimg.getWidth(); x++) {
				double xc=x/(double)(rgbimg.getWidth()-1)*.999+.0005;
				if(a.contains(xc,yc)) {
					graphics.setColor(new Color(rgb[p/3]));
					float px=spec[p]*(renderXRes-pointSize); //res-pointSize so maximal point is visible
					float py=spec[p+1]*(renderYRes-pointSize);
					r.setRect(Math.round(px),Math.round(py),pointSize,pointSize);
					graphics.fill(r);
				}
				p+=3;
			}
		}
	}
	
	protected void paintCurPlot() {
		curPlot.getRaster().setDataElements(0,0,renderXRes,renderYRes,transTemplate);
		boolean hasNonNull=false;
		for(int i=0; i<compPlots.length; i++) {
			if(compPlots[i]!=null) {
				curPlotGraphics.drawImage(compPlots[i],null,null);
				hasNonNull=true;
			}
		}
		if(!hasNonNull)
			curPlotGraphics.drawImage(fullPlot,null,null);
	}
}
