/** @file SegmentedImage.java
 *  @brief Frame for displaying the segmented image        
 *
 *	@author editted by: Eric Durback
 *  @bug No known bugs.
 */

import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.util.*;
import java.awt.geom.*;
import java.io.*;
import java.util.prefs.Preferences;


public class SegmentedImage extends ImageViewer
{
	float[] yuvimg;
	int width,height;
	ThresholdMap thresh;
	Point hint;
  
	public SegmentedImage(String title, ThresholdMap thresh) { super(title); init(thresh); }
	protected void init(ThresholdMap thresh) {
		this.thresh=thresh;
		setBackground(Color.GRAY);
		setTitle("Segmented Viewer");
	}

	public void setImage(float[] yuvimg, int width, int height) {
		this.yuvimg=yuvimg;
		this.width=width;
		this.height=height;
		setImage(thresh.segment(yuvimg,width,height));
	}
	
	public ThresholdMap getThresh() { return thresh; }
	public void updateMap(Area area, short index) {
		thresh.updateMap(area,index);
		setImage(thresh.segment(yuvimg,width,height));
	}
	public void updateMap(Vector areas) {
		for(short i=0; i<areas.size(); i++)
			thresh.updateMap((Area)areas.get(i),i);
		setImage(thresh.segment(yuvimg,width,height));
	}
	
	/*	public void reSegment(byte[] _tmap, int[][]_averageColors)
	{

		tmap = _tmap;
		averageColors = _averageColors;

		int[] data=imageData.getPixels();

		IndexColorModel cmodel=makeColorModel(averageColors);

		if (tmap==null || cmodel==null) return;

		
		image = new BufferedImage(imageData.image_width, imageData.image_height,
		    BufferedImage.TYPE_BYTE_INDEXED,cmodel);

		segmentImage(data, tmap, imageData.image_width, imageData.image_height);

		repaint();
	}

	public void showHint(Point p)
	{

		if(p!=null) hint = scalePoint(p);
		else hint = null;
		
		repaint();
	}


	public void segmentImage(int[] data, byte[] tmap, int width, int height) 
	{
		
		touched = true;
		int size_y=16, size_u=64, size_v=64;
		byte[] imgdata=new byte[data.length];

		for (int i=0; i<data.length; i++) 
		{
		    int y=(data[i]>>16)&0xff;
		    int u=(data[i]>>8)&0xff;
		    int v=data[i]&0xff;
		    y=y>>4;
		    u=u>>2;
		    v=v>>2;
		    imgdata[i]=tmap[(y*size_u+u)*size_v+v];
		}

		image.getRaster().setDataElements(0,0,width,height,imgdata);
		repaint();
	}

	public IndexColorModel makeColorModel(int[][] averageColors) 
	{
		if (averageColors == null)
		    return null;
		
		byte[] byte_cmap=new byte[(averageColors.length+1)*3];


		int i;
		for (i=1; i<averageColors.length+1; i++) 
		{
		    //byte_cmap[i*3]=(byte) ((cmap[i]>>16) & 0xff);
		    //byte_cmap[i*3+1]=(byte) ((cmap[i]>>8) & 0xff);
		    //byte_cmap[i*3+2]=(byte) (cmap[i] & 0xff);
		    byte_cmap[i*3] = (byte)(averageColors[i-1][0]&0xff);
		    byte_cmap[i*3+1] = (byte)(averageColors[i-1][1]&0xff);
		    byte_cmap[i*3+2] = (byte)(averageColors[i-1][2]&0xff);
		}

		byte_cmap[0] = (byte)128;
		byte_cmap[1] = (byte)128;
		byte_cmap[2] = (byte)128;

		IndexColorModel cmodel=new IndexColorModel(7, averageColors.length + 1, byte_cmap,
						   0, false); 
		return cmodel;
	}

	public int getPixel(int x, int y) 
	{
		Dimension sz=getSize();
		x=(x * image.getWidth()) / sz.width;
		y=(y * image.getHeight()) / sz.height;
		return image.getRGB(x, y);
	}
	
	public double getScale()
	{
		return (double)imagePanel.getWidth()/(double)width;
	}
	
	public Point scalePoint(Point p)
	{
		return new Point( (int)((double)p.x * getScale()),(int)((double)p.y * getScale())); 
	
	}
	
	public void setSizes()
	{
		height = imagePanel.getHeight();
		width = imagePanel.getWidth();		
	}
	
	public Dimension getModifiedSize(int x, int y)
	{
		Insets i = this.getInsets();
		
		//System.out.println(i);
		
		return new Dimension(x+(i.left+i.right), y+(i.top+i.bottom));
	}

	public void keyPressed(KeyEvent e) 
	{
		//imageShow.keyImageChange(e);
		
	}
	public void keyReleased(KeyEvent e) { }
	public void keyTyped(KeyEvent e) { }

	
	
	public void componentResized(ComponentEvent e) 
	{
			
			double changeX = (double)imagePanel.getWidth() / width;
			double changeY = (double)imagePanel.getHeight() / height;
		
			if(getWidth()<= 120 || getHeight() <= 98)
		  	{
		  		this.setSize(getModifiedSize(120,98));	  		
		  	}
		  	
		  	else if(   (changeX > changeY && imagePanel.getWidth() > width  ) 
		  	   || (changeX < changeY && imagePanel.getWidth() < width  )  )
		  	{
		  		changeY = changeX;
		  		this.setSize(getModifiedSize(imagePanel.getWidth(),(int)(height*changeX)));
		  	}
		  	else
		  	{
		  		changeX = changeY;
		  		this.setSize(getModifiedSize((int)(width*changeY),imagePanel.getHeight()));
		  	}
		  	
		  	//imageShow.sizeFixed = false;
	
	}
	public void componentHidden(ComponentEvent e) { }
	
	public void componentMoved(ComponentEvent e)
	{ 
	
		prefs.putInt("SegmentedImage.location.x",getLocation().x);
		prefs.putInt("SegmentedImage.location.y",getLocation().y);

	}
	public void componentShown(ComponentEvent e) { }
*/
}
