/** @file Thumbnail.java
 *  @brief 
 *
 *  Button Component that contains smaller image of an image loaded by the user.
 *
 *	@author editted by: Eric Durback
 *  @bug No known bugs.
 */

import java.awt.image.*;
import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import java.awt.geom.*;
import java.util.prefs.Preferences;

public class Thumbnail extends JToggleButton
{
	BufferedImage _image;
	String[] imglist;
	
	ImageData imageData;
	
	int curimg;
	int numImages;
	boolean isRGB;
	
	
	
	public Thumbnail (ImageData imageData, int imageNum) 
	{
	  	
		this.imageData = imageData;
		curimg = imageNum;
		
		_image=imageData.getRGBImage(imageNum);
		double scale = (double)_image.getHeight()/(double)_image.getWidth();
		ImageIcon i = new ImageIcon(_image.getScaledInstance(72 ,(int)(72*scale), Image.SCALE_SMOOTH));
		setIcon(i);

		repaint();	
	}

}
