/** @file HelpBox.java
 *  @brief Frame for a help window
 *
 *	@author Eric Durback
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

public class HelpBox extends JFrame implements ComponentListener, FocusListener 
{
	

	static Preferences prefs = Preferences.userNodeForPackage(EasyTrain.class);

	JTextArea jtextArea;
	JScrollPane HelpScroll;


  	public static void main(String args[]) 
  	{}
 
 	public void focusGained(FocusEvent e) {} 
 
 	public void focusLost(FocusEvent e) {}

	public static void usageAndExit() 
	{
		System.out.println("usage: java ImageShow (-isRGB|-isYUV) raw_image [raw images]");
		System.exit(1);		
	}

  	public HelpBox ()
  	{
     
	    setBackground(Color.black);
	    setSize(450,600);
	    setTitle("EasyTrain Help");
	    setLocation(prefs.getInt("HelpBox.location.x",50),prefs.getInt("HelpBox.location.y",50));
	    
	    jtextArea = new JTextArea();
	    char buf[] = new char[8192];
		int n;
	    
	    try{
	    
		    BufferedReader in = new BufferedReader(new FileReader("help.txt"));
			while ((n = in.read(buf, 0, buf.length)) > 0)
			{
			    jtextArea.append(new String(buf, 0, n));
			}
			in.close();
		} catch(Exception e){}
	    
	    HelpScroll = new JScrollPane(jtextArea,
		                    JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
		                    JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
	    
	    
	  	this.getContentPane().add(HelpScroll);
	 
		jtextArea.setEditable(false);
	  	
	    addComponentListener(this);
	    addFocusListener(this);  
    
	}

	public void componentResized(ComponentEvent e) {    }
	public void componentHidden(ComponentEvent e) { }
  
	public void componentMoved(ComponentEvent e)
  	{ 
		prefs.putInt("HelpBox.location.x",getLocation().x);
		prefs.putInt("HelpBox.location.y",getLocation().y);
  
  	}
  	public void componentShown(ComponentEvent e) { }

}
