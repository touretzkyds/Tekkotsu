package org.tekkotsu.sketch;

import org.tekkotsu.mon.*;

import java.awt.event.*;
import javax.swing.*;
import javax.swing.tree.*;
import javax.swing.event.*;
import javax.swing.plaf.TreeUI;
import java.lang.String;
import java.awt.*;
import javax.imageio.ImageIO;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.image.IndexColorModel;
import java.util.*;
import java.util.Date;
import java.util.Vector;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.Comparator;
import java.util.Hashtable;
import java.io.PrintWriter;
import java.io.FileOutputStream;
import java.util.prefs.Preferences;
import java.io.File;
import java.net.*;
import java.io.*;
import java.util.StringTokenizer;
import java.awt.geom.*;
import javax.imageio.metadata.IIOMetadataNode;
import org.w3c.dom.NodeList;
import org.w3c.dom.Node;
import java.lang.Object;


public class SketchGUI extends JFrame implements ActionListener,
                                                 TreeSelectionListener,
                                                 VisionUpdatedListener,
                                                 MouseListener,
						 TreeWillExpandListener
{
    public enum Space { unknown, cam, local, world }
    Space space;          // 1 == cam, 2 == local, 3 == world

    SketchPanel sketchPanel;
    SketchPanel3D sketchPanel3D;
    boolean is3d = false;
    JButton rescaleBut, refreshListBut, colorsBut, threedBut;
    JCheckBox checkAllBut;
    boolean defaultSelected = true;
    JCheckBox invertAllBut;
    boolean defaultInversion = false;
    JCheckBox autoRefreshBut;
    boolean autoRefreshEnabled = true;
    JTree sketchTree = null;
    JTree colorTree = null;
    JScrollPane colorPane = null;
    JScrollPane sketchPane = null;
    TreePath[] prevColors = null;
    HashMap<String,String> colorNames = null;
    HashMap<String, DefaultMutableTreeNode> colorNodes = null;

    DefaultMutableTreeNode root, colorRoot;
    HashMap<DefaultMutableTreeNode, ArrayList<DefaultMutableTreeNode>> colorGroups;
    AffineTransform Tmat, TmatInv;
    JLabel status;
    float mspf=0;
    float mspfGamma = 0.9f;
    String state="";
    String host;
    int listingPort;
    int sketchPort;
    static Preferences prefs = Preferences.userNodeForPackage(SketchGUI.class);

    // the socket over which listings are retrieved and Sketch commands are sent
    Socket listingSocket=null;
    PrintWriter netout = null; // network output
    BufferedReader netin = null; // network input

    VisionListener listener;

    PopupMenu popMenu;
    SketchOrShapeInfo currentMenuObject;

    TreePath curPath;
    DefaultMutableTreeNode curNode = null;
    TreeSelectionEvent lastSelectionEvent = null;
    Boolean treeJustCollapsed = false;

    int mouseClickModifiers = 0; // most recent mouse button and control/shift modifier info
    Vector itemStack;
    int sketchCount;  // we won't render the pixel array if there are no sketches, only shapes
    BufferedImage img;
    boolean imageNeedsUpdate;
    boolean focusJustChanged = false;

    int imageWidth = -1, imageHeight = -1;

    Color backgroundColor = new Color(64,64,64);
    int defaultWidth = 640, defaultHeight = 480; // Use webcam defaults if we cannot connect.
    boolean panelBoundsSet = false;

    SketchPanel curSketchPanel;
    String panelTitle;
    int panelCount = 1;

    public static void main(String args[]) {
        if(args.length<2)
            usage();

        Space _space = Space.unknown;
        int _listingPort = 0;
        int _sketchPort = 0;
        if ( args[1].equals("cam") ) {
            _space = Space.cam;
            _listingPort = 5800;
            _sketchPort = 5801;
        }
        else if ( args[1].equals("local") ) {
            _space = Space.local;
            _listingPort = 5802;
            _sketchPort = 5803;
        }
        else if ( args[1].equals("world") ) {
            _space = Space.world;
            _listingPort = 5804;
            _sketchPort = 5805;
        }
        else usage();

        SketchGUI gui = new SketchGUI(args[0], _listingPort, _sketchPort, _space);
        gui.addWindowListener(new WindowAdapter() {
                public void windowClosing(WindowEvent e) {System.exit(0);}});
        gui.setVisible(true);
    }

    public static void usage() {
        System.out.println("Usage: java org/tekkotsu/sketch/SketchGUI hostname [cam/local/world]");
        System.exit(2);
    }

    public void actionPerformed(ActionEvent e) {
	try {
	    if(e.getActionCommand().compareTo("rescale")==0)
		rescaleAction();
	    else if(e.getActionCommand().compareTo("refreshlist")==0)
		refreshlistAction();
            else if(e.getActionCommand().compareTo("3dgui")==0)
                openOrClose3DPanel();
	    else if(e.getActionCommand().compareTo("checkall")==0)
		checkAllAction(e);
	    else if(e.getActionCommand().compareTo("invertall")==0)
		invertAllAction(e);
	    else if(e.getActionCommand().compareTo("autorefresh")==0)
		autoRefreshAction(e);
            else if(e.getActionCommand().compareTo("showcolors")==0)
                showColorAction(e);
	}
	catch(IOException ioe) {
	    System.err.println("Transfer error");
	    reconnect();
	    actionPerformed(e);
	}
    }

    public void autoRefreshAction(ActionEvent e) {
	autoRefreshEnabled = ((JCheckBox)(e.getSource())).isSelected();
    }

    public void checkAllAction(ActionEvent e) {
	System.out.println("checkAllAction");
        defaultSelected = ((JCheckBox)(e.getSource())).isSelected();
	selectOrDeselectAll();
    }

    void selectOrDeselectAll() {
        // select/delselect all the shapes
        DefaultMutableTreeNode node = root;
        while ( (node = node.getNextNode()) != null ) {
            if (node.getUserObject() instanceof ShapeInfo) {
                ShapeInfo obj = (ShapeInfo)node.getUserObject();
                TreePath path = new TreePath(node.getPath());
                if (defaultSelected) {
                    // if not already selected, make it so
		    if (!sketchTree.isPathSelected(path)) {
			sketchTree.addSelectionPath(path);
		    }
                } else {
                    if (sketchTree.isPathSelected(path)) {
                        sketchTree.removeSelectionPath(path);
                    }
                }
            }
        }

        // select all the colors if we're selecting all shapes
        node = colorRoot;
	if (defaultSelected)
	    while ( (node = node.getNextNode()) != null )
		colorTree.addSelectionPath(new TreePath(node.getPath()));
	prevColors = colorTree.getSelectionPaths();
	updateDisplay();
	lastSelectionEvent = null;
    }

    public void invertAllAction(ActionEvent e) {

        defaultInversion = ((JCheckBox)(e.getSource())).isSelected();

        DefaultMutableTreeNode n = root;
        while((n = n.getNextNode())!= null) {
            if (n.getUserObject() instanceof SketchOrShapeInfo) {
                SketchOrShapeInfo obj = (SketchOrShapeInfo)n.getUserObject();
                // Reset all shapes to follow the default when turning on invert all
                if (defaultInversion && obj instanceof ShapeInfo) {
                    ((ShapeInfo)obj).setUseDefaultInversion(true);
                }
                if (obj instanceof ShapeInfo && ((ShapeInfo)obj).getUseDefaultInversion()) {
                    obj.setInverted(defaultInversion);
                }
            }
        }
	updateDisplay();
	sketchTree.treeDidChange(); // update icons
    }

    // 3D GUI
    public void openOrClose3DPanel() {
        if (!is3d) {
            sketchPanel3D = new SketchPanel3D();
            is3d = true;
            refreshListBut.doClick();
        } else {
            sketchPanel3D.close();
            is3d = false;
        }
    }

    //color button was pressed; pops out or hides the color pane
    public void showColorAction(ActionEvent e) {
        if (colorPane.isVisible()) {
            colorPane.setVisible(false);
	    //necessary to refresh the frame so the pane appears/disappears
            this.setSize(this.getWidth()*2/3, this.getHeight());
        } else {
            Rectangle sketchBounds = sketchPane.getBounds();
            colorPane.setMinimumSize(new Dimension(this.getWidth()/3, 0));
            colorPane.setVisible(true);
            this.setSize(this.getWidth()*3/2, this.getHeight());
        }
	sketchTree.updateUI();
	colorTree.updateUI();
    }

    public void rescaleAction() {
        curSketchPanel.leftBound = 0;
        curSketchPanel.rightBound = imageWidth;
        curSketchPanel.topBound = 0;
        curSketchPanel.bottomBound = imageHeight;
        TreePath[] paths = sketchTree.getSelectionPaths();
        if(paths!=null) {
            for (int path_i = 0; path_i < paths.length; path_i++) {
                DefaultMutableTreeNode node
                    = (DefaultMutableTreeNode)(paths[path_i].getLastPathComponent());

                if (node == root) continue;
                if(!(node.getUserObject() instanceof SketchOrShapeInfo)) {
                    System.out.println("rescaleAction:: placeholder text can't be selected");
                    continue;
                }

                SketchOrShapeInfo vinfo = (SketchOrShapeInfo)(node.getUserObject());
		curSketchPanel.scaleToSketchOrShape(vinfo);
                if (vinfo instanceof SketchInfo)
                    ((SketchInfo)vinfo).unloadImage();
            }
        }
        updateDisplay();
    }

    // refreshColors() is called in ColorSlider.java
    public void refreshColors() {
	DefaultMutableTreeNode n = root;
	if (n!= null) {
	    while((n = n.getNextNode())!= null) {
		if (n.getUserObject() instanceof SketchInfo) {
		    SketchInfo vinfo = (SketchInfo)n.getUserObject();
		    ((TCPVisionListener)listener).setReadingImage();
		    netout.println("get "+vinfo.id);
		    try {
			String inputLine;
			while((inputLine=readLine()).compareTo("get end") != 0) {
			    System.out.println(inputLine);
			}
		    } catch (IOException ioe) {
			System.err.println("Transfer error");
			reconnect();
			refreshColors();
		    }
		    while(((TCPVisionListener)listener).isReadingImage())
			// thread.yield() in java?
			// *** The following sleep is needed to prevent some kind of thread lockup.  -- DST 1/30/2009
			try { Thread.sleep(1); } catch (InterruptedException ie) {}
		    System.out.println("done with id:"+vinfo.id);
		    ((SketchInfo)vinfo).copyImage(((TCPVisionListener)listener).getImage());
		    ColorConverter.hasChanged = true;
	
		    //((SketchInfo)n.getUserObject()).unloadImage();
		} else if (n.getUserObject() instanceof SketchOrShapeInfo) {
		    continue; // ((SketchOrShapeInfo)n.getUserObject()).setVisible(false);
		}
	    }
	}
	valueChanged(null);
    }

    public void readColors() {
	if ( netout == null) return;
	colorNames = new HashMap<String,String>();
        colorNodes = new HashMap<String, DefaultMutableTreeNode>();
        colorGroups = new HashMap<DefaultMutableTreeNode, ArrayList<DefaultMutableTreeNode>>();
	String inputLine = "";
	try{
	    netout.println("colors");
	    System.out.println(inputLine = readLine());
	    while((inputLine=readLine()).compareTo("colors end") != 0) {
		System.out.println(inputLine);
		StringTokenizer colortoken = new StringTokenizer(inputLine,",");
		Color color = new Color(Integer.parseInt(colortoken.nextToken()),
					Integer.parseInt(colortoken.nextToken()),
					Integer.parseInt(colortoken.nextToken()));
		String colorName = colortoken.nextToken();
		colorNames.put(color.toString(), colorName);
		DefaultMutableTreeNode colorNode = new DefaultMutableTreeNode(colorName);
		colorNodes.put(color.toString(), colorNode);
		colorGroups.put(colorNode, new ArrayList<DefaultMutableTreeNode>());
		colorRoot.add(colorNode);
	    }
	    System.out.println(inputLine);

	}
	catch(IOException ioe) {
	    System.err.println("Transfer error in readColors():  '"+inputLine+"'");
	    reconnect();
	}
    }

    public void refreshlistAction() throws IOException {
        Hashtable oldSelected = new Hashtable();
        Hashtable oldInverted = new Hashtable();
	Hashtable oldExpanded = new Hashtable();
        Hashtable oldColorSelected = new Hashtable();
	saveCurrentSettings(oldSelected,oldInverted,oldExpanded,oldColorSelected);

        // make sure we have a working connection; try reconnect once, then punt
        if (netout==null) reconnect();
        if (netout==null) return;

        if ( !panelBoundsSet )	// Get sketch bounds from the robot if we don't already have them
	    getPanelBounds();

        colorRoot.removeAllChildren();	// Colors may have changed, so always reload them
	readColors();
	reselectColors(oldColorSelected);   // Re-select any color nodes that were selected before

        root.removeAllChildren();
        itemStack = new Vector();
        sketchCount = 0;
	readSketchAndShapeList(oldInverted,oldColorSelected);
	pairChildrenWithParents();
	((DefaultTreeModel)sketchTree.getModel()).reload(); // clears selection info
	reselectSketchesAndShapes(oldSelected,oldExpanded);
	updateDisplay();

        if (is3d) {
            sketchPanel3D.viewIsSet = false;
            sketchPanel3D.rescale();
        }

	lastSelectionEvent = null;
    }

    void saveCurrentSettings(Hashtable oldSelected, Hashtable oldInverted, Hashtable oldExpanded, Hashtable oldColorSelected) {
	// Save existing selection and inversion settings for any objects that might remain after the reload
        DefaultMutableTreeNode node = root;
        if (node!= null) {
            while((node = node.getNextNode())!= null) {
                if (node.getUserObject() instanceof SketchOrShapeInfo) {
                    SketchOrShapeInfo oinfo = (SketchOrShapeInfo)node.getUserObject();
		    TreePath path = new TreePath(node.getPath());
                    oldSelected.put(new Integer(oinfo.getId()),new Boolean(sketchTree.isPathSelected(path)));
		    oldExpanded.put(new Integer(oinfo.getId()),new Boolean(sketchTree.isExpanded(path)));
		    if (oinfo instanceof ShapeInfo) {
			ShapeInfo sinfo = (ShapeInfo)oinfo;
			if (!sinfo.getUseDefaultInversion())
			    oldInverted.put(new Integer(sinfo.getId()),new Boolean(sinfo.inverted));
		    } else if (oinfo instanceof SketchInfo) {
			SketchInfo skinfo = (SketchInfo)oinfo;
			oldInverted.put(new Integer(skinfo.getId()),new Boolean(skinfo.inverted));
			skinfo.unloadImage();
		    }
		}
	    }
        }

        // Save current color selections
        node = colorRoot;
	while ((node = node.getNextNode())!= null)
	    oldColorSelected.put(node.toString(), new Boolean(false));
        if (colorTree.getSelectionPaths() != null)
            for (TreePath tp : colorTree.getSelectionPaths())
                oldColorSelected.put(tp.getLastPathComponent().toString(), new Boolean(true));
    }

    void getPanelBounds() {
        String inputLine = "";
	try {
	    netout.println("size");
	    System.out.println(inputLine = readLine());
	    while((inputLine=readLine()).compareTo("size end") != 0) {
		System.out.println(inputLine);
		StringTokenizer sizetoken = new StringTokenizer(inputLine);
		String token = sizetoken.nextToken();
		if (token.equals("width")){
		    imageWidth = Integer.parseInt(sizetoken.nextToken());
		}
		else if (token.equals("height")){
		    imageHeight = Integer.parseInt(sizetoken.nextToken());
		}
	    }
        } catch (IOException ioe) {
            System.err.println("Error reading sketch bounds.");
	    imageWidth = 225;
	    imageHeight = 225;
        }
	System.out.println(inputLine);
        sketchPanel.leftBound = 0;
        sketchPanel.rightBound = imageWidth;
        sketchPanel.topBound = 0;
        sketchPanel.bottomBound = imageHeight;
        System.out.println("Setting bounds to "+sketchPanel.rightBound + " x "+sketchPanel.bottomBound);
	panelBoundsSet = true;
    }

    void reselectColors(Hashtable oldColorSelected) {
	// Re-select any colors that were selected before the refresh action
	colorTree.clearSelection();
	DefaultMutableTreeNode node = colorRoot;
	while ( (node = node.getNextNode()) != null ) {
		Boolean val = (Boolean)oldColorSelected.get(node.toString());
		if ( val != null && val.equals(true) )
		    colorTree.addSelectionPath(new TreePath(node.getPath()));
	    }
	((DefaultTreeModel)colorTree.getModel()).reload();  // force redisplay
	prevColors = colorTree.getSelectionPaths();
    }

    void readSketchAndShapeList(Hashtable oldInverted, Hashtable oldColorSelected) {
        // Read in the new sketch/shape list
	if ( netout == null ) return;
	String inputLine;
	try {
	    netout.println("list");
	    System.out.println(inputLine = readLine());  // eat the "list begin"
	    while((inputLine=readLine()).compareTo("list end") != 0) {
		// parse type (sketch or shape)
		StringTokenizer st = new StringTokenizer(inputLine,": ");
		String type = st.nextToken();

		if (type.equals("tmat")) {
		    float[] tvals = new float[12];
		    inputLine = readLine();
		    st = new StringTokenizer(inputLine,": ");
		    for (int i=0; i<12; i++)
			tvals[i] = Float.parseFloat(st.nextToken());
		    double scale = Math.sqrt(tvals[0]*tvals[0]+tvals[1]*tvals[1]);
		    System.out.println("tmat:  tx="+tvals[3]+"  ty="+tvals[7]+"  scale="+scale);
		    Tmat.setTransform(tvals[0], tvals[1], tvals[4], tvals[5], tvals[3], tvals[7]);
		    System.out.println("Tmat is " + Tmat.toString());
		    try { TmatInv.setTransform(Tmat.createInverse()); }
		    catch (java.awt.geom.NoninvertibleTransformException nte)
			{ System.out.println("Error occured while trying to make the inverse of Tmat"); }
		    continue;
		}

		// if we get here, it's a sketch or a shape

		// parse id
		inputLine = readLine();
		System.out.println(inputLine);
		st = new StringTokenizer(inputLine,": ");
		st.nextToken();
		int id = Integer.parseInt(st.nextToken());

		//parse parentId
		inputLine = readLine();
		st = new StringTokenizer(inputLine,": ");
		st.nextToken();
		int parentId = Integer.parseInt(st.nextToken());

		// parse name
		inputLine = readLine();
		st = new StringTokenizer(inputLine," \r\n");
		st.nextToken();
		String name = st.nextToken();

		// parse sketch or shape subtype
		inputLine = readLine();
		st = new StringTokenizer(inputLine,": ");
		st.nextToken();
		int subtype = Integer.parseInt(st.nextToken());

		// parse color
		inputLine = readLine();
		st = new StringTokenizer(inputLine,": ");
		st.nextToken();
		Color color = new Color( Integer.parseInt(st.nextToken()),
					 Integer.parseInt(st.nextToken()),
					 Integer.parseInt(st.nextToken()));

		//parse colormap if sketch
		int colormap = SketchInfo.COLORMAP_SEG_TYPE;
		if(type.equals("sketch")) {
		    inputLine = readLine();
		    st = new StringTokenizer(inputLine,": ");
		    st.nextToken();
		    colormap = Integer.parseInt(st.nextToken());
		}

		// create node
		System.out.println(type +
				   " id:"+id+" parentId:"+parentId+" name:"+name +
				   " type:" + subtype);
		String colormsg = "  color: " + color +
		    (type.equals("sketch") ? (" colormap:" + colormap) : "");
		System.out.println(colormsg);
		SketchOrShapeInfo oinfo;

		if(type.equals("sketch")) {
		    ++sketchCount;
		    oinfo =  new SketchInfo(this, id, parentId, name, color, colormap,
					    subtype, imageWidth, imageHeight);
		    if (oldInverted.containsKey(new Integer(id)))
			oinfo.setInverted(((Boolean)oldInverted.get(new Integer(id))).booleanValue());
		} else if (type.equals("shape")) {
		    oinfo = parseShape(id, parentId, name, color, subtype, imageWidth, imageHeight);
		    ShapeInfo sinfo = (ShapeInfo)oinfo;
		    if (oldInverted.containsKey(new Integer(id))) {
			sinfo.setUseDefaultInversion(false);
			sinfo.setInverted(((Boolean)oldInverted.get(new Integer(id))).booleanValue());
		    } else {
			sinfo.setInverted(defaultInversion);
		    }
		} else {
		    System.out.println("Invalid type!");
		    color = new Color(0);
		    oinfo = new SketchOrShapeInfo(this, id, parentId, name, color);
		}

		if (space != Space.cam)
		    sketchPanel.scaleToSketchOrShape(oinfo);
		if (is3d)
		    sketchPanel3D.scaleToSketchOrShape(oinfo);

		DefaultMutableTreeNode newnode = new DefaultMutableTreeNode(oinfo);
		root.add(newnode);
		if ( oinfo instanceof ShapeInfo && ! (oinfo instanceof GraphicsShapeInfo) )
		    addNodeToColorGroup(newnode, color, oldColorSelected);
		//add children to newNode if oinfo is a member of GraphicsShapeInfo
		if (oinfo instanceof GraphicsShapeInfo) {
		    GraphicsShapeInfo ginfo = (GraphicsShapeInfo)oinfo;
		    for (GraphicsShapeInfo.GraphicsElement elt : ((GraphicsShapeInfo)oinfo).elements) {
			DefaultMutableTreeNode elementNode = new DefaultMutableTreeNode(elt);
			newnode.add(elementNode);
			addNodeToColorGroup(elementNode, elt.color, oldColorSelected);
		    }
		}
	    }
	} catch (IOException ioe) {
	    System.err.println("Error reading sketch or shape.");
	    //ioe.printStackTrace();
	    return;
	}

    }

    void pairChildrenWithParents() {
        // pair children with parents
        Vector allnodes = new Vector();
        Enumeration nodeEn = root.preorderEnumeration();
        while(nodeEn.hasMoreElements())
            allnodes.add(nodeEn.nextElement());
        for (int i=0; i<allnodes.size(); i++) {
            DefaultMutableTreeNode tempNode = (DefaultMutableTreeNode)(allnodes.elementAt(i));
	    if (( tempNode == root ) || (tempNode.getUserObject() instanceof GraphicsShapeInfo.GraphicsElement) )
		continue;
            DefaultMutableTreeNode potentialParent = root;
            while((potentialParent = potentialParent.getNextNode()) != null)
                if ((potentialParent.getUserObject() instanceof SketchOrShapeInfo ) &&
		    ! (potentialParent.getUserObject() instanceof GraphicsShapeInfo.GraphicsElement )) {
                    if(((SketchOrShapeInfo)(tempNode.getUserObject())).parentId ==
                       ((SketchOrShapeInfo)(potentialParent.getUserObject())).id &&
                       !potentialParent.isNodeAncestor(tempNode)) {
                        potentialParent.add(tempNode);
                        break;
                    }
                }
        }
        sortTree(root);
    }

    void reselectSketchesAndShapes(Hashtable oldSelected, Hashtable oldExpanded) {
	// Re-select any tree elements that were selected before
        sketchTree.clearSelection();
        DefaultMutableTreeNode node = root;
        if (node!= null) {
            while ((node = node.getNextNode())!= null) {
                if (node.getUserObject() instanceof SketchOrShapeInfo) {
                    TreePath path = new TreePath(node.getPath());
		    SketchOrShapeInfo obj = (SketchOrShapeInfo)node.getUserObject();
		    int id = obj.getId();
		    if (oldExpanded.containsKey(new Integer(id)))
			if (((Boolean)oldExpanded.get(new Integer(id))).booleanValue())
			    sketchTree.expandPath(path);
		    if (oldSelected.containsKey(new Integer(id))) {
			if (((Boolean)oldSelected.get(new Integer(id))).booleanValue()) {
			    sketchTree.addSelectionPath(path);
			    if ( obj instanceof GraphicsShapeInfo )
				selectGraphicsObjects(path,true);
			}
		    } else if (defaultSelected && obj instanceof ShapeInfo) {
			sketchTree.addSelectionPath(path);
			if ( obj instanceof GraphicsShapeInfo )
			    selectGraphicsObjects(path,true);
		    }
                }
            }
        }
    }

    void expandAll() {
	// called by init to expand the initial sketch tree
        DefaultMutableTreeNode node = root;
        if (node!= null)
            while ( (node = node.getNextNode()) != null )
                if (node.getUserObject() instanceof SketchOrShapeInfo)
		    sketchTree.expandPath(new TreePath(node.getPath()));
    }

    void addNodeToColorGroup(DefaultMutableTreeNode newnode, Color color, Hashtable oldColorSelected) {
	// add this color if we haven't seen it before
	if ( !colorNodes.containsKey(color.toString()) ) {
	    String colorName;
	    if (colorNames.containsKey(color.toString()))
		colorName = colorNames.get(color.toString());
	    else
		colorName = String.format("<%d, %d, %d>",color.getRed(),color.getGreen(),color.getBlue());
	    DefaultMutableTreeNode colorNode = new DefaultMutableTreeNode(colorName);
	    colorNodes.put(color.toString(), colorNode);
	    colorGroups.put(colorNode, new ArrayList<DefaultMutableTreeNode>());
	    colorRoot.add(colorNode);
	    if (oldColorSelected.containsKey(colorNode.toString())) {
		if (((Boolean)oldColorSelected.get(colorNode.toString())).booleanValue())
		    colorTree.addSelectionPath(new TreePath(colorNode.getPath()));
	    } else if (defaultSelected) {
		colorTree.addSelectionPath(new TreePath(colorNode.getPath()));
	    }
	}
	// now add the new node to the color group
	colorGroups.get(colorNodes.get(color.toString())).add(newnode);
    }

    public SketchOrShapeInfo parseShape(int id, int parentId, String name,
                                        Color color, int shapetype,
                                        int width, int height)
        throws IOException
    {
        String inputLine;
        StringTokenizer st;
        SketchOrShapeInfo result;

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float cx = Float.parseFloat(st.nextToken());
        float cy = Float.parseFloat(st.nextToken());
        float cz = Float.parseFloat(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        boolean obstacle = Integer.parseInt(st.nextToken()) == 1;
        boolean landmark = Integer.parseInt(st.nextToken()) == 1;

        if(shapetype == 1) { // lineshape
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float e1x = Float.parseFloat(st.nextToken());
            float e1y = Float.parseFloat(st.nextToken());
            float e1z = Float.parseFloat(st.nextToken());

            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float e2x = Float.parseFloat(st.nextToken());
            float e2y = Float.parseFloat(st.nextToken());
            float e2z = Float.parseFloat(st.nextToken());

            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float r = Float.parseFloat(st.nextToken());

            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float theta = Float.parseFloat(st.nextToken());

            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            boolean end1_valid = Integer.parseInt(st.nextToken()) == 1;
            boolean end1_active = Integer.parseInt(st.nextToken()) == 1;
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            boolean end2_valid = Integer.parseInt(st.nextToken()) == 1;
            boolean end2_active = Integer.parseInt(st.nextToken()) == 1;

            System.out.println("  cxyz: " +cx+" "+cy+" "+cz);
            System.out.println("  e1xyz/va: "+e1x+" "+e1y +" "+e1z+" / "+end1_valid+" "+end1_active);
            System.out.println("  e2xyz/va: "+e2x+" "+e2y +" "+e2z+" / "+end2_valid+" "+end2_active);
            System.out.println("  r: " + r + "  theta: "+theta);
            result = new LineShapeInfo(this, id, parentId, name, color,
                                       cx,cy,cz, obstacle, landmark, e1x,e1y,e1z, e2x,e2y,e2z,
                                       r, theta, end1_valid, end1_active, end2_valid, end2_active);

        } else if(shapetype == 2) { // ellipseshape
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float semimajor = Float.parseFloat(st.nextToken());
            float semiminor = Float.parseFloat(st.nextToken());
            System.out.println(" axes:"+semimajor+" "+semiminor);

            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float orientation = Float.parseFloat(st.nextToken());
            System.out.println(" orientation:"+orientation);

            result = new EllipseShapeInfo(this,id,parentId,name,color,
                                          cx,cy,cz, obstacle, landmark, semimajor, semiminor, orientation);

        } else if (shapetype == 3) { // pointshape
            result =  new PointShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark);

        } else if (shapetype == 4) { // agentshape
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float orientation = Float.parseFloat(st.nextToken());
            System.out.println(" orientation:"+orientation);
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float offsetX = Float.parseFloat(st.nextToken());
            System.out.println(" offsetX:"+offsetX);
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float offsetY = Float.parseFloat(st.nextToken());
            System.out.println(" offsetY:"+offsetY);
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float halfDimsX = Float.parseFloat(st.nextToken());
            System.out.println(" halfDimsX:"+halfDimsX);
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float halfDimsY = Float.parseFloat(st.nextToken());
            System.out.println(" halfDimsY:"+halfDimsY);
            result =  new AgentShapeInfo(this, id, parentId, name, color, cx,cy,cz, obstacle, landmark,
                                         orientation, offsetX, offsetY, halfDimsX, halfDimsY);
        } else if (shapetype == 5) { // sphereshape
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            float radius = Float.parseFloat(st.nextToken());
            System.out.println(" radius:"+radius);

            result =  new SphereShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark, radius);

        } else if (shapetype == 6) { // polygonshape
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            int num_vertices = Integer.parseInt(st.nextToken());
            System.out.println("num_vtx: " + num_vertices);
            float[][] vertices = new float[num_vertices][3];

            for (int i = 0; i < num_vertices; i++) {
                inputLine = readLine();
                st = new StringTokenizer(inputLine,": ");
                st.nextToken();
                vertices[i][0] = Float.parseFloat(st.nextToken());
                vertices[i][1] = Float.parseFloat(st.nextToken());
                vertices[i][2] = Float.parseFloat(st.nextToken());
                System.out.println("vertex "+i+": "+vertices[i][0]+" "+vertices[i][1] +" "+vertices[i][2]);
            }
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            boolean end1_valid = Integer.parseInt(st.nextToken()) == 1;
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            boolean end2_valid = Integer.parseInt(st.nextToken()) == 1;
            System.out.println("End1_valid: " + end1_valid);
            System.out.println("End2_valid: " + end2_valid);
            result =  new PolygonShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
                                           num_vertices, vertices, end1_valid, end2_valid);

        } else if(shapetype == 7) { // blobshape
            result = parseBlob(id,parentId,name,color,cx,cy,cz,obstacle, landmark);
        } else if(shapetype == 8) { // brickshape
	    result = parseBrick(id,parentId,name,color,cx,cy,cz,obstacle, landmark);
        } else if(shapetype == 9) { // pyramidshape
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float FLx = Float.parseFloat(st.nextToken());
            float FLy = Float.parseFloat(st.nextToken());
            System.out.println("FL: " + FLx + " " + FLy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float BLx = Float.parseFloat(st.nextToken());
            float BLy = Float.parseFloat(st.nextToken());
            System.out.println("BL: " + BLx + " " + BLy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float FRx = Float.parseFloat(st.nextToken());
            float FRy = Float.parseFloat(st.nextToken());
            System.out.println("FR: " + FRx + " " + FRy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float BRx = Float.parseFloat(st.nextToken());
            float BRy = Float.parseFloat(st.nextToken());
            System.out.println("BR: " + BRx + " " + BRy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float Topx = Float.parseFloat(st.nextToken());
            float Topy = Float.parseFloat(st.nextToken());
            System.out.println("Top: " + Topx + " " + Topy);
            result =  new PyramidShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
                                           FLx, FLy, FRx, FRy, BLx, BLy, BRx, BRy,
                                           Topx, Topy);
        } else if(shapetype == 10) { // localzationparticleshape
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float orient = Float.parseFloat(st.nextToken());
            float weight = Float.parseFloat(st.nextToken());
            System.out.println("  orient: " + orient + "   weight: " + weight);
            result =  new LocalizationParticleShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
                                                        orient, weight);
        } else if (shapetype == 12) { // markershape
            inputLine = readLine();
            String[] values = inputLine.split(":",2);
            result =  new MarkerShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
                                          values[0], values[1]);

        } else if(shapetype == 13) { // cylindershape
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float h = Float.parseFloat(st.nextToken());
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float rad = Float.parseFloat(st.nextToken());
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            System.out.println(inputLine);
            float qw = Float.parseFloat(st.nextToken());
            float qx = Float.parseFloat(st.nextToken());
            float qy = Float.parseFloat(st.nextToken());
            float qz = Float.parseFloat(st.nextToken());
            float q[] = {qw, qx, qy, qz};
            result =  new CylinderShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark, h, rad, q);
        } else if (shapetype == 14) { // SIFT shape
            result =  parseSift(id,parentId,name,color,cx,cy,cz,obstacle, landmark);
        } else if (shapetype == 15) { // AprilTag shape
            result =  parseAprilTag(id,parentId,name,color,cx,cy,cz,obstacle, landmark);
        } else if (shapetype == 16) { // GraphicsElement shape
            inputLine = readLine();
            st = new StringTokenizer(inputLine,": ");
            st.nextToken();
            int numElem = Integer.parseInt(st.nextToken());
            System.out.println(" numElements: " + numElem);
            GraphicsShapeInfo.GraphicsElement elements[] = new GraphicsShapeInfo.GraphicsElement[numElem];
            result = new GraphicsShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark, elements);
            for(int elementnum = 0; elementnum < numElem; elementnum++) {
                inputLine = readLine();
                st = new StringTokenizer(inputLine,": ");
                st.nextToken();
                int eleType = Integer.parseInt(st.nextToken());
		Color ecolor = new Color(Integer.parseInt(st.nextToken()),
					 Integer.parseInt(st.nextToken()),
					 Integer.parseInt(st.nextToken()));
		name = st.nextToken();
                switch (eleType) {
                case 0: { //LineElement or lineGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
		    st.nextToken();
                    float e1x = Float.parseFloat(st.nextToken());
                    float e1y = Float.parseFloat(st.nextToken());
                    float e2x = Float.parseFloat(st.nextToken());
                    float e2y = Float.parseFloat(st.nextToken());
                    elements[elementnum] = new GraphicsShapeInfo.LineElement(name, e1x, e1y, e2x, e2y, ecolor, (GraphicsShapeInfo)result);
		    break; }
                case 1: { //PolygonElement or polygonGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    int numvtx = Integer.parseInt(st.nextToken());
                    float[][] evertices = new float[numvtx][3];
                    for (int j = 0; j < numvtx; j++) {
                        inputLine = readLine();
                        st = new StringTokenizer(inputLine,": ");
                        st.nextToken();
                        evertices[j][0] = Float.parseFloat(st.nextToken());
                        evertices[j][1] = Float.parseFloat(st.nextToken());
                    }
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    boolean closed = Boolean.parseBoolean(st.nextToken());
                    elements[elementnum] = new GraphicsShapeInfo.PolygonElement(name, numvtx, evertices, closed, ecolor, (GraphicsShapeInfo)result);
                    break; }

                case 2: { //EllipseElement or ellipseGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float ecx = Float.parseFloat(st.nextToken());
                    float ecy = Float.parseFloat(st.nextToken());
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float esemimajor = Float.parseFloat(st.nextToken());
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float esemiminor = Float.parseFloat(st.nextToken());
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float eorientation = Float.parseFloat(st.nextToken());
		    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    boolean efilled = Boolean.parseBoolean(st.nextToken());
                    elements[elementnum] = new GraphicsShapeInfo.EllipseElement(name, ecx, ecy, esemimajor, esemiminor, eorientation, efilled, ecolor, (GraphicsShapeInfo)result);
                    break; }

                case 3: { //TextElement or textGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float ex = Float.parseFloat(st.nextToken());
                    float ey = Float.parseFloat(st.nextToken());
                    inputLine = readLine();
                    int index = inputLine.indexOf(":");
                    String emsg = inputLine.substring(index+2);
                    elements[elementnum] = new GraphicsShapeInfo.TextElement(name, ex, ey, emsg, ecolor, (GraphicsShapeInfo)result);
		    break; }

                case 4: { //LocalizationParticleElement or locParticleGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float pex = Float.parseFloat(st.nextToken());
                    float pey = Float.parseFloat(st.nextToken());
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float peorientation = Float.parseFloat(st.nextToken());
		    st.nextToken();
		    float peweight = Float.parseFloat(st.nextToken());
                    elements[elementnum] = new GraphicsShapeInfo.LocalizationParticleElement(name, pex, pey, peorientation, peweight, ecolor, (GraphicsShapeInfo)result);
                    break; }

                case 5: { //AxisAngleElemenet or axisAngleGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float qw = Float.parseFloat(st.nextToken());
                    float qx = Float.parseFloat(st.nextToken());
                    float qy = Float.parseFloat(st.nextToken());
                    float qz = Float.parseFloat(st.nextToken());
                    float q[] = {qw, qx, qy, qz};
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    float cqx = Float.parseFloat(st.nextToken());
                    float cqy = Float.parseFloat(st.nextToken());
                    float cqz = Float.parseFloat(st.nextToken());
                    //System.out.println("Quaternion values: w = " + qw + ", x = " + qx + ", y = " + qy + ", z = " + qz + ", cqx = " + cqx + ", cqy = " + cqy + ", cqz = " + cqz);
                    elements[elementnum] = new GraphicsShapeInfo.AxisAngleElement(name, q, cqx, cqy, cqz, (GraphicsShapeInfo)result);
                    break; }

		case 6: { //PointElement or pointGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
		    float px = Float.parseFloat(st.nextToken());
                    float py = Float.parseFloat(st.nextToken());
		    // System.out.println("X is: " + px + " Y is: " + py );
                    elements[elementnum] = new GraphicsShapeInfo.PointElement(name, px, py, ecolor, (GraphicsShapeInfo)result);
                    break; }

                case 7: { //BoundingBoxElement or boundingGType
                    inputLine = readLine();
                    st = new StringTokenizer(inputLine,": ");
                    st.nextToken();
                    boolean vertexDraw = Boolean.parseBoolean(st.nextToken());
                    if (vertexDraw) {
                        inputLine = readLine();
                        st = new StringTokenizer(inputLine,": ");
                        st.nextToken();
                        int numCmp = Integer.parseInt(st.nextToken());
                        Vector components = new Vector(numCmp);
                        for (int j = 0; j < numCmp; j++) {
                            float [][] vertices = new float[8][3];
                            for (int k = 0; k < 8; k++) {
                                inputLine = readLine();
                                st = new StringTokenizer(inputLine,": ");
                                st.nextToken();
                                vertices[k][0] = Float.parseFloat(st.nextToken());
                                vertices[k][1] = Float.parseFloat(st.nextToken());
                                vertices[k][2] = Float.parseFloat(st.nextToken());
                            }
                            components.add(vertices);
                        }
                        elements[elementnum] =new GraphicsShapeInfo.BoundingBoxElement(name, components, ecolor, (GraphicsShapeInfo)result);
                    }
                    else {
                        inputLine = readLine();
                        st = new StringTokenizer(inputLine,": ");
                        st.nextToken();
                        float ecx = Float.parseFloat(st.nextToken());
                        float ecy = Float.parseFloat(st.nextToken());
                        float ecz = Float.parseFloat(st.nextToken());
                        inputLine = readLine();
                        st = new StringTokenizer(inputLine,": ");
                        st.nextToken();
                        float w = Float.parseFloat(st.nextToken());
                        float h = Float.parseFloat(st.nextToken());
                        float l = Float.parseFloat(st.nextToken());
                        inputLine = readLine();
                        st = new StringTokenizer(inputLine,": ");
                        st.nextToken();
                        float qw = Float.parseFloat(st.nextToken());
                        float qx = Float.parseFloat(st.nextToken());
                        float qy = Float.parseFloat(st.nextToken());
                        float qz = Float.parseFloat(st.nextToken());
                        float q[] = {qw, qx, qy, qz};
                        elements[elementnum] = new GraphicsShapeInfo.BoundingBoxElement(name, q, ecx, ecy, ecz, w, h, l, ecolor, (GraphicsShapeInfo)result);
                    }
                    break; }
                }
            }
        } else if (shapetype == 17) { // Domino shape
            result = parseDomino(id,parentId,name,color,cx,cy,cz,obstacle,landmark);    
	} else if (shapetype == 18) { // Naught shape
            inputLine = readLine();
	    System.out.println(inputLine);
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float h = Float.parseFloat(st.nextToken());
            inputLine = readLine();
	    System.out.println(inputLine);
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float rad = Float.parseFloat(st.nextToken());
            result =  new NaughtShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark, h, rad);
	} else if (shapetype == 19) { // Cross shape
	    inputLine = readLine();
	    System.out.println(inputLine);
	    st = new StringTokenizer(inputLine,": ");
	    st.nextToken();
	    float e1x = Float.parseFloat(st.nextToken());
	    float e1y = Float.parseFloat(st.nextToken());
	    inputLine = readLine();
	    System.out.println(inputLine);
	    st = new StringTokenizer(inputLine,": ");
	    st.nextToken();
	    float e2x = Float.parseFloat(st.nextToken());
	    float e2y = Float.parseFloat(st.nextToken());
	    inputLine = readLine();
	    System.out.println(inputLine);
	    st = new StringTokenizer(inputLine,": ");
	    st.nextToken();
	    float e3x = Float.parseFloat(st.nextToken());
	    float e3y = Float.parseFloat(st.nextToken());
	    inputLine = readLine();
	    System.out.println(inputLine);
	    st = new StringTokenizer(inputLine,": ");
	    st.nextToken();
	    float e4x = Float.parseFloat(st.nextToken());
	    float e4y = Float.parseFloat(st.nextToken());
	    inputLine = readLine();
	    System.out.println(inputLine);
	    st = new StringTokenizer(inputLine,": ");
	    st.nextToken();
	    float armwidth = Float.parseFloat(st.nextToken());
	    result = new CrossShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
					e1x,e1y,e2x,e2y,e3x,e3y,e4x,e4y,armwidth);
        } else {
            System.out.println("Invalid shape!  id="+id);
            result =  new ShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark);
        }
        return result;
    }

    public BrickShapeInfo parseBrick(int id, int parentId, String name, Color color,
                                   float cx, float cy, float cz, boolean obstacle, boolean landmark)
        throws IOException
    {
        String inputLine;
        StringTokenizer st;

            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float GFLx = Float.parseFloat(st.nextToken());
            float GFLy = Float.parseFloat(st.nextToken());
            System.out.println("GFL: " + GFLx + " " + GFLy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float GFRx = Float.parseFloat(st.nextToken());
            float GFRy = Float.parseFloat(st.nextToken());
            System.out.println("GFR: " + GFRx + " " + GFRy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float GBLx = Float.parseFloat(st.nextToken());
            float GBLy = Float.parseFloat(st.nextToken());
            System.out.println("GBL: " + GBLx + " " + GBLy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float GBRx = Float.parseFloat(st.nextToken());
            float GBRy = Float.parseFloat(st.nextToken());
            System.out.println("GBR: " + GBRx + " " + GBRy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float TFLx = Float.parseFloat(st.nextToken());
            float TFLy = Float.parseFloat(st.nextToken());
            System.out.println("TFL: " + TFLx + " " + TFLy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float TFRx = Float.parseFloat(st.nextToken());
            float TFRy = Float.parseFloat(st.nextToken());
            System.out.println("TFR: " + TFRx + " " + TFRy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float TBLx = Float.parseFloat(st.nextToken());
            float TBLy = Float.parseFloat(st.nextToken());
            System.out.println("TBL: " + TBLx + " " + TBLy);
            inputLine = readLine();
            st = new StringTokenizer(inputLine, ": ");
            st.nextToken();
            float TBRx = Float.parseFloat(st.nextToken());
            float TBRy = Float.parseFloat(st.nextToken());
            System.out.println("TBR: " + TBRx + " " + TBRy);

	    return new BrickShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
				      GFLx, GFLy, GFRx, GFRy, GBLx, GBLy, GBRx, GBRy,
				      TFLx, TFLy, TFRx, TFRy, TBLx, TBLy, TBRx, TBRy);
    }

    public DominoShapeInfo parseDomino(int id, int parentId, String name, Color color,
                                   float cx, float cy, float cz, boolean obstacle, boolean landmark)
        throws IOException
    {
	BrickShapeInfo b = parseBrick(id, parentId, name, color, cx, cy, cz, obstacle, landmark);

        String inputLine;
        StringTokenizer st;

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        int lowValue = Integer.parseInt(st.nextToken());
        int highValue = Integer.parseInt(st.nextToken());
	System.out.println("values: " + lowValue + " " + highValue);

	return new DominoShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
				   b.GFLx, b.GFLy, b.GFRx, b.GFRy, b.GBLx, b.GBLy, b.GBRx, b.GBRy,
				   b.TFLx, b.TFLy, b.TFRx, b.TFRy, b.TBLx, b.TBLy, b.TBRx, b.TBRy,
				   lowValue, highValue);
    }

    public BlobShapeInfo parseBlob(int id, int parentId, String name, Color color,
                                   float cx, float cy, float cz, boolean obstacle, boolean landmark)
        throws IOException
    {
        String inputLine;
        StringTokenizer st;

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        String colorname = st.nextToken();
        System.out.println(" colorname: "+colorname);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float area = Float.parseFloat(st.nextToken());
        System.out.println(" area: "+area);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        int orient = Integer.parseInt(st.nextToken());
        System.out.println(" orient: "+orient);
        if ( space == Space.cam )
            orient = 0; // always display cam-space blobs as lieing in the camera plane

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float topLeft_x = Float.parseFloat(st.nextToken());
        float topLeft_y = Float.parseFloat(st.nextToken());
        float topLeft_z = Float.parseFloat(st.nextToken());
        System.out.println(" topLeft: " + topLeft_x + " " +
                           topLeft_y + " " + topLeft_z);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float topRight_x = Float.parseFloat(st.nextToken());
        float topRight_y = Float.parseFloat(st.nextToken());
        float topRight_z = Float.parseFloat(st.nextToken());
        System.out.println(" topRight: " + topRight_x + " " +
                           topRight_y + " " + topRight_z);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float bottomLeft_x = Float.parseFloat(st.nextToken());
        float bottomLeft_y = Float.parseFloat(st.nextToken());
        float bottomLeft_z = Float.parseFloat(st.nextToken());
        System.out.println(" bottomLeft: " + bottomLeft_x + " " +
                           bottomLeft_y + " " + bottomLeft_z);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float bottomRight_x = Float.parseFloat(st.nextToken());
        float bottomRight_y = Float.parseFloat(st.nextToken());
        float bottomRight_z = Float.parseFloat(st.nextToken());
        System.out.println(" bottomRight: " + bottomRight_x + " " +
                           bottomRight_y + " " + bottomRight_z);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
	boolean top_valid = Boolean.parseBoolean(st.nextToken());
        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        boolean bottom_valid = Boolean.parseBoolean(st.nextToken());
        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        boolean left_valid = Boolean.parseBoolean(st.nextToken());
        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
	boolean right_valid = Boolean.parseBoolean(st.nextToken());
        System.out.println(" valid edges: top= " + top_valid + "; bottom= " +
                           bottom_valid + "; left= " + left_valid + "; right= " + right_valid);

        return new BlobShapeInfo(this, id,parentId,name,color,colorname,
                                 cx,cy,cz,obstacle,landmark,area,orient,
                                 topLeft_x, topLeft_y, topLeft_z,
                                 topRight_x, topRight_y, topRight_z,
                                 bottomLeft_x, bottomLeft_y, bottomLeft_z,
                                 bottomRight_x, bottomRight_y, bottomRight_z,
				 top_valid, bottom_valid, left_valid, right_valid);
    }

    public SiftShapeInfo parseSift(int id, int parentId, String name, Color color,
                                   float cx, float cy, float cz, boolean obstacle, boolean landmark)
        throws IOException
    {
        String inputLine;
        StringTokenizer st;

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        String objectName = st.nextToken();
        System.out.println(" objectname: "+objectName);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        int objectID = Integer.parseInt(st.nextToken());
        System.out.println(" objectID: "+objectID);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        String modelName = st.nextToken();
        System.out.println(" modelname: "+modelName);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float topLeft_x = Float.parseFloat(st.nextToken());
        float topLeft_y = Float.parseFloat(st.nextToken());
        float topLeft_z = Float.parseFloat(st.nextToken());
        System.out.println(" topLeft: " + topLeft_x + " " +
                           topLeft_y + " " + topLeft_z);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float topRight_x = Float.parseFloat(st.nextToken());
        float topRight_y = Float.parseFloat(st.nextToken());
        float topRight_z = Float.parseFloat(st.nextToken());
        System.out.println(" topRight: " + topRight_x + " " +
                           topRight_y + " " + topRight_z);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float bottomLeft_x = Float.parseFloat(st.nextToken());
        float bottomLeft_y = Float.parseFloat(st.nextToken());
        float bottomLeft_z = Float.parseFloat(st.nextToken());
        System.out.println(" bottomLeft: " + bottomLeft_x + " " +
                           bottomLeft_y + " " + bottomLeft_z);

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float bottomRight_x = Float.parseFloat(st.nextToken());
        float bottomRight_y = Float.parseFloat(st.nextToken());
        float bottomRight_z = Float.parseFloat(st.nextToken());
        System.out.println(" bottomRight: " + bottomRight_x + " " +
                           bottomRight_y + " " + bottomRight_z);

        return new SiftShapeInfo(this, id,parentId,objectID,name,objectName,modelName,color,
                                 cx,cy,cz,obstacle,landmark,
                                 topLeft_x,topLeft_y,topLeft_z,
                                 topRight_x,topRight_y,topRight_z,
                                 bottomLeft_x,bottomLeft_y,bottomLeft_z,
                                 bottomRight_x,bottomRight_y,bottomRight_z);
    }

    public AprilTagShapeInfo parseAprilTag(int id, int parentId, String name, Color color,
                                           float cx, float cy, float cz, boolean obstacle, boolean landmark)
        throws IOException
    {
        String inputLine;
        StringTokenizer st;

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        int tagID = Integer.parseInt(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float orientation = Float.parseFloat(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float qw = Float.parseFloat(st.nextToken());
        float qx = Float.parseFloat(st.nextToken());
        float qy = Float.parseFloat(st.nextToken());
        float qz = Float.parseFloat(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        int hammingDistance = Integer.parseInt(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float p0x = Float.parseFloat(st.nextToken());
        float p0y = Float.parseFloat(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float p1x = Float.parseFloat(st.nextToken());
        float p1y = Float.parseFloat(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float p2x = Float.parseFloat(st.nextToken());
        float p2y = Float.parseFloat(st.nextToken());

        inputLine = readLine();
        st = new StringTokenizer(inputLine,": ");
        st.nextToken();
        float p3x = Float.parseFloat(st.nextToken());
        float p3y = Float.parseFloat(st.nextToken());

        float p[][] = {{p0x,p0y},{p1x,p1y},{p2x,p2y},{p3x,p3y}};
        float q[] = {qw, qx, qy, qz};

        System.out.println("  tagID: "+tagID+" orientation: "+orientation+" hammingDistance: "+hammingDistance);
        System.out.println(" p0="+p0x+","+p0y+"   p2="+p2x+","+p2y);
        System.out.println(" centroid="+cx+" , "+cy+" , "+cz);
        return new AprilTagShapeInfo(this, id, parentId, name, color, cx, cy, cz, obstacle, landmark,
                                     tagID, orientation, q, hammingDistance, p);
    }

    public SketchGUI(String _host, int _listingPort, int _sketchPort,
                     Space _space) {
        super();
        host = _host;
        listingPort = _listingPort;
        sketchPort = _sketchPort;
        space = _space;

        Tmat = new AffineTransform();
        TmatInv = new AffineTransform();

        if(space == Space.cam)
            root = new DefaultMutableTreeNode("camspace");
        else if (space == Space.local)
            root = new DefaultMutableTreeNode("localspace");
        else if (space == Space.world)
            root = new DefaultMutableTreeNode("worldspace");

        colorRoot = new DefaultMutableTreeNode("Colors");


        listener = new TCPVisionListener(host, sketchPort);

        // network setup
        reconnect();

        // Get the image size immediately after connecting
        if (netout != null) {
            try{
                String inputLine;
                netout.println("size");
                System.out.println(inputLine = readLine());
                while((inputLine=readLine()).compareTo("size end") != 0) {
                    System.out.println(inputLine);
                    StringTokenizer sizetoken = new StringTokenizer(inputLine);
                    String token = sizetoken.nextToken();
                    if (token.equals("width")){
                        imageWidth = Integer.parseInt(sizetoken.nextToken());

                    }
                    else if (token.equals("height")){
                        imageHeight = Integer.parseInt(sizetoken.nextToken());
                    }
                }
                System.out.println(inputLine);

            }
            catch(IOException ioe) {
                System.err.println("Transfer error");
            }

            panelBoundsSet = true;
        }
        else {
            imageWidth = defaultWidth;
            imageHeight = defaultHeight;
        }

        sketchPanel = new SketchPanel(this, listener, space, prefs, imageWidth, imageHeight);

        img = new BufferedImage(imageWidth, imageHeight, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = img.createGraphics();
        g.clearRect(0,0,imageWidth, imageHeight);
        itemStack = new Vector();

        init();

    }

    public void init() {
        int strutsize=10;
        int sepsize=5;
        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(Box.createVerticalStrut(strutsize),BorderLayout.NORTH);
        getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.WEST);
        getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.EAST);
        setTitle(space.name()+" GUI: "+host);

        sketchPanel.setMinimumSize(new Dimension(imageWidth/2, imageHeight/2));
        // Make window big enough to see sketches clearly, but not too big.
        float scaleFactor = (imageWidth <= 320) ? 2 : (imageWidth >= 1000) ? 0.5f : 1;
        sketchPanel.setPreferredSize(new Dimension(Math.round(imageWidth*scaleFactor),
                                                   Math.round(imageHeight*scaleFactor)));
        sketchPanel.setLockAspectRatio(true);

        curSketchPanel = sketchPanel;
        panelTitle = space.name() + " view"; // don't append host here because of clones
        sketchPanel.makeSketchFrame(sketchPanel, panelTitle+": "+host);
        {

            Box tmp1 = Box.createHorizontalBox();
            tmp1.add(Box.createHorizontalStrut(strutsize));
            {
                Box tmp2 = Box.createVerticalBox();
                tmp2.add(Box.createVerticalStrut(strutsize));

                {
                    Box buttonBox = Box.createVerticalBox();
                    {
                        Box tmp3 = Box.createHorizontalBox();
                        tmp3.add(status=new JLabel(state));
                        //tmp3.add(Box.createHorizontalGlue());

                        //tmp3.add(Box.createHorizontalStrut(strutsize));

                        rescaleBut = new JButton("Rescale");
                        rescaleBut.setAlignmentX(0.5f);
                        rescaleBut.addActionListener(this);
                        rescaleBut.setActionCommand("rescale");
                        rescaleBut.setEnabled(true);
                        rescaleBut.setToolTipText("Rescales the displayed sketch;");
                        tmp3.add(rescaleBut);
                        tmp3.add(Box.createHorizontalStrut(strutsize));

                        refreshListBut = new JButton("Refresh Listing");
                        refreshListBut.setAlignmentX(0.5f);
                        refreshListBut.addActionListener(this);
                        refreshListBut.setActionCommand("refreshlist");
                        refreshListBut.setEnabled(true);
                        refreshListBut.setToolTipText("Refreshes the sketch listing;");
                        tmp3.add(refreshListBut);
                        tmp3.add(Box.createHorizontalStrut(strutsize));

                        if (space == Space.world || space == Space.local)
                        {
                            threedBut = new JButton("3D View");
                            threedBut.setAlignmentX(0.5f);
                            threedBut.addActionListener(this);
                            threedBut.setActionCommand("3dgui");
                            threedBut.setEnabled(true);
                            threedBut.setToolTipText("Open the 3D SketchPanel");
                            tmp3.add(threedBut);
                            tmp3.add(Box.createHorizontalStrut(strutsize/2));
                        }
                        colorsBut = new JButton("Colors");
                        colorsBut.setAlignmentX(0.5f);
                        colorsBut.addActionListener(this);
                        colorsBut.setActionCommand("showcolors");
                        colorsBut.setEnabled(true);
                        colorsBut.setToolTipText("Display color visibility tools;");
                        tmp3.add(colorsBut);
                        tmp3.add(Box.createGlue());
                        buttonBox.add(tmp3);

                    }

                    buttonBox.add(Box.createVerticalStrut(strutsize));
                    {
                        Box tmp4 = Box.createHorizontalBox();
                        //tmp4.add(Box.createHorizontalGlue());
                        //tmp4.add(Box.createHorizontalStrut(strutsize));

                        checkAllBut = new JCheckBox("Select All Shapes", defaultSelected);
                        checkAllBut.setAlignmentX(0.5f);
                        checkAllBut.addActionListener(this);
                        checkAllBut.setActionCommand("checkall");
                        checkAllBut.setEnabled(true);
                        checkAllBut.setToolTipText("Selects all shapes for display.");
                        tmp4.add(checkAllBut);
                        tmp4.add(Box.createHorizontalStrut(strutsize));

                        invertAllBut = new JCheckBox("Invert All Shapes", defaultInversion);
                        invertAllBut.setAlignmentX(0.5f);
                        invertAllBut.addActionListener(this);
                        invertAllBut.setActionCommand("invertall");
                        invertAllBut.setEnabled(true);
                        invertAllBut.setToolTipText("Inverts all shapes.");
                        tmp4.add(invertAllBut);
                        tmp4.add(Box.createGlue());

			autoRefreshBut = new JCheckBox("Auto-Refresh", autoRefreshEnabled);
			autoRefreshBut.setAlignmentX(0.5f);
			autoRefreshBut.addActionListener(this);
			autoRefreshBut.setActionCommand("autorefresh");
			autoRefreshBut.setEnabled(true);
			autoRefreshBut.setToolTipText("Enable/disable automatic refresh.");
			tmp4.add(autoRefreshBut);
			tmp4.add(Box.createGlue());

                        buttonBox.add(tmp4);
                    }

                    tmp2.add(buttonBox, BorderLayout.CENTER);
                }

                // create box for two panes
                tmp2.add(Box.createHorizontalStrut(strutsize));
                {
                    Box paneBox = Box.createHorizontalBox();

                    // Sketch Tree:
                    sketchTree = new JTree(new DefaultTreeModel(initSketchTree(host, listingPort)));
                    sketchPane = new JScrollPane(sketchTree);
                    paneBox.add(sketchPane);
                    // set up sketch node selection
                    sketchTree.getSelectionModel().setSelectionMode(TreeSelectionModel.DISCONTIGUOUS_TREE_SELECTION);
		    sketchTree.setExpandsSelectedPaths(false); // only manual path expansion
                    //Listen for when the selection changes.
                    sketchTree.addTreeSelectionListener(this);
                    sketchTree.addMouseListener(this);
                    sketchTree.setCellRenderer(new SketchTreeRenderer());
		    sketchTree.addTreeWillExpandListener(this);

                    paneBox.add(Box.createHorizontalStrut(strutsize));

		    //Color Tree
                    Box colorBox = Box.createVerticalBox();
                    colorTree = new JTree(new DefaultTreeModel(initColorTree(host, listingPort)));
                    colorPane = new JScrollPane(colorTree);
                    colorPane.setVisible(false);
                    colorBox.add(colorPane);

                    paneBox.add(colorBox);

                    // set up sketch node selection
                    colorTree.getSelectionModel().setSelectionMode(TreeSelectionModel.DISCONTIGUOUS_TREE_SELECTION);
                    //Listen for when the selection changes.
                    colorTree.addTreeSelectionListener(this);
                    colorTree.addMouseListener(this);
                    colorTree.setCellRenderer(new SketchTreeRenderer());

                    tmp2.add(paneBox);
                }


                tmp2.add(Box.createVerticalStrut(strutsize));
                {
                    Box tmp4 = Box.createHorizontalBox();
                    tmp4.add(status=new JLabel(state));
                    tmp4.add(Box.createHorizontalGlue());
                    tmp2.add(tmp4);
                }
                tmp2.add(Box.createVerticalStrut(strutsize));
                tmp1.add(tmp2);
            }            tmp1.add(Box.createHorizontalStrut(strutsize));
            getContentPane().add(tmp1,BorderLayout.CENTER);
        }
        pack();

        String name="SketchGUI"+".location";
        setLocation(prefs.getInt(name+".x",50),prefs.getInt(name+".y",50));
        addWindowListener(new CloseSketchGUIAdapter(this));

        //sketchPanel.getListener().addListener(this);


        popMenu = new PopupMenu();
        MenuItem invBox = new MenuItem("Invert");
        invBox.addActionListener(new PopupInvertListener());
        popMenu.add(invBox);
        MenuItem newWindow = new MenuItem("Clone current window");
        newWindow.addActionListener(new PopupNewWindowListener(this));
        popMenu.add(newWindow);
        this.add(popMenu);

        refreshListBut.doClick();
	expandAll();
    }

    class CloseSketchGUIAdapter extends WindowAdapter {
        SketchGUI gui;
        CloseSketchGUIAdapter(SketchGUI _gui) { gui = _gui; }
        public void windowClosing(WindowEvent e) {
            gui.close();
        }
    }

    class PopupInvertListener implements ActionListener {
        public void actionPerformed(ActionEvent e) {
            if (currentMenuObject != null) {
                currentMenuObject.invert();
                if (currentMenuObject instanceof ShapeInfo)
                    ((ShapeInfo)currentMenuObject).setUseDefaultInversion(false);
		else
		    imageNeedsUpdate = true;
                updateDisplay();
		sketchTree.treeDidChange();   // update the icon
            }
        }
    }

    class PopupNewWindowListener implements ActionListener {
        SketchGUI gui;

        public PopupNewWindowListener(SketchGUI _gui) { gui = _gui; }

        public void actionPerformed(ActionEvent e)
        {
            SketchPanel sketchPanel=new SketchPanel(gui, listener, Space.cam, prefs, imageWidth, imageHeight);
            sketchPanel.setMinimumSize(new Dimension(imageWidth/2, imageHeight/2));
            sketchPanel.setPreferredSize(new Dimension(imageWidth*2, imageHeight*2));
            sketchPanel.setLockAspectRatio(true);

            sketchPanel.makeSketchFrame(sketchPanel, "Dummy");

            sketchPanel.imageUpdated(getSketchImage(),sketchTree.getSelectionPaths());
        }
    }

    public void close() {
        try {
            if(listingSocket!=null && !listingSocket.isClosed())
                listingSocket.close();
        } catch (IOException ioe) {
            System.err.println("close failed:");
            ioe.printStackTrace();
        }
        prefs.putInt("SketchGUI.location.x",getLocation().x);
        prefs.putInt("SketchGUI.location.y",getLocation().y);
        sketchPanel.getListener().kill();
        Component p=sketchPanel;
        while(p.getParent()!=null)
            p=p.getParent();
        if(p instanceof Window) {
            Window w=(Window)p;
            prefs.putInt("SketchPanel.location.x",w.getLocation().x);
            prefs.putInt("SketchPanel.location.y",w.getLocation().y);
            w.dispose();
        } else
            System.out.println("That's weird - root container isn't window");
        if (is3d)
            openOrClose3DPanel();
        try { dispose(); } catch(Exception ex) {
	    System.err.println("SketchGUI " + space.name() + " got " + ex);
	}
    }

    public TreeNode initSketchTree(String host, int port) {
        // Just for demonstration:
        DefaultMutableTreeNode cam = new DefaultMutableTreeNode("hit refresh");
        root.insert(cam,0);
        return root;
    }

    public TreeNode initColorTree(String host, int port) {
        return colorRoot;
    }


    //================ sortTree ================
    public void sortTree(DefaultMutableTreeNode rootNode) {
        // set up comparator
        Comparator comp =
            new Comparator() {
                public int compare(Object o1, Object o2) {
                    return compare((DefaultMutableTreeNode) o1, (DefaultMutableTreeNode) o2);
                }
                public int compare(DefaultMutableTreeNode n1, DefaultMutableTreeNode n2) {
		    if ( ! (n1.getUserObject() instanceof SketchOrShapeInfo) ) return 0;
		    if ( ! (n2.getUserObject() instanceof SketchOrShapeInfo) ) return 0;
                    Integer i1 = new Integer(((SketchOrShapeInfo)n1.getUserObject()).id);
                    Integer i2 = new Integer(((SketchOrShapeInfo)n2.getUserObject()).id);
                    if ( (n1.getUserObject() instanceof LocalizationParticleShapeInfo) ==
                         (n2.getUserObject() instanceof LocalizationParticleShapeInfo) )
                        return i1.compareTo(i2);
                    else
                        if (n1.getUserObject() instanceof LocalizationParticleShapeInfo)
                            return 1;
                        else
                            return -1;
                }
            };

        // sort the roots children
        Object[] objs = new Object[rootNode.getChildCount()];
        Enumeration children = rootNode.children();
        for (int i=0;children.hasMoreElements();i++) {
            DefaultMutableTreeNode child = (DefaultMutableTreeNode) children.nextElement();
            objs[i] = child;
        }

        Arrays.sort(objs, comp);
        rootNode.removeAllChildren();

        // insert newly ordered children
        for (int i=0;i<objs.length;i++) {
            DefaultMutableTreeNode orderedNode = (DefaultMutableTreeNode) objs[i];
            rootNode.add(orderedNode);
            if (!orderedNode.isLeaf()) {
                sortTree(orderedNode);
            }
        }
    }


    //================ mouse click handling ================

    public void mousePressed(MouseEvent e) {
	mouseClickModifiers = e.getModifiersEx();
	// System.out.println("\nMouse pressed, modifier = "+mouseClickModifiers);
	curPath = null;
	curNode = null;

	// Make sure we're clicking in the sketch panel, not the color panel
	Point windowLoc = getLocation();
	Point clickLoc = e.getLocationOnScreen();
	int clickRelativeX = clickLoc.x - windowLoc.x;
	int sketchRightEdge = sketchPanel.getWidth() + sketchPanel.getLocation().x;
	if ( clickRelativeX > sketchRightEdge ) return;

	curPath = sketchTree.getPathForLocation(e.getX(),e.getY());
        if (curPath == null) return;
	curNode = (DefaultMutableTreeNode)curPath.getLastPathComponent();
	Object obj = curNode.getUserObject();
	if (e.getButton() == MouseEvent.BUTTON3 && obj instanceof SketchOrShapeInfo) {
	    currentMenuObject = (SketchOrShapeInfo)obj;
	    popMenu.show(sketchTree, e.getX(), e.getY());
	    lastSelectionEvent = null;
	    return;
	}
	if ( lastSelectionEvent != null )
	    deferredValueChanged(lastSelectionEvent);
	lastSelectionEvent = null;
    }

    // We have to override these methods or Java complains
    public void mouseClicked(MouseEvent e) {}
    public void mouseReleased(MouseEvent e){}
    public void mouseEntered(MouseEvent e) {}
    public void mouseExited(MouseEvent e)  {}

    public void treeWillExpand(TreeExpansionEvent e) throws ExpandVetoException {
	/****************************************************************
	Note: the order of callbacks is:
		1. TreeWillExpand
		2. ValueChanged
		3. MousePressed
	****************************************************************/
    }

    public void treeWillCollapse(TreeExpansionEvent e) {
	treeJustCollapsed = true;
    }

    public void valueChanged(TreeSelectionEvent e) {
	// If this event was triggered by a tree collapse event, undo
	// the deselections.  Otherwise cache this event until we can
	// find out what kind of mouse click triggered it.
	/*
	  System.out.println("valueChanged path len=" + e.getPaths().length);
	  TreePath[] dpaths = e.getPaths();
	  int dnpaths = dpaths.length;
	  for (int i=0; i < dnpaths && i<5; i++)
	      System.out.println((e.isAddedPath(i) ? "  add " : "  remove ") + dpaths[i]);
	*/
	  if ( treeJustCollapsed ) {
	      // Collapsing causes deselection, which we must undo
	      TreePath[] paths = e.getPaths();
	      int npaths = paths.length;
	      for (int i=0; i < npaths && i<5; i++)
		  if ( ((DefaultMutableTreeNode)paths[i].getLastPathComponent()).getUserObject() instanceof SketchOrShapeInfo )
		      sketchTree.addSelectionPath(paths[i]);
	      treeJustCollapsed = false;
	      lastSelectionEvent = null;
	  } else {
	      if ( lastSelectionEvent == null ) {
		  lastSelectionEvent = e;
		  // System.out.println("  set lastSelectionEvent");
	      }
	  }
    }

    void deferredValueChanged(TreeSelectionEvent e) {
	TreePath[] paths = e.getPaths();
	int npaths = paths.length;
	// System.out.println("deferredValueChanged:  npaths=" + npaths);
	// for (int i=0; i < npaths && i<5; i++)
	//     System.out.println((e.isAddedPath(i) ? "  add " : "  remove ") + paths[i]);

	if ( e.getPath().getPath()[0] == colorRoot ) {
	    // System.out.println("Ignoring color click");
	    valueChangedColorCheck();
	    return;
	}

        imageNeedsUpdate = true;
	int button1Stuff = InputEvent.CTRL_DOWN_MASK | InputEvent.SHIFT_DOWN_MASK | InputEvent.BUTTON1_DOWN_MASK;
	int button1 = InputEvent.BUTTON1_DOWN_MASK;
	int controlButton1 = InputEvent.CTRL_DOWN_MASK | InputEvent.BUTTON1_DOWN_MASK;
	int shiftButton1 = InputEvent.SHIFT_DOWN_MASK | InputEvent.BUTTON1_DOWN_MASK;

	if ( (mouseClickModifiers & button1Stuff) == shiftButton1 ||
	     (mouseClickModifiers & button1Stuff) == controlButton1 ) {
	    // For control-click and shift-click: JTree handles selection/deselection.
	    // All we need to do is propagate (de)selection of GraphicsShape children
	    for (int i=0; i<paths.length; i++) {
		Object uobj = ((DefaultMutableTreeNode)paths[i].getLastPathComponent()).getUserObject();
		if ( uobj instanceof GraphicsShapeInfo )
		    selectGraphicsObjects(paths[i], e.isAddedPath(paths[i]));
	    }
	} else if ( (mouseClickModifiers & button1Stuff) == button1 ) {
	    // For regular click, JTree will deselect everything else.
	    // Use the paths provided to undo this deselection for the
	    // previously selected sketches (or shapes, as
	    // appropriate).
	    Object temp = curNode.getUserObject();
	    SketchOrShapeInfo curObject = (temp instanceof SketchOrShapeInfo) ? (SketchOrShapeInfo)temp : null;
	    boolean isSketch = (curObject instanceof SketchInfo);
	    boolean isShape = (curObject instanceof ShapeInfo);
	    for (int i=0; i<npaths; i++) {
		Object uobj = ((DefaultMutableTreeNode)paths[i].getLastPathComponent()).getUserObject();
		if ( uobj instanceof SketchOrShapeInfo )
		    if ( (!isSketch && uobj instanceof SketchInfo) ||
			 (!isShape && uobj instanceof ShapeInfo) ) {
			// Undo the appropriate types of deselections
			if (! e.isAddedPath(i) ) {
			    // System.out.println("  re-added: "+paths[i]);
			    sketchTree.addSelectionPath(paths[i]);
			}
		    }
	    }
	    // If we clicked on a GraphicsShape that was already selected,
	    // it won't appear in the paths list, so check for it explicitly.
	    if ( curObject instanceof GraphicsShapeInfo)
		selectGraphicsObjects(curPath, true);
	}
	sketchTree.revalidate();

	valueChangedColorCheck();
	focusJustChanged = false;

	updateDisplay();

	curPath = null;
	curNode = null;
	// System.out.println("Exit valueChanged");
    }

    void updateDisplay() {
        TreePath[] selPaths = sketchTree.getSelectionPaths();
        if (selPaths == null) {
	    // Nothing is selected: blank the sketch view and exit
            curSketchPanel.imageUpdated(null, null);
            if (sketchPanel3D != null)
		sketchPanel3D.imageUpdated(null);
            return;
        }

	// For all selected sketches: add to itemStack for rendering.
	itemStack.clear();
        for (TreePath path : selPaths) {
            DefaultMutableTreeNode node = (DefaultMutableTreeNode)(path.getLastPathComponent());
            if (node != root && node.getUserObject() instanceof SketchInfo) {
		SketchInfo obj = (SketchInfo)(node.getUserObject());
		if ( ! obj.isImageLoaded() )
		    loadImage(obj);
		curSketchPanel.scaleToSketchOrShape(obj);
		if ( ! itemStack.contains(obj) )
		    itemStack.add(obj);
	    }
	}
        curSketchPanel.imageUpdated(getSketchImage(), selPaths);
        if (is3d)
            sketchPanel3D.imageUpdated(selPaths);
    }

    void selectGraphicsObjects(TreePath path, boolean isSelected) {
	Enumeration nodeEnum = ((DefaultMutableTreeNode)path.getLastPathComponent()).children();
	while (nodeEnum.hasMoreElements()) {
	    TreePath childPath = new
		TreePath(((DefaultMutableTreeNode)nodeEnum.nextElement()).getPath());
	    if (isSelected) {
		// System.out.println("  adding graphics child: " + childPath);
		sketchTree.addSelectionPath(childPath);
	    } else {
		// System.out.println("  removing graphics child: " + childPath);
		sketchTree.removeSelectionPath(childPath);
	    }
	}
	sketchTree.revalidate();
    }

    void valueChangedColorCheck() {
        TreePath[] newColors = colorTree.getSelectionPaths();  // returns null rather than empty array
	if (newColors == null)
	    newColors = new TreePath[0];
	if (prevColors == null)
	    prevColors = new TreePath[0];

        // check for new colors added
        for (TreePath newcolorpath : newColors) {
            if (!colorGroups.containsKey(newcolorpath.getLastPathComponent())) continue;
            boolean found = false;
            for (TreePath prevcolorpath : prevColors) {
                if (!colorGroups.containsKey(prevcolorpath.getLastPathComponent())) continue;
                if (newcolorpath.equals(prevcolorpath)) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                DefaultMutableTreeNode colorNode = (DefaultMutableTreeNode)newcolorpath.getLastPathComponent();
                for (DefaultMutableTreeNode node : colorGroups.get(colorNode))
                    if (node.getUserObject() instanceof ShapeInfo &&
			! (node.getUserObject() instanceof GraphicsShapeInfo) )
                      sketchTree.addSelectionPath(new TreePath(node.getPath()));
            }
        }

        // check for disabled color
        for (TreePath prevcolorpath : prevColors) {
            if (!colorGroups.containsKey(prevcolorpath.getLastPathComponent())) continue;
            boolean found = false;
            for (TreePath newcolorpath : newColors) {
                if (!colorGroups.containsKey(newcolorpath.getLastPathComponent())) continue;
                if (prevcolorpath.equals(newcolorpath)) {
                    found = true;
                    break;
                }
            }
            // something was previously selected, but now is not
            if (!found) {
                DefaultMutableTreeNode colorNode = 
		    (DefaultMutableTreeNode)prevcolorpath.getLastPathComponent();
                for (DefaultMutableTreeNode node : colorGroups.get(colorNode)) {
                    SketchOrShapeInfo obj = (SketchOrShapeInfo)node.getUserObject();
                    if ( obj instanceof ShapeInfo &&
			 ! (obj instanceof GraphicsShapeInfo &&
			    ! (obj instanceof GraphicsShapeInfo.GraphicsElement)) )
			sketchTree.removeSelectionPath(new TreePath(node.getPath()));
                }
            }
        }

        prevColors = colorTree.getSelectionPaths();
    }

    //================ loadImage ================
    void loadImage (SketchOrShapeInfo vinfo) {
	((TCPVisionListener)listener).setReadingImage();
	netout.println("get "+vinfo.id);
	try {
	    String inputLine;
	    while((inputLine=readLine()).compareTo("get end") != 0) {
		System.out.println(inputLine);
	    }
	} catch (IOException ioe) {
	    System.err.println("Transfer error");
	    reconnect();
	    updateDisplay();
	}
	while(((TCPVisionListener)listener).isReadingImage())
	    // thread.yield() in java?
	    // *** The following sleep is needed to prevent some kind of thread lockup.  -- DST 1/30/2009
	    try { Thread.sleep(1); } catch (InterruptedException ie) {}
	System.out.println("done with id:"+vinfo.id);
	((SketchInfo)vinfo).copyImage(((TCPVisionListener)listener).getImage());
    }

    protected String readLine() throws java.io.IOException {
        if(netin==null)
            reconnect();
        if(netin==null)
            throw new java.io.IOException("no connection");
        String ans=netin.readLine();
        if(ans==null)
            throw new java.io.IOException("lost connection");
        return ans;
    }

    public void reconnect() {
        System.out.print(host+":"+listingPort+" reconnecting...");
        netout = null;
        try {
            if(listingSocket!=null && !listingSocket.isClosed())
                listingSocket.close();
            netin=null;
            netout=null;
            listingSocket = new Socket(host,listingPort);
            netout = new PrintWriter(listingSocket.getOutputStream(), true);
            netin = new BufferedReader(new InputStreamReader(listingSocket.getInputStream()));
        } catch (UnknownHostException e) {
            System.err.println("Don't know about host:"+host);
            System.exit(1);
        } catch (IOException ioe) {
            System.err.println("reconnection failed: "+ioe);
            //ioe.printStackTrace();
            return;
        }
        System.out.println("done");
        if (refreshListBut != null) {
            refreshListBut.doClick(); //auto refresh on reconnect
        }
    }

    public void visionUpdated(VisionListener listener) {}
    public void sensorsUpdated(VisionListener l) {}

    public BufferedImage getSketchImage() {
        if (imageNeedsUpdate) createSketchImage();
        return img;
    }

    private void createSketchImage() {
        // Cull not visible objects from vector
        // render all the sketches in the vector

        img = new BufferedImage(imageWidth, imageHeight, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = img.createGraphics();
        g.setBackground(backgroundColor);
        g.clearRect(0,0,imageWidth, imageHeight);

        int pixcount = img.getWidth()*img.getHeight();
        int rarr[] = new int[pixcount];
        int garr[] = new int[pixcount];
        int barr[] = new int[pixcount];
        int counts[] = new int[pixcount];
        for (int i=0; i < pixcount; i++) {
            rarr[i] = 0;
            garr[i] = 0;
            barr[i] = 0;
            counts[i] = 0;
        }

        // Draw SketchINTS
        for (int i=0; i<itemStack.size(); i++) {
            SketchOrShapeInfo vinfo = (SketchOrShapeInfo)itemStack.elementAt(i);
	    if (vinfo instanceof SketchInfo &&
		  ((SketchInfo)vinfo).getSketchType() != SketchInfo.SKETCH_BOOL_TYPE)
	      ((SketchInfo)vinfo).renderToArrays(rarr,garr,barr,counts);
        }

        for (int y=0; y<img.getHeight(); y++) {
            for (int x=0; x<img.getWidth(); x++) {
                int pos = y*img.getWidth()+x;
                if (counts[pos] > 0) {
                    g.setColor(new Color(rarr[pos]/counts[pos],
                                         garr[pos]/counts[pos],
                                         barr[pos]/counts[pos]));
                    g.drawLine(x,y,x,y);
                }
            }
        }

        // Draw SketchBOOLs
        for (int i=0; i < img.getWidth()*img.getHeight(); i++) {
            rarr[i] = 0;
            garr[i] = 0;
            barr[i] = 0;
            counts[i] = 0;
        }

        for (int i=0; i<itemStack.size(); i++) {
            SketchOrShapeInfo vinfo = (SketchOrShapeInfo)itemStack.elementAt(i);
	    if (vinfo instanceof SketchInfo &&
		((SketchInfo)vinfo).getSketchType() == SketchInfo.SKETCH_BOOL_TYPE)
	      ((SketchInfo)vinfo).renderToArrays(rarr,garr,barr,counts);
        }

        for (int y=0; y<img.getHeight(); y++) {
            for (int x=0; x<img.getWidth(); x++) {
                int pos = y*img.getWidth()+x;
                if (counts[pos] > 0) {
                    g.setColor(new Color(rarr[pos]/counts[pos],
                                         garr[pos]/counts[pos],
                                         barr[pos]/counts[pos]));
                    g.drawLine(x,y,x,y);
                }
            }
        }

        imageNeedsUpdate = false;
    }

    public void setCurrentSketchPanel(SketchPanel s, TreePath[] paths) {
        if (curSketchPanel != s) {
            curSketchPanel.loseFocus();
            focusJustChanged = true;
        }
        curSketchPanel = s;
    }

    public void autoRefreshSketch() {  // called in ControllerListener.java
    	if (refreshListBut != null && autoRefreshEnabled)
    	    refreshListBut.doClick();
    }

    private class SketchTreeRenderer extends DefaultTreeCellRenderer {

        public SketchTreeRenderer() {}

        public Component getTreeCellRendererComponent(JTree tree,
                                                      Object value,
                                                      boolean sel,
                                                      boolean expanded,
                                                      boolean leaf,
                                                      int row,
                                                      boolean hasFocus) {
            super.getTreeCellRendererComponent(tree, value, sel,
                                               expanded, leaf, row,
                                               hasFocus);
            try {
                SketchOrShapeInfo vinfo = (SketchOrShapeInfo)(((DefaultMutableTreeNode)value).getUserObject());

                setIcon(vinfo.getIcon());
                setToolTipText(vinfo.toString());

            } catch (ClassCastException e) {}

            return this;
        }
    }

} // class
