
/** @file EasyTrain.java
 *  @brief
 *
 *  Color segmentation tool for the Tekkotsu framework based on the VisionTrain and TileTrain tools
 *
 *	@author editted by: Eric Durback
 *  @bug No known bugs.





** NOTE FOR LINUX USERS **
If your using linux, and you are receiving warnings about preferences, look at
the installation notes for java on linux at the link below will help
http://java.sun.com/j2se/1.5.0/install-linux.html 
**************************

TODO

-redraw on resize
-resize stopping over another window

*/


import java.awt.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.undo.UndoManager;
import javax.swing.undo.UndoableEdit;
import javax.swing.undo.UndoableEditSupport;
import javax.swing.undo.AbstractUndoableEdit;
import java.awt.event.*;
import java.util.*;
import java.awt.geom.*;
import java.io.*;
import java.util.prefs.Preferences;
import java.util.Vector;
import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeEvent;

public class EasyTrain extends JFrame implements ComponentListener, ImageSelectionManager.SelectionListener,
ActionListener, ListSelectionListener, PropertyChangeListener, ListDataListener, DocumentListener, UndoableEditListener
{
	static WindowAdapter exitOnClose=new WindowAdapter() { public void windowClosing(WindowEvent e) { System.exit(0); } };
	static Preferences prefs = Preferences.userNodeForPackage(EasyTrain.class);

	static String specExt=".spc";

	ImageData imageData;
	ColorConverter toRGB,toYUV,YUVtoSpectrum;
	
	SpectrumViewer spectrumFrame;
	ImagePanel spectrum;
	ImageSelectionManager spectrumSelection;
	ImageSelectionManager rgbSelection;
	
	String space;
	JComboBox spacelist;

	ThumbnailShow thumb;
	ImageViewer rgbImageShow;
	SegmentedImage segmentedImageShow;
	HelpBox helpBox;
	HintController hc;
	
	Vector imageAreas; //collection of Area[]s, one per color, one Area per image file
	Vector colorAreas; //collection of Areas, one per color
	
	KeyedUndoManager undoManager;
	UndoableEditSupport undoListeners;
	
	class HintController extends MouseInputAdapter {
		ImagePanel spectrum;
		ImageData data;
		Vector panels;
		float[] img;
		int width;
		int height;
		
		public HintController(ImagePanel spectrum) {
			this.spectrum=spectrum;
			this.data=data;
			panels=new Vector();
		}
		public void addImagePanel(ImagePanel img) {
			panels.add(img);
			addMouseEventSource(img);
		}
		public void addMouseEventSource(Component evtSrc) {
			evtSrc.addMouseListener(this);
			evtSrc.addMouseMotionListener(this);
		}
		public void setImage(float[] img, int width, int height) {
			this.img=img;
			this.width=width;
			this.height=height;
		}
		public void mouseMoved(MouseEvent e) {
			Point p=e.getPoint();
			double x=(p.getX()+.5)/e.getComponent().getWidth();
			double y=(p.getY()+.5)/e.getComponent().getHeight();
			Point2D.Double p2d=new Point2D.Double(x,y);
			for(Iterator it=panels.iterator(); it.hasNext();) {
				ImagePanel cur=(ImagePanel)it.next();
				if(cur!=e.getComponent()) {
					cur.setHint(p2d);
				}
			}
			int px=(int)(x*width);
			int py=(int)(y*height);
			int i=(py*width+px)*3;
			spectrum.setHint(new Point2D.Float(img[i],img[i+1]));
		}
		public void mouseDragged(MouseEvent e) {
			Point p=e.getPoint();
			Component c=e.getComponent();
			if(p.getX()<0 || p.getX()>=c.getWidth() || p.getY()<0 || p.getY()>=c.getHeight()) {
				//in case a drag goes outside
				for(Iterator it=panels.iterator(); it.hasNext();)
					((ImagePanel)it.next()).setHint(null);
				spectrum.setHint(null);
			} else {
				mouseMoved(e);
			}
		}
		public void mouseExited(MouseEvent e) {
			for(Iterator it=panels.iterator(); it.hasNext();)
				((ImagePanel)it.next()).setHint(null);
			spectrum.setHint(null);
		}
	}
	
	class KeyedUndoManager extends UndoManager implements KeyListener {
		JComponent undoBut,redoBut;
		KeyedUndoManager(JComponent undoBut, JComponent redoBut) {
			super();
			this.undoBut=undoBut;
			this.redoBut=redoBut;
		}
		public boolean addEdit(UndoableEdit anEdit) {
			boolean ans=super.addEdit(anEdit);
			refreshButtons();
			return ans;
		}
		public void undoableEditHappened(UndoableEditEvent e) {
			super.undoableEditHappened(e);
			refreshButtons();
		}
		public void undo() {
			super.undo();
			refreshButtons();
		}
		public void redo() {
			super.redo();
			refreshButtons();
		}
		public void keyPressed(KeyEvent e) {
			if((e.getModifiersEx()&(InputEvent.META_DOWN_MASK|InputEvent.CTRL_DOWN_MASK))!=0) {
				if(e.getKeyCode()==KeyEvent.VK_Z) {
					if((e.getModifiersEx()&(InputEvent.SHIFT_DOWN_MASK))!=0) {
						//System.out.println("canRedo()=="+canRedo());
						if(canRedo())
							redo();
					} else {
						//System.out.println("canUndo()=="+canUndo());
						if(canUndo())
							undo();
					}
				}
			}
		}
		public void keyReleased(KeyEvent e) {} 
		public void keyTyped(KeyEvent e) {}
		void refreshButtons() {
			if(undoBut!=null)
				undoBut.setEnabled(canUndo());
			if(redoBut!=null)
				redoBut.setEnabled(canRedo());
		}
	}

	class ListSelectionEdit extends AbstractUndoableEdit {
		EasyTrain src;
		int previous, next;
		ListSelectionEdit(EasyTrain src, int previous, int next) {
			super();
			this.src=src;
			this.previous=previous;
			this.next=next;
		}
		public void redo() {
			super.redo();
			//System.out.println("Redo: "+this);
			src.setIgnoreUndoRedo(true);
			if(next<0)
				src.colorlist.getSelectionModel().clearSelection();
			else
				src.colorlist.setSelectedIndex(next);
			src.setIgnoreUndoRedo(false);
		}
		public void undo() {
			super.undo();
			//System.out.println("Undo: "+this);
			src.setIgnoreUndoRedo(true);
			if(previous<0)
				src.colorlist.getSelectionModel().clearSelection();
			else
				src.colorlist.setSelectedIndex(previous);
			src.setIgnoreUndoRedo(false);
		}
		public boolean isSignificant() {
			return false;
		}
		public String toString() {
			return super.toString()+" previous:"+previous+" next:"+next;
		}
	}
	class AddColorEdit extends AbstractUndoableEdit {
		EasyTrain src;
		String name;
		int index;
		AddColorEdit(EasyTrain src, String name, int index) {
			this.src=src;
			this.name=name;
			this.index=index;
			//System.out.println("AddColorEdit("+src+","+name+","+index+")");
		}
		public void redo() {
			super.redo();
			//System.out.println("Redo: "+this);
			src.setIgnoreUndoRedo(true);
			src.list.add(index,name); //list listener should add (empty) entries for colorAreas and imageAreas
			src.setIgnoreUndoRedo(false);
		}
		public void undo() {
			super.undo();
			//System.out.println("Undo: "+this);
			src.setIgnoreUndoRedo(true);
			src.list.remove(index); //list listener should remove entries for colorAreas and imageAreas
			src.setIgnoreUndoRedo(false);
		}
		public String toString() {
			return super.toString()+" name:"+name+" position:"+index;
		}
	}
	class RemoveColorEdit extends AbstractUndoableEdit {
		EasyTrain src;
		String name;
		int index;
		Area colorArea;
		Area[] imageAreas;
		RemoveColorEdit(EasyTrain src, String name, int index, Area colorArea, Area[] imageAreas) {
			this.src=src;
			this.name=name;
			this.index=index;
			this.colorArea=colorArea;
			this.imageAreas=imageAreas;
		}
		public void redo() {
			super.redo();
			//System.out.println("Redo: "+this);
			src.setIgnoreUndoRedo(true);
			src.list.remove(index); //list listener should remove entries for colorAreas and imageAreas
			src.colorlist.getSelectionModel().clearSelection();
			src.segmentedImageShow.updateMap(src.colorAreas);
			src.setIgnoreUndoRedo(false);
		}
		public void undo() {
			super.undo();
			//System.out.println("Undo: "+this);
			src.setIgnoreUndoRedo(true);
			src.list.add(index,name);//list listener should add (empty) entries for colorAreas and imageAreas
			src.colorAreas.setElementAt(colorArea,index);
			src.imageAreas.setElementAt(imageAreas,index);
			src.colorlist.setSelectedIndex(index);
			src.segmentedImageShow.updateMap(src.colorAreas);
			src.setIgnoreUndoRedo(false);
		}
		public String toString() {
			return super.toString()+" name:"+name+" index:"+index;
		}
	}
	class RenameColorEdit extends AbstractUndoableEdit {
		EasyTrain src;
		String prevname;
		String newname;
		int index;
		RenameColorEdit(EasyTrain src, String prevname, String newname, int index) {
			this.src=src;
			this.prevname=prevname;
			this.newname=newname;
			this.index=index;
		}
		public void redo() {
			super.redo();
			//System.out.println("Redo: "+this);
			src.setIgnoreUndoRedo(true);
			src.list.setElementAt(newname,index);
			src.colorlist.setSelectedIndex(index);
			src.valueChanged(null);
			src.setIgnoreUndoRedo(false);
		}
		public void undo() {
			super.undo();
			//System.out.println("Undo: "+this);
			src.setIgnoreUndoRedo(true);
			src.list.setElementAt(prevname,index);
			src.colorlist.setSelectedIndex(index);
			src.valueChanged(null);
			src.setIgnoreUndoRedo(false);
		}
		public boolean addEdit(UndoableEdit anEdit) {
			if(anEdit instanceof RenameColorEdit) {
				RenameColorEdit rename=(RenameColorEdit)anEdit;
				if(rename.src!=src || rename.index!=index)
					return false;
				newname=rename.newname;
				rename.die();
				return true;
			}
			return false;
		}
		public boolean replaceEdit(UndoableEdit anEdit) {
			if(anEdit instanceof RenameColorEdit) {
				RenameColorEdit rename=(RenameColorEdit)anEdit;
				if(rename.src!=src || rename.index!=index)
					return false;
				prevname=rename.prevname;
				rename.die();
				return true;
			}
			return false;
		}
		public String toString() {
			return super.toString()+" prevname:"+prevname+" newname:"+newname+" index:"+index;
		}
	}
	public void addUndoableEditListener(UndoableEditListener l) { undoListeners.addUndoableEditListener(l); }
	public void removeUndoableEditListener(UndoableEditListener l) { undoListeners.removeUndoableEditListener(l); }
	
	boolean ignoreUndoRedo;
	void setIgnoreUndoRedo(boolean b) { ignoreUndoRedo=b; }
	boolean getIgnoreUndoRedo() { return ignoreUndoRedo; }
	public void undoableEditHappened(UndoableEditEvent e) {
		if(!getIgnoreUndoRedo()) {
			//System.out.println("Add Edit: "+e.getEdit());
			undoManager.addEdit(e.getEdit());
		}
	}
	
	public static void main(String args[]) 
	{
		if (args.length<2) 
		{
			usageAndExit();
		}

		boolean isRGB=true;
		if(args[0].equals("-isRGB"))
			isRGB=true;
		else if(args[0].equals("-isYUV"))
			isRGB=false;
		else 
		{
			System.out.println(args[0]+" is not valid color mode");
			usageAndExit();
		}
		String[] files;
		String space="HSB";
		if(args[1].equals("-show")) {
			space=args[2];
			if(!space.equalsIgnoreCase("YUV") && !space.equalsIgnoreCase("xy") && !space.equalsIgnoreCase("Lab") && !space.equalsIgnoreCase("HSB") && !space.equalsIgnoreCase("rg"))
				usageAndExit();
			files=new String[args.length-3];
			for(int i=0; i<files.length; i++)
				files[i]=args[i+3];
		} else {
			files=new String[args.length-1];
			for(int i=0; i<files.length; i++)
				files[i]=args[i+1];
		}
			
		//init training tool
		EasyTrain easyTrain = new EasyTrain(isRGB,space,files);
		
		easyTrain.addWindowListener(exitOnClose);
	}

	public static void usageAndExit() 
	{
		System.out.println("usage: java EasyTrain (-isRGB|-isYUV) [-show YUV|HSB|rg|xy|Lab] filename [filename ..]");
		System.out.println("       Select YUV mode if you use PNG images, RGB with JPEG.");
		System.out.println("       Using PNG format images is recommended.");
		System.out.println("       A mode must be specified.");
		System.exit(1);
	}

	public EasyTrain(boolean isRGB, String space, String files[]) 
	{
		System.out.println("Interpreting images as "+(isRGB?"RGB":"YUV")+" colorspace");
		System.out.println("Displaying spectrum as "+space+" colorspace");

		
		//INITIALIZING PREFS
		try{
			if(prefs.keys() == null || prefs.keys().length==0) //no prefs yet
		{
			setDefaultPrefs();    
		}
		}catch(Exception e){}
		
		//LOADING IMAGES
		toRGB=toYUV=YUVtoSpectrum=null;
		if(isRGB) {
			toYUV = new ColorConverter.VisionListenerRGBtoYUV();
		} else {
			toRGB = new ColorConverter.YUVtoRGB();
		}
		setSpace(space);
		//System.out.println("Press return to continue...");
		//try { System.in.read(); } catch(Exception e) {}
		//System.out.println("Loading...");
		imageData=new ImageData(files,toRGB,toYUV,YUVtoSpectrum);
		//System.out.println("done");
		imageAreas=new Vector();
		colorAreas=new Vector();
		undoManager=new KeyedUndoManager(null,null);
		undoListeners=new UndoableEditSupport();
		addUndoableEditListener(this);
		
		//INIT MAIN COMPONENTS
		setupSpectrumFrame();
		setupRGBImageShow();
		setupSegmentedImageShow();
		helpBox = new HelpBox();
		thumb = new ThumbnailShow(imageData);
		thumb.addKeyListener(undoManager);
		thumb.addUndoableEditListener(this);
		setupControlPanel((JComponent)getContentPane());
		
		hc = new HintController(spectrum);
		hc.setImage(imageData.getSpectrumImage(0),imageData.getWidth(0),imageData.getHeight(0));
		hc.addImagePanel(rgbImageShow.getDisplay());
		hc.addImagePanel(segmentedImageShow.getDisplay());
		
		rgbImageShow.addKeyListener(thumb);
		segmentedImageShow.addKeyListener(thumb);
		spectrumFrame.addKeyListener(thumb);
		thumb.addPropertyChangeListener(ThumbnailShow.CURIMG_PROPERTY,this);

		rgbImageShow.addWindowListener(exitOnClose);
		segmentedImageShow.addWindowListener(exitOnClose);
		spectrumFrame.addWindowListener(exitOnClose);		
		
		colorlist.requestFocusInWindow();
		colorlist.setSelectedIndex(0);
		undoManager.discardAllEdits(); //don't want to be able to undo past initialization!
		undoManager.undoBut=undo;
		
		thumb.setVisible(true);
		rgbImageShow.setVisible(true);
		restoreFromPrefs(rgbImageShow);
		rgbImageShow.addComponentListener(this);
		segmentedImageShow.setVisible(true);
		restoreFromPrefs(segmentedImageShow);
		segmentedImageShow.addComponentListener(this);
		spectrumFrame.setVisible(true);
		restoreFromPrefs(spectrumFrame);
		spectrumFrame.addComponentListener(this);
		setVisible(true);
	}
	
	void setSpace(String space) {
		this.space=space;
		if(space.equalsIgnoreCase("YUV")) {
			YUVtoSpectrum = new ColorConverter.YUVtoUVY();
		} else if(space.equalsIgnoreCase("xy")) {
			YUVtoSpectrum = new ColorConverter.Scale(new ColorConverter.YUVtoOther(new ColorConverter.RGBtoxy_()),1.5f);
		} else if(space.equalsIgnoreCase("Lab")) {
			YUVtoSpectrum = new ColorConverter.YUVtoOther(new ColorConverter.RGBtoabL());
		} else if(space.equalsIgnoreCase("HSB")) {
			YUVtoSpectrum = new ColorConverter.YUVtoOther(new ColorConverter.RGBtoRotatedHSB());
		} else if(space.equalsIgnoreCase("rg")) {
			YUVtoSpectrum = new ColorConverter.YUVtoOther(new ColorConverter.RGBtorg_());
		} else {
			usageAndExit();
		}
	}
	
	void setupRGBImageShow() {
		rgbImageShow = new ImageViewer("RGB Image View");
		rgbImageShow.setImage(imageData.getRGBImage(0));
		rgbImageShow.setName("RGBImageShow");
		restoreFromPrefs(rgbImageShow);
		JComponent comp=rgbImageShow.getDisplay();
		comp.setLayout(new BorderLayout());
		rgbSelection=new ImageSelectionManager(comp);
		rgbImageShow.addKeyListener(rgbSelection);
		rgbImageShow.addKeyListener(undoManager);
		rgbSelection.addSelectionListener(this);
		rgbSelection.addUndoableEditListener(this);
		comp.add(rgbSelection,BorderLayout.CENTER);
	}
	
	void setupSegmentedImageShow() {
		ThresholdMap tmap=new ThresholdMap(YUVtoSpectrum,16,64,64);
		segmentedImageShow = new SegmentedImage("Segmented Image View",tmap);
		segmentedImageShow.setImage(imageData.getYUVImage(0),imageData.getWidth(0),imageData.getHeight(0));
		segmentedImageShow.setName("SegmentedImageShow");
		restoreFromPrefs(segmentedImageShow);
		segmentedImageShow.addKeyListener(undoManager);
	}
	
	void setupSpectrumFrame() {
		spectrumFrame=new SpectrumViewer("Color Spectrum",20);
		spectrumFrame.setName("Spectrum");
		restoreFromPrefs(spectrumFrame);
		spectrumFrame.addKeyListener(undoManager);
		spectrumFrame.getContentPane().setBackground(Color.BLACK);
		
		spectrum=spectrumFrame.getDisplay();
		spectrum.setHintPointSize(2);
		spectrum.setHintStroke(2);
		spectrum.setLayout(new BorderLayout());
		spectrumSelection=new ImageSelectionManager(spectrumFrame);
		spectrumSelection.addSelectionListener(this);
		spectrumSelection.addUndoableEditListener(this);
		spectrum.add(spectrumSelection,BorderLayout.CENTER);

		spectrumFrame.plotImages(imageData.getSpectrumImages(), imageData.getRGBImages());
	}
	
	
	JTextField colorname;
	JList colorlist;
	DefaultListModel list;
	JScrollPane colorlistscroll;
	JButton remove, clear, save, /*autoSelect,*/ load, undo, help, quit;
	JCheckBox showColors,invert,realtime;
	JFileChooser chooser;
	
	String defaultName;
	
	public void setupControlPanel (JComponent root) 
	{

		setTitle("Controls");
		setName("Controls");
		setLocation(prefs.getInt(getName()+".location.x",50),prefs.getInt(getName()+".location.y",50));
		setResizable(false);
		
		//setup margins around window
		root.setLayout(new BorderLayout());
		root.add(Box.createVerticalStrut(10),BorderLayout.NORTH);
		root.add(Box.createVerticalStrut(10),BorderLayout.SOUTH);
		root.add(Box.createHorizontalStrut(10),BorderLayout.EAST);
		root.add(Box.createHorizontalStrut(10),BorderLayout.WEST);
		JComponent newroot=new JPanel();
		root.add(newroot,BorderLayout.CENTER);
		root=newroot;
		
		//add actual controls
		root.setLayout(new BoxLayout(root,BoxLayout.Y_AXIS));

		colorname=new JTextField();
		colorname.addActionListener(this);
		colorname.getDocument().addDocumentListener(this);
		colorname.setToolTipText("Press 'enter' to add a new entry");
		root.add(colorname);
		
		list=new DefaultListModel();
		list.addListDataListener(this);
		list.addElement(new String("<add>"));
		colorlist=new JList(list);
		colorlist.setFixedCellWidth(1);
		colorlist.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		colorlist.addListSelectionListener(this);
		colorlist.setToolTipText("Select color for editing, click <add> and type name (then press 'enter') to add a new entry");
		
		colorlistscroll=new JScrollPane(colorlist, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
		root.add(colorlistscroll);
		
		addComponentListener(this);
		
		chooser = new JFileChooser();
		chooser.setSelectedFile(new File("default"));
		
		root.add(Box.createVerticalStrut(10));
		String[] spacenames={"YUV","HSB","rg","xy","Lab"};
		spacelist=new JComboBox(spacenames);
		for(int i=0; i<spacelist.getItemCount(); i++) {
			if(spacelist.getItemAt(i).toString().equalsIgnoreCase(space)) {
				spacelist.setSelectedIndex(i);
				break;
			}
		}
		space=(String)spacelist.getSelectedItem();
		spacelist.addActionListener(this);
		root.add(spacelist);
		root.add(Box.createVerticalStrut(10));
		
		addButtons(root);
		addKeyListener(undoManager);
		pack();
		addComponentListener(this);
	}
	
	public void addButtons(JComponent root)
	{
		//Adding Buttons
		
		undo=new JButton("undo");
		undo.addActionListener(this);
		undo.setEnabled(false);
		undo.setToolTipText("Undoes the last action, or press Ctl-Z in an editor window (Shift-Ctl-Z to redo)");
		root.add(undo);
		
		remove=new JButton("remove");
		remove.addActionListener(this);
		remove.setToolTipText("Removes the selected color from the list");
		root.add(remove);
		
		clear=new JButton("clear");
		clear.addActionListener(this);
		clear.setToolTipText("Clears the current color selection, or press Ctl-D in an editor window (Ctl-A to select all)");
		root.add(clear);
		
		invert=new JCheckBox("invert");
		invert.addActionListener(this);
		invert.setToolTipText("Inverts the background of the color space window");
		root.add(invert);
		
		realtime=new JCheckBox("Realtime");
		realtime.addActionListener(this);
		realtime.setToolTipText("Causes selection updates to be processed while selection is in progress");
		root.add(realtime);
		
		showColors=new JCheckBox("all pixels");
		showColors.addActionListener(this);
		showColors.setToolTipText("Causes the full color spectrum to be displayed, regardless of what is selected in the image viewer");
		root.add(showColors);
		showColors.setSelected(false);
		
		//autoSelect=new JButton("auto select");
		//autoSelect.addActionListener(this);
		//root.add(autoSelect);
		
		help=new JButton("help");
		help.addActionListener(this);
		help.setToolTipText("Brings up the help.txt file");
		root.add(help);
		
		
		save=new JButton("save");
		save.addActionListener(this);
		save.setToolTipText("Saves the current color and image selections, exports a theshold (.tm) and color definition (.col) file");
		root.add(save);
		
		load =new JButton("load");
		load.addActionListener(this);
		load.setToolTipText("Loads previously saved selections");
		root.add(load);
		
		quit =new JButton("quit");
		quit.addActionListener(this);
		root.add(quit);
		
		
	}
	
	public void propertyChange(PropertyChangeEvent evt) {
		if(evt.getPropertyName().equals(ThumbnailShow.CURIMG_PROPERTY)) {
			int i=((Integer)evt.getNewValue()).intValue();
			rgbImageShow.setImage(imageData.getRGBImage(i));
			segmentedImageShow.setImage(imageData.getYUVImage(i),imageData.getWidth(i),imageData.getHeight(i));
			hc.setImage(imageData.getSpectrumImage(i),imageData.getWidth(i),imageData.getHeight(i));
			int curcolor=colorlist.getSelectedIndex();
			if(curcolor>=0) {
				rgbSelection.removeSelectionListener(this);
				rgbSelection.removeUndoableEditListener(this);
				rgbSelection.setSelectedArea(((Area[])imageAreas.get(curcolor))[i]);
				rgbSelection.addSelectionListener(this);
				rgbSelection.addUndoableEditListener(this);
			}
		}
	}
	
	public void actionPerformed(ActionEvent e)
	{
		if (e.getSource()==save) {
			int returnval=chooser.showSaveDialog(save);
			if (returnval==JFileChooser.APPROVE_OPTION) {
				saveFiles(chooser.getSelectedFile());
				chooser.setSelectedFile(chooser.getSelectedFile());
			}
		}
		else if (e.getSource()==load) {
			chooser.setFileFilter(new javax.swing.filechooser.FileFilter() {
				public boolean accept(File f) { return f.isDirectory() || f.getName().endsWith(specExt); }
				public String getDescription() { return "*"+specExt; }
			});
			int returnval=chooser.showOpenDialog(load);
			if (returnval==JFileChooser.APPROVE_OPTION) {
				loadFiles(chooser.getSelectedFile());
				chooser.setSelectedFile(chooser.getSelectedFile());
			}
		}
		else if (e.getSource()==spacelist)
		{
			String newspace=(String)spacelist.getSelectedItem();
			if(newspace.equalsIgnoreCase(space))
				return;
			
			boolean hasArea=false;
			for(int i=0; i<colorAreas.size(); i++) {
				if(colorAreas.get(i)!=null && !((Area)colorAreas.get(i)).isEmpty()) {
					hasArea=true;
					break;
				}
			}
			int s = !hasArea ? JOptionPane.OK_OPTION : JOptionPane.showConfirmDialog(this,"Changing color space will require clearing your color selections.  This cannot be undone.","Clear Color Selections?",JOptionPane.OK_CANCEL_OPTION);
			if(s==JOptionPane.OK_OPTION) {
				setSpace(newspace);
				for(int i=0; i<colorAreas.size(); i++)
					((Area)colorAreas.get(i)).reset();
				imageData.changeSpectrum(YUVtoSpectrum);
				segmentedImageShow.getThresh().changeSpectrum(YUVtoSpectrum);
				spectrumSelection.clear();
				spectrumFrame.plotImages(imageData.getSpectrumImages(),imageData.getRGBImages());
				int curcolor=colorlist.getSelectedIndex();
				if(curcolor>=0 && curcolor<list.size()) {
					Area[] ims=(Area[])imageAreas.get(curcolor);
					spectrumFrame.plotImageAreas(imageData.getSpectrumImages(),imageData.getRGBImages(),ims);
				}
				if(showColors.isSelected()) {
					spectrumFrame.showFullPlot();
				} else {
					spectrumFrame.showCurPlot();
				}
				undoManager.discardAllEdits();
			} else {
				spacelist.setSelectedItem(space);
			}
		}
		else if (e.getSource()==clear) 
		{
			spectrumSelection.clear();
		} 
		else if (e.getSource()==invert) 
		{
			if(invert.isSelected()) {
				spectrumFrame.getContentPane().setBackground(Color.WHITE);
			} else {
				spectrumFrame.getContentPane().setBackground(Color.BLACK);
			}
			spectrumSelection.invertColors();
		}
		else if (e.getSource()==realtime)
		{}
		else if (e.getSource()==showColors) 
		{
			if(showColors.isSelected()) {
				spectrumFrame.showFullPlot();
			} else {
				spectrumFrame.showCurPlot();
			}
		}
		/*else if (e.getSource()==autoSelect) 
		{
		 	//spectrum.autoSelect();
		}*/
		else if (e.getSource()==undo) 
		{
			if(undoManager.canUndo())
				undoManager.undo();
		} 
		else if (e.getSource()==help) 
		{
			if(helpBox.isVisible()) {
				helpBox.toFront();
			} else {
				helpBox.setVisible(true);
			}
		} 
		else if (e.getSource()==quit) 
		{
		 	System.exit(0);
		} 
		else if (e.getSource()==remove) 
		{
			int curcolor=colorlist.getSelectedIndex();
			if(curcolor>=0 && curcolor<list.size()-1)
				list.remove(curcolor);
			colorlist.getSelectionModel().clearSelection();
			segmentedImageShow.updateMap(colorAreas);
		} 
		else if (e.getSource()==colorname) 
		{
			String s=e.getActionCommand();
			if (!s.equals("") && !(s.startsWith("<") && s.endsWith(">"))) {
				int i;
				for (i=0; i<list.getSize() && !list.get(i).equals(s); i++) {}
				
				if (i!=list.getSize()) {
					colorname.setToolTipText("That name is already taken");
				} else {
					int curcolor=colorlist.getSelectedIndex();
					if(curcolor<0) {
						list.add(list.size()-1,s);
						colorname.setToolTipText("Type a name and press 'enter' to add a new entry");
					} else {
						undoListeners.beginUpdate();
						list.setElementAt(s,curcolor);
						if(curcolor==list.size()-1)
							list.addElement("<add>");
						undoListeners.endUpdate();
						colorname.setToolTipText("Edit text to rename current entry");
					}
				}
			}
		}
		else
		{
			System.out.println("Unknown action performed: "+e);
			System.out.println("Source is: "+e.getSource());
		}
	}
	
	int lastcurcolor=-1;
	public void valueChanged(ListSelectionEvent e) 
	{
		int curcolor=colorlist.getSelectedIndex();
		if(lastcurcolor==curcolor && e!=null) //if e is null, it's a request for refresh
			return;
		if(!getIgnoreUndoRedo() && e!=null)
			undoListeners.postEdit(new ListSelectionEdit(this,lastcurcolor,curcolor));
		if(curcolor<0 || curcolor>=list.size() || curcolor>=imageAreas.size()) {
			lastcurcolor=-1;
			remove.setEnabled(false);
			colorname.setText("");
			rgbSelection.removeSelectionListener(this);
			rgbSelection.removeUndoableEditListener(this);
			spectrumSelection.removeSelectionListener(this);
			spectrumSelection.removeUndoableEditListener(this);
			rgbSelection.clear();
			spectrumSelection.clear();
			rgbSelection.addSelectionListener(this);
			rgbSelection.addUndoableEditListener(this);
			spectrumSelection.addSelectionListener(this);
			spectrumSelection.addUndoableEditListener(this);
			colorname.setToolTipText("Type a name and press 'enter' to add a new entry");
		} else {
			lastcurcolor=curcolor;
			remove.setEnabled(curcolor<list.size()-1); //only enable remove if the "<new>" option isn't selected
			Area[] ims=(Area[])imageAreas.get(curcolor);
			int curimg=thumb.getCurrentImage();
			rgbSelection.removeSelectionListener(this);
			rgbSelection.removeUndoableEditListener(this);
			spectrumSelection.removeSelectionListener(this);
			spectrumSelection.removeUndoableEditListener(this);
			rgbSelection.setSelectedArea(ims[curimg]);
			spectrumSelection.setSelectedArea((Area)colorAreas.get(curcolor));
			rgbSelection.addSelectionListener(this);
			rgbSelection.addUndoableEditListener(this);
			spectrumSelection.addSelectionListener(this);
			spectrumSelection.addUndoableEditListener(this);
			if(!showColors.isSelected())
				spectrumFrame.plotImageAreas(imageData.getSpectrumImages(),imageData.getRGBImages(),ims);
			if(curcolor==list.size()-1) {
				colorname.setText("<enter name>");
				colorname.setToolTipText("Type a name and press 'enter' to add a new entry");
			} else {
				colorname.setText((String)list.get(curcolor));
				colorname.setToolTipText("Edit text to rename current entry");
			}
			colorname.select(0,colorname.getText().length());
			colorname.requestFocusInWindow();
		}
	}
	
	public void selectionInProgress(ImageSelectionManager src) {
		if(realtime.isSelected())
			doSelectionUpdate(src,src.getSelectionInProgress());
	}
	public void selectionChanged(ImageSelectionManager src) {
		doSelectionUpdate(src,src.getSelectedArea());
	}
	void doSelectionUpdate(ImageSelectionManager src, Area selection) {
		int curcolor=colorlist.getSelectedIndex();
		if(curcolor<0) {
			curcolor=list.size()-1;
			colorlist.setSelectedIndex(curcolor);
		}
		if(src==spectrumSelection) {
			colorAreas.setElementAt(selection,curcolor);
			segmentedImageShow.updateMap(selection,(short)curcolor);
		} else if(src==rgbSelection) {
			int curimg=thumb.getCurrentImage();
			Area[] a=(Area[])imageAreas.get(curcolor);
			a[curimg]=selection;
			spectrumFrame.updateImageArea(imageData.getSpectrumImage(curimg),imageData.getRGBImage(curimg),a[curimg],curimg);
		}
		//undo.setEnabled(true);
	}

	public void restoreFromPrefs(Container c) {
		String name=c.getName();
		int w=prefs.getInt(name+".size.width",100);
		int h=prefs.getInt(name+".size.height",100);
		w+=c.getInsets().left+c.getInsets().right;
		h+=c.getInsets().top+c.getInsets().bottom;
		c.setSize(new Dimension(w,h));
		c.setLocation(prefs.getInt(c.getName()+".location.x",50),prefs.getInt(c.getName()+".location.y",50));
	}

	public void componentResized(ComponentEvent e) 
	{
		Container c=(Container)e.getComponent();
		String name=c.getName();
		//System.out.println(name+" size: "+c.getSize());
		//System.out.println(name+" insets: "+c.getInsets());
		int w=c.getSize().width-c.getInsets().left-c.getInsets().right;
		int h=c.getSize().height-c.getInsets().top-c.getInsets().bottom;
		prefs.putInt(name+".size.width",w);
		prefs.putInt(name+".size.height",h);
	}
	public void componentHidden(ComponentEvent e) {}
	
	public void componentMoved(ComponentEvent e)
	{ 	
		prefs.putInt(e.getComponent().getName()+".location.x",e.getComponent().getLocation().x);
		prefs.putInt(e.getComponent().getName()+".location.y",e.getComponent().getLocation().y);
	}
	public void componentShown(ComponentEvent e) { }

	public void setDefaultPrefs()
	{
		prefs.putInt("Spectrum.location.x",10);
		prefs.putInt("Spectrum.location.y",10);
		prefs.putInt("Spectrum.size.width",700);
		prefs.putInt("Spectrum.size.height",700);

		prefs.putInt("Controls.location.x",780);
		prefs.putInt("Controls.location.y",50);

		prefs.putInt("HelpBox.location.x",100);
		prefs.putInt("HelpBox.location.y",100);

		prefs.putInt("RGBImageShow.location.x",50);
		prefs.putInt("RGBImageShow.location.y",200);

		prefs.putInt("SegmentedImage.location.x",400);
		prefs.putInt("SegmentedImage.location.y",200);
	}  
  
	Vector colornames=new Vector(); //to be used only as a backing so we know the names after they are removed for undo functionality
	public void intervalAdded(ListDataEvent e) {
		for(int i=e.getIndex0(); i<=e.getIndex1(); i++) {
			imageAreas.add(i,new Area[imageData.getNumImages()]);
			colorAreas.add(i,new Area());
			colornames.add(i,list.get(i));
			if(!getIgnoreUndoRedo())
				undoListeners.postEdit(new AddColorEdit(this,(String)list.get(i),i));
			//undo.setEnabled(true);
		}
		if(colorlist!=null) {
			int curcolor=colorlist.getSelectedIndex();
			remove.setEnabled(curcolor>=0 && curcolor<list.size()-1);
		}
	}
	public void intervalRemoved(ListDataEvent e) {
		if(!getIgnoreUndoRedo())
			for(int i=e.getIndex0(); i<=e.getIndex1(); i++)
				undoListeners.postEdit(new RemoveColorEdit(this,(String)colornames.get(i),i,(Area)colorAreas.get(i),(Area[])imageAreas.get(i)));
		//undo.setEnabled(true);
		for(int i=e.getIndex0(); i<=e.getIndex1(); i++) {
			imageAreas.removeElementAt(e.getIndex0());
			colorAreas.removeElementAt(e.getIndex0());
			colornames.removeElementAt(e.getIndex0());
		}
		int curcolor=colorlist.getSelectedIndex();
		remove.setEnabled(curcolor>0 && curcolor<list.size()-1);
	}
	public void contentsChanged(ListDataEvent e) {
		if(e.getIndex0()!=e.getIndex1())
			System.out.println("WARNING: Unhandled multi-change");
		else {
			//item renamed/replaced
			int index=e.getIndex0();
			if(!getIgnoreUndoRedo())
				undoListeners.postEdit(new RenameColorEdit(this,(String)colornames.get(index),(String)list.get(index),index));
			colornames.setElementAt(list.get(index),index);
		}
	}

	public void changedUpdate(DocumentEvent e) {
		if(e.getDocument()==colorname.getDocument()) {
			int curcolor=colorlist.getSelectedIndex();
			if(curcolor>=0 && curcolor<list.getSize()-1) {
				actionPerformed(new ActionEvent(colorname,0,colorname.getText()));
			}
		}
	}
	public void insertUpdate(DocumentEvent e) { changedUpdate(e); }
	public void removeUpdate(DocumentEvent e) { changedUpdate(e); }
	
	public void loadFiles(File f) {
		String basename;
		if(f.getName().endsWith(".tm") || f.getName().endsWith(".col")) {
			String path=f.getPath();
			f=new File(path.substring(0,path.lastIndexOf('.'))+specExt);
			basename=f.getName().substring(0,f.getName().lastIndexOf('.'));
		} if(f.getName().endsWith(specExt)) {
			basename=f.getName().substring(0,f.getName().lastIndexOf('.'));
		} else {
			basename=f.getName();
			f=new File(f.getPath()+specExt);
		}
		if(!f.canRead()) {
			JOptionPane.showMessageDialog(this,"The file "+f+" could not be opened","File Access Denied",JOptionPane.ERROR_MESSAGE);
			return;
		}
		setIgnoreUndoRedo(true);
		
		//read basic color info and color areas
		try {
			BufferedReader load=new BufferedReader(new FileReader(f));
			
			colorlist.getSelectionModel().clearSelection();
			if(list.size()>1)
				list.removeRange(0,list.size()-2);
			((Area)colorAreas.get(0)).reset();
			spacelist.setSelectedItem(load.readLine());
			int numColors = Integer.parseInt(load.readLine());
			for(int i=0; i<numColors; i++) {
				String name=load.readLine();
				list.add(list.size()-1,name); //the ListDataListener should take care of managing colorAreas, imageAreas, etc.
				colorAreas.setElementAt(loadArea(load),i);
			}
			load.close();
		} catch (Exception ex) {
			System.out.println("Error loading from '"+f.getPath() + "': "+ex);
			JOptionPane.showMessageDialog(this,"The file "+f+" could not be loaded","File Access Denied",JOptionPane.ERROR_MESSAGE);
		}
		
		// load image areas where found
		String[] files=imageData.getFileNames();
		for(int i=0; i<files.length; i++) {
			f=getImageAreaFile(basename,files[i]);
			if(!f.exists())
				continue;
			try {
				BufferedReader load=new BufferedReader(new FileReader(f));
				for(String color=load.readLine(); color!=null; color=load.readLine()) {
					int idx=list.indexOf(color);
					Area a=loadArea(load);
					if(idx==-1) {
						System.out.println("Warning: ignoring area for color '"+color+"', found in '"+f.getPath()+"' -- was not previously defined in '"+basename+specExt+"'");
					} else {
						Area[] areas=(Area[])imageAreas.get(idx);
						areas[i]=a;
					}
				}
				load.close();
			} catch (Exception ex) {
				System.out.println("Error loading from '"+f.getPath() + "': " + ex);
				JOptionPane.showMessageDialog(this,"The file "+f+" could not be loaded","File Access Denied",JOptionPane.ERROR_MESSAGE);
			}
		}
		
		segmentedImageShow.updateMap(colorAreas);
		//if(!showColors.isSelected())
		//spectrumFrame.plotImageAreas(imageData.getSpectrumImages(),imageData.getRGBImages(),(Area[])imageAreas.get(0));
		
		colorlist.setSelectedIndex(0);
		setIgnoreUndoRedo(false);
		undoManager.discardAllEdits();
	}
	
	public void saveFiles(File f) {
		ThresholdMap thresh=segmentedImageShow.getThresh();
		if(f.getName().endsWith(specExt) || f.getName().endsWith(".tm") || f.getName().endsWith(".col"))
			f=new File(f.getPath().substring(0,f.getPath().lastIndexOf('.')));

		// Saving threshold map
		try {
			thresh.saveFile(new File(f.getPath()+".tm"));
		} catch (Exception ex) {
			System.out.println("Error saving to '"+f.getPath() + ".tm': " + ex);
		}
		
		// Saving color definition file
		try {
			FileWriter file_col_fw=new FileWriter(f.getPath() + ".col");
			file_col_fw.write("0 (128 128 128) \"unclassified\" 8 1.00\n");
			for (int i=0; i<list.size()-1; i++) {
				Color c=thresh.getRepresentitiveColor(i);
				file_col_fw.write((i+1)+" (" + c.getRed() + " " + c.getGreen() + " " + c.getBlue() + ") " +"\"" + list.get(i)+ "\" 8 0.75\n");
			}
			file_col_fw.close();
 		} catch (Exception ex) {
			System.out.println("Error saving to '"+f.getPath() + ".col': " + ex);
		}
		
		// Saving colorAreas
		try {
			FileWriter save=new FileWriter(f.getPath() + specExt);
			
			save.write(space+"\n");
			save.write((list.size()-1) + "\n");
			for(int i=0; i<list.size()-1; i++) {
				save.write(list.get(i) + "\n");   
				
				Area a = (Area)colorAreas.get(i);
				String pathString = createPathString(a.getPathIterator(null));
				save.write(pathString);  
			}
			save.close();
		} catch (Exception ex) {
			System.out.println("Error saving to '"+f.getPath() + specExt+"': " + ex);
			JOptionPane.showMessageDialog(this,"The file "+f+" could not be saved","File Access Denied",JOptionPane.ERROR_MESSAGE);
		}
		
		// Saving imageAreas
		String[] files=imageData.getFileNames();
		for(int i=0; i<files.length; i++) {
			//first check if there are even any areas to save
			boolean hasNonNull=false;
			for(int j=0; j<list.size()-1 && !hasNonNull; j++) {
				Area[] a = (Area[])imageAreas.get(j);
				hasNonNull=(a[i]!=null);
			}
			
			//only create a file if there's something to go in it!
			if(hasNonNull) {
				try {
					FileWriter save=new FileWriter(getImageAreaFile(f.getName(),files[i]));
					for (int j=0; j<list.size()-1; j++) {
						Area[] a = (Area[])imageAreas.get(j);
						if(a[i]!=null) {
							save.write(list.get(j) + "\n");
							String pathString = createPathString(a[i].getPathIterator(null));
							save.write(pathString);  
						}
					}
					save.close();
				} catch (Exception ex) {
					System.out.println("Error saving to '"+files[i] +"-"+f.getName()+".areas': " + ex);
					JOptionPane.showMessageDialog(this,"The file '"+files[i] +"-"+f.getName()+".areas' could not be saved","File Access Denied",JOptionPane.ERROR_MESSAGE);
				}
			}
		}                 
	}
	
	String createPathString(PathIterator path)
	{
		
		String retPath = "<AREA>\n";
		
		double[] coords = new double[6];
		int type;       
		
		while(!path.isDone())
		{
			switch(path.currentSegment(coords)) {
				case PathIterator.SEG_MOVETO: //new polygon
					retPath += "START " + coords[0] + " " + coords[1] + "\n";
					break;
				case PathIterator.SEG_LINETO: //line 
					retPath += "LINE " + coords[0] + " " + coords[1] + "\n";
					break;
				case PathIterator.SEG_CLOSE: //end of curr polygon
					retPath += "CLOSE\n";
					break;
			}
			path.next();
		}
		retPath += "</AREA>\n";
		return retPath;
	}
	
	Area loadArea(BufferedReader load) throws java.io.IOException {
		if(!load.readLine().equals("<AREA>")) {
			System.out.println("Error: File Format is incorrect");
			return null;
		}
		GeneralPath curPath=null;
		Area curArea=new Area();
		for(String in = load.readLine(); !in.equals("</AREA>"); in = load.readLine()) {
			StringTokenizer inTok = new StringTokenizer(in);
			String t = inTok.nextToken();
			if(t.equals("START")) {
				curPath = new GeneralPath(GeneralPath.WIND_NON_ZERO);
				double x = Double.parseDouble(inTok.nextToken());
				double y = Double.parseDouble(inTok.nextToken());
				curPath.moveTo((float)x,(float)y);
			} else if(t.equals("LINE")) {
				double x = Double.parseDouble(inTok.nextToken());
				double y = Double.parseDouble(inTok.nextToken());
				curPath.lineTo((float)x,(float)y);
			} else if(t.equals("CLOSE")) {
				curPath.closePath();
				curArea.add(new Area(curPath));
				curPath=null;
			}
		}
		if(curPath!=null)
			System.out.println("Warning: unclosed polygon during load");
		return curArea;
	}
	
	File getImageAreaFile(String basename, String imgFile) {
		File f=new File(imgFile.substring(0,imgFile.lastIndexOf('.'))+"-"+basename+".areas");
		//System.out.println("Checking "+f);
		return f;
	}
}
