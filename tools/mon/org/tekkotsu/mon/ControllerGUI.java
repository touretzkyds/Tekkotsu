package org.tekkotsu.mon;


import java.awt.*;
import java.awt.font.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.beans.*;
import java.util.*;
import java.util.prefs.Preferences;
import java.io.PrintWriter;
import java.io.FileWriter;
import javax.swing.*;
import javax.swing.plaf.*;
import javax.swing.plaf.basic.BasicToolTipUI;
import javax.swing.text.*;
//Admittedly a little sloppy, but I don't want to spend forever on this...

public class ControllerGUI extends JFrame implements ActionListener, KeyListener, EStopListener.UpdatedListener {
	public class MyMenu extends JList {
		public MyMenu() { super(); }
		// use custom tooltip to support multiple lines
		public JToolTip createToolTip() { return new JMultiLineToolTip(); }
	}

	public org.tekkotsu.sketch.SketchGUI cameraSkGUI=null, localSkGUI=null, worldSkGUI=null;
	public ColorSlider accessibility=null;
	MyMenu menu;
	JScrollPane menuScroll;
	JComboBox title;
	public class MyLabel extends JLabel {
		public MyLabel(String s) { super(s); }
		// use custom tooltip to support multiple lines
		public JToolTip createToolTip() { return new JMultiLineToolTip(); }
	}

	MyLabel status;
	JList scripts;
	ControllerListener comm;
	EStopListener estopComm;
	GamepadListener gamepadcomm;
	JButton backBut;
	JButton refreshBut;
	JButton reconnectBut;
	EStopPanel estop;
	JButton estopBut;
	JTextField inputField;
	boolean isUpdating=false;
	DefaultListModel scriptsModel=new DefaultListModel();
	Vector inputFieldHistory;
	int inputFieldLocation;
	static Preferences prefs = Preferences.userNodeForPackage(ControllerGUI.class);
	final static int MAX_STORE_INPUT_HIST=20;

	ControllerGUI(String addr, int port, boolean gamepadEnabled) {
		super();
		String title = "TekkotsuMon" +
		    (gamepadEnabled ? "with Gamepad:" : "") + " Controller";
		setTitle(title + " (" + addr + ")");
        comm = new ControllerListener(addr,port);
	if ( gamepadEnabled ) {
	    gamepadcomm = new GamepadListener(addr,GamepadListener.defPort);
	    gamepadcomm.needConnection();
	} else {
	    gamepadcomm = null;
	}
	init();
	comm.gui = this;
	comm.needConnection();
	}

	// Disables the component if there's nothing selected to apply it to
	public class AutoDisableListener implements PropertyChangeListener, ListSelectionListener {
		JComponent target;
		public AutoDisableListener(JComponent target, JList source) {
			this.target=target;
			source.addPropertyChangeListener(this);
			source.addListSelectionListener(this);
		}
		public void propertyChange(PropertyChangeEvent evt) {
			int min=((JList)evt.getSource()).getMinSelectionIndex();
			int max=((JList)evt.getSource()).getMaxSelectionIndex();
			//target.setEnabled(!((JList)evt.getSource()).isSelectionEmpty());
			target.setEnabled(min!=-1 && max-min==0);
		}
		public void valueChanged(ListSelectionEvent evt) {
			int min=((JList)evt.getSource()).getMinSelectionIndex();
			int max=((JList)evt.getSource()).getMaxSelectionIndex();
			//target.setEnabled(!((JList)evt.getSource()).isSelectionEmpty());
			target.setEnabled(min!=-1 && max-min==0);
		}
	}

	final static ImageIcon rarrow = new ImageIcon("images/rarrow.gif");
	final static ImageIcon larrow = new ImageIcon("images/larrow.gif");
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.gif");

	public class MyCellRenderer implements ListCellRenderer {
		public java.awt.Component getListCellRendererComponent(JList list,Object value,int index,boolean isSelected,boolean cellHasFocus) {
			ControllerListener.MenuEntry me=(ControllerListener.MenuEntry)value;
			/*			if(me.hasSubmenu) {
						setIcon(rarrow);
						setHorizontalTextPosition(SwingConstants.RIGHT);
						setIconTextGap(5);
						} else
						setIconTextGap(5+rarrow.getIconWidth());
			*/
			//System.out.println(this.getText()+" "+me.selected+" "+isSelected);
			int numpre=(int)(Math.log(list.getModel().getSize()-1)/Math.log(10))-(index==0?0:(int)(Math.log(index)/Math.log(10)));
			StringBuffer pre=new StringBuffer();
			for(int i=0;i<numpre;i++)
				pre.append(" ");
			JLabel title=new JLabel(pre.toString()+index+". "+me.title);
			title.setBackground(me.selected?list.getSelectionBackground():list.getBackground());
			if(me.title.length()>0 && me.title.charAt(0)=='#')
				title.setForeground(new Color(2f/3,0f,0f));
			else
				title.setForeground(me.selected?list.getSelectionForeground():list.getForeground());
			title.setEnabled(list.isEnabled());
			title.setFont(list.getFont());
			title.setOpaque(true);
			//I would rather use title.getPreferredSize().height instead of the hard-coded 15
			//but sometimes values around 16390 are returned, and sometimes 0 -- doesn't make sense
			title.setPreferredSize(new Dimension(list.getSize().width,15));
			//Vector tmp2=new Vector();
			//for(int i=0;i<1000000;i++) {tmp2.add(new Integer(0));tmp2.remove(tmp2.size()-1);}
			if(cellHasFocus)
				title.setBorder(BorderFactory.createLineBorder(Color.GRAY,1));
			else
				title.setBorder(BorderFactory.createEmptyBorder(1,1,1,1));
			JComponent tmp=Box.createHorizontalBox();
			if(me.hasSubmenu) {
				tmp.add(title);
				//				tmp.add(Box.createHorizontalGlue());
				JLabel arr=new JLabel(rarrow);
				arr.setEnabled(list.isEnabled());
				tmp.add(arr);
				if(me.description.length()!=0)
					tmp.setToolTipText(me.description);
				return tmp;
			} else {
				tmp.add(title);
				tmp.add(Box.createHorizontalStrut(rarrow.getIconWidth()));
				if(me.description.length()!=0)
					tmp.setToolTipText(me.description);
				return tmp;
			}
		}
	}

	public class JListDoubleClick extends MouseAdapter {
		ControllerGUI gui;
		JListDoubleClick(ControllerGUI gui) { this.gui=gui; }
		public void mouseClicked(MouseEvent e) {
			if (e.getClickCount() == 2) {
				int index = ((JList)e.getSource()).locationToIndex(e.getPoint());
				gui.doubleClicked((JList)e.getSource(),index);
			}
		}
	}

	public class MySelectionListener implements ListSelectionListener {
		ControllerGUI gui;
		MySelectionListener(ControllerGUI gui) { this.gui=gui; }
		public void valueChanged(ListSelectionEvent e) {
			if(!gui.isUpdating && !e.getValueIsAdjusting())
				gui.comm.sendSelection(((JList)e.getSource()).getSelectedIndices());
		}
	}

	public class EscFilter extends KeyAdapter {
		JButton trigger;
		EscFilter(JButton trigger) { this.trigger=trigger; }
		public void keyReleased(KeyEvent evt) {
			if(evt.getKeyCode()==KeyEvent.VK_ESCAPE)
				trigger.doClick();
		}
	}
	public class DelFilter extends KeyAdapter {
		ControllerGUI gui;
		DelFilter(ControllerGUI gui) { this.gui=gui; }
		public void keyReleased(KeyEvent evt) {
			if(evt.getKeyCode()==KeyEvent.VK_BACK_SPACE || evt.getKeyCode()==KeyEvent.VK_DELETE)
				gui.actionPerformed(new ActionEvent(evt.getSource(),0,"delbookmark"));
		}
	}
	public class ReturnFilter extends KeyAdapter {
		ControllerGUI gui;
		ReturnFilter(ControllerGUI gui) { this.gui=gui; }
		public void keyReleased(KeyEvent evt) {
			if(evt.getKeyCode()==KeyEvent.VK_ENTER)
				gui.doubleClicked((JList)evt.getSource(),0);
		}
	}

	void gotConnection() {
		updated();
		repaint();
	}

	void lostConnection() {
		updated();
		repaint();
	}

	void updated() {
			//		System.out.println("updated");
		isUpdating=true;
		Point p=menuScroll.getViewport().getViewPosition();

		//scripts.setEnabled(comm._isConnected);
		title.setEnabled(comm._isConnected);
		backBut.setEnabled(comm._isConnected);
		estopBut.setEnabled(comm._isConnected);
		refreshBut.setEnabled(comm._isConnected);
		inputField.setEnabled(comm._isConnected);
		if(!comm._isConnected) {
			synchronized(menuScroll) {
				title.removeAllItems();
				title.addItem("-");
				menu.setListData(new Vector());
				status.setText("Reconnecting...");
				status.setToolTipText("");
			}
		} else {
			if(comm._connectCount>0) {
				if(comm._connectCount==1) {
					for(int i=0; i<scriptsModel.getSize(); i++)
						if(scriptsModel.get(i).toString().equals("STARTUP"))
							comm.sendInput(((ScriptEntry)scriptsModel.get(i)).cmd);
					//restart servers for any windows we still have open
					Iterator mons=comm.dynObjSrcs.values().iterator();
					LinkedList started=new LinkedList(); //only start each entry once (may open multiple items)
					while(mons.hasNext()) {
						String c=(String)mons.next();
						if(!started.contains(c)) {
							comm.sendInput("!root "+c);
							started.add(c);
						}
					}
					//Set focus to the main menu
					//menu.requestFocus();
				}
				for(int i=0; i<scriptsModel.getSize(); i++)
					if(scriptsModel.get(i).toString().equals("CONNECT"))
						comm.sendInput(((ScriptEntry)scriptsModel.get(i)).cmd);
				comm._connectCount=-1; //so we don't get it again
			}

			Vector menuitems;
			synchronized(comm._menus) {
				boolean titleDirty=false;
				int index=comm._titles.size()-1;
				for(Iterator it=comm._titles.iterator(); index>=0 && it.hasNext();index--) {
					String entry=(String)it.next();
					if(!entry.equals((String)title.getItemAt(index))) {
						titleDirty=true;
						break;
					}
				}
				if(titleDirty) {
					title.removeAllItems();//clearing the title popup menu, but we'll refresh the items if we're still connected
					int len=0;
					for(Iterator it=comm._titles.iterator(); it.hasNext();) {
						String entry=(String)it.next();
						title.insertItemAt(entry,0);
						if(entry.length()>len)
							len=entry.length();
					}
					title.setMaximumSize(new Dimension((int)(len*8)+25,title.getHeight()));
					title.setSelectedIndex(0);
				}
				menuitems=(Vector)comm._menus.lastElement();
				menuitems=(Vector)menuitems.clone();
				updateStatusText();
			}

			//synchronized(this) {
				menu.setValueIsAdjusting(true);
				menu.setListData(menuitems);
				Vector sels=new Vector();
				for(int i=0; i<menuitems.size(); i++)
					if(((ControllerListener.MenuEntry)menuitems.get(i)).selected)
						sels.add(new Integer(i));
				int[] selsArr=new int[sels.size()];
				for(int i=0; i<selsArr.length; i++)
					selsArr[i]=((Integer)sels.get(i)).intValue();
				menu.setSelectedIndices(selsArr);
				menu.setValueIsAdjusting(false);
			//}
		}
		menuScroll.getViewport().setViewPosition(p);
		isUpdating=false;
		comm._updatedFlag=false;
	}

	/*public void paint(Graphics g) {
		if(comm!=null && comm._updatedFlag)
			updated();
		super.paint(g);
		}*/

	public void estopUpdated(EStopListener l) {
		updateStatusText();
		if(estopBut!=null)
			estopBut.setText(estopComm.getEStop()?"Un-Stop":"Stop!");
	}

	public void updateStatusText() {
		if(comm._status.length()>0) {
			status.setText(comm._status);
		} else {
			if(status!=null && estopComm!=null) //otherwise still doing initial connect, leave at default text
				status.setText(estopComm.getEStop()?"Stopped":"Running");
		}
		if(status!=null)
			status.setToolTipText(comm._status);
	}

	public void actionPerformed(ActionEvent evt) {
		if(evt.getSource()==backBut) {
			//System.out.println("back button clicked");
			comm.sendReturn();
		} else if(evt.getSource()==estopBut) {
			estopComm.toggleEStop();
		} else if(evt.getSource()==refreshBut) {
			//System.out.println("refresh button clicked");
			comm.sendRefresh();
		} else if(evt.getSource()==title) {
			//System.out.println("title selection changed");
			int selected=title.getSelectedIndex();
			for(int i=0; i<selected; i++)
				comm.sendReturn();
		} else if(evt.getSource()==inputField) {
			//System.out.println("input: "+inputField.getText());
			int[] sel=menu.getSelectedIndices();
			if (inputField.getText().length()==0) {
				return;
			} else if(sel.length==0 || inputField.getText().charAt(0)=='!')
				comm.sendInput(inputField.getText());
			else
				comm.sendInput("!input "+inputField.getText());
			/*else if(sel.length==1) {
				comm.sendSelectionPath(comm.buildSelectionPath(sel[0]));
				comm.sendInput(inputField.getText());
			} else
				for(int i=0; i<sel.length; i++)
					comm.sendInput(inputField.getText(),comm.buildSelectionPath(sel[i]));
			*/

			if(!inputFieldHistory.get(inputFieldHistory.size()-1).equals(inputField.getText())) {
				inputFieldHistory.add(inputField.getText());
			}
			inputFieldLocation=inputFieldHistory.size();
			inputField.setText("");
		} else if(evt.getSource()==reconnectBut) {
			int port=comm._port;
			String addr=comm._host;
			HashMap dynObjs=comm.dynObjs;
			HashMap dynObjSrcs=comm.dynObjSrcs;
			HashMap dynObjPorts=comm.dynObjPorts;
			comm.kill();
			comm = new ControllerListener(addr,port);
			comm.gui=this;
			comm.dynObjs=dynObjs;
			comm.dynObjSrcs=dynObjSrcs;
			comm.dynObjPorts=dynObjPorts;
			comm.needConnection();
			estop.close();
			estop.open();
		} else if(evt.getActionCommand().equals("CameraSketchSpace")) {
			if(cameraSkGUI==null || !cameraSkGUI.isDisplayable()) {
				org.tekkotsu.sketch.SketchGUI.Space _space = org.tekkotsu.sketch.SketchGUI.Space.cam;
				int _listingPort = 5800;
				int _sketchPort = 5801;
				cameraSkGUI = new org.tekkotsu.sketch.SketchGUI(comm._host, _listingPort, _sketchPort, _space);
				cameraSkGUI.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
				cameraSkGUI.setVisible(true);
			} else {
				cameraSkGUI.close();
				cameraSkGUI=null;
			}
		} else if(evt.getActionCommand().equals("LocalSketchSpace")) {
			if(localSkGUI==null || !localSkGUI.isDisplayable()) {
				org.tekkotsu.sketch.SketchGUI.Space _space = org.tekkotsu.sketch.SketchGUI.Space.local;
				int _listingPort = 5802;
				int _sketchPort = 5803;
				localSkGUI = new org.tekkotsu.sketch.SketchGUI(comm._host, _listingPort, _sketchPort, _space);
				localSkGUI.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
				localSkGUI.setVisible(true);
			} else {
				localSkGUI.close();
				localSkGUI=null;
			}
		} else if(evt.getActionCommand().equals("WorldSketchSpace")) {
			if(worldSkGUI==null || !worldSkGUI.isDisplayable()) {
				org.tekkotsu.sketch.SketchGUI.Space _space = org.tekkotsu.sketch.SketchGUI.Space.world;
				int _listingPort = 5804;
				int _sketchPort = 5805;
				worldSkGUI = new org.tekkotsu.sketch.SketchGUI(comm._host, _listingPort, _sketchPort, _space);
				worldSkGUI.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
				worldSkGUI.setVisible(true);
			} else {
				worldSkGUI.close();
				worldSkGUI=null;
			}
		} else if(evt.getActionCommand().equals("HeadRC") || evt.getActionCommand().equals("WalkRC") || evt.getActionCommand().equals("ArmRC")) {
			if(evt.getActionCommand().equals("HeadRC")) {
				comm.sendInput("!root \"TekkotsuMon\" \"HeadController\"");
				comm.dynObjSrcs.put("HeadPointerGUI ","\"TekkotsuMon\" \"HeadController\"");
			} else if(evt.getActionCommand().equals("WalkRC")) {
				comm.sendInput("!root \"TekkotsuMon\" \"WalkController\"");
				comm.dynObjSrcs.put("WalkGUI","\"TekkotsuMon\" \"WalkController\"");
			} else {
				comm.sendInput("!root \"TekkotsuMon\" \"ArmController\"");
				comm.dynObjSrcs.put("ArmGUI","\"TekkotsuMon\" \"ArmController\"");
			}
		} else if(evt.getActionCommand().equals("raw") || evt.getActionCommand().equals("depth") ||
			  evt.getActionCommand().equals("rle") || evt.getActionCommand().equals("reg")) {
			if(evt.getActionCommand().equals("raw")) {
				comm.sendInput("!root \"TekkotsuMon\" \"RawCam\"");
				comm.dynObjSrcs.put("RawVisionGUI","\"TekkotsuMon\" \"RawCam\"");
			} else if(evt.getActionCommand().equals("depth")) {
			        comm.sendInput("!root \"TekkotsuMon\" \"DepthCam\"");
			        comm.dynObjSrcs.put("DepthVisionGUI","\"TekkotsuMon\" \"DepthCam\"");

			} else if(evt.getActionCommand().equals("rle")) {
			        comm.sendInput("!root \"TekkotsuMon\" \"SegCam\"");
				comm.dynObjSrcs.put("SegVisionGUI","\"TekkotsuMon\" \"SegCam\"");
			} else {
				comm.sendInput("!root \"TekkotsuMon\" \"RegionCam\"");
				comm.dynObjSrcs.put("RegionVisionGUI","\"TekkotsuMon\" \"RegionCam\"");
			}

		} else if(evt.getActionCommand().equals("addbookmark")) {
			int sel=comm.firstSelected();
			if(sel==-1 || !comm._isConnected) {
				scriptsModel.addElement(new ScriptEntry("",""));
			} else {
				String title=((ControllerListener.MenuEntry)((Vector)comm._menus.lastElement()).get(menu.getMinSelectionIndex())).title;
				String command="!root";
				Vector path=comm.buildSelectionPath(sel);
				for(int i=1; i<path.size(); i++)
					command+=" \""+path.get(i)+"\"";
				scriptsModel.addElement(new ScriptEntry(title,command));
			}
			new EditScriptGUI(scriptsModel,scriptsModel.getSize()-1,true);
		} else if(evt.getActionCommand().equals("editbookmark")) {
			for(int i=0; i<scriptsModel.getSize(); i++)
				if(scripts.isSelectedIndex(i))
					new EditScriptGUI(scriptsModel,i,false);
		} else if(evt.getActionCommand().equals("delbookmark")) {
			for(int i=0; i<scriptsModel.getSize(); i++)
				if(scripts.isSelectedIndex(i))
					scriptsModel.remove(i--);
		} else if(evt.getActionCommand().equals("changecolors")) {
		    if (accessibility == null)
    		    accessibility = new ColorSlider(this);
    		else {
    		    accessibility.close();
    		    accessibility = null;
    		}
		}
	}

	public void doubleClicked(JList list, int index) {
		if(list==menu) {
			//Vector items=(Vector)comm._menus.lastElement();
			//System.out.println(items.get(index)+" selected");
			comm.sendSelect();
			/*			int[] sel=menu.getSelectedIndices();
						if(sel.length==1) {
						comm.sendSelectionPath(comm.buildSelectionPath(sel[0]));
						} else if(sel.length>1) {
						Vector selectionPaths=new Vector();
						for(int i=0; i<sel.length; i++)
						selectionPaths.add(comm.buildSelectionPath(sel[i]));
						for(int i=0; i<selectionPaths.size(); i++)
						comm.sendSelectionPath((Vector)selectionPaths.get(i));
						}
			*/
			inputField.setText("");
		} else if(list==scripts) {
			for(int i=0; i<scriptsModel.getSize(); i++)
				if(scripts.isSelectedIndex(i))
					comm.sendInput(((ScriptEntry)scriptsModel.get(i)).cmd);
		} else {
			System.out.println("Unknown list double-clicked");
		}
	}

	public class ConstSizeJButton extends JButton {
		Dimension dim;
		ConstSizeJButton(String s, ImageIcon i, Dimension d) { super(s,i); dim=d; }
		public Dimension getPreferredSize() { return dim; }
		public Dimension getMinimumSize() { return dim; }
		public Dimension getMaximumSize() { return dim; }
		public Dimension getSize() { return dim; }
	}

	public class SynchPaint extends JScrollPane {
		public Object lock=new Object();

		SynchPaint(JList l) { super(l); }
		public void paint(Graphics g) {
			synchronized(lock) {
				super.paint(g);
			}
		}
	}

	protected void init() {
	        // grabs preference settings for the color slider settings
	    	ColorConverter.visionType = prefs.getInt("ColorConverter.visionType",0);
	    	ColorConverter.setSliders(prefs.getInt("ColorConverter.sliderVals[0]",0),
	                              	  prefs.getInt("ColorConverter.sliderVals[1]",0),
	                              	  prefs.getInt("ColorConverter.sliderVals[2]",0));

		int spacer_size=10;

		getContentPane().setLayout(new BorderLayout());
		getContentPane().add(Box.createHorizontalStrut(spacer_size),BorderLayout.WEST);
		getContentPane().add(Box.createHorizontalStrut(spacer_size),BorderLayout.EAST);
		getContentPane().add(Box.createVerticalStrut(spacer_size),BorderLayout.NORTH);
		getContentPane().add(Box.createVerticalStrut(spacer_size),BorderLayout.SOUTH);

		menu=new MyMenu();
		menu.setCellRenderer(new MyCellRenderer());
		menu.setFixedCellWidth(165);
		menu.addMouseListener(new JListDoubleClick(this));
		menu.addListSelectionListener(new MySelectionListener(this));
		menu.addKeyListener(new ReturnFilter(this));

		JPanel content=new JPanel(new BorderLayout());
		Box titleBox=Box.createVerticalBox();
		title=new JComboBox();
		title.addItem("-");
		title.setAlignmentX(0);
		title.addActionListener(this);
		titleBox.add(title);
		titleBox.add(Box.createVerticalStrut(spacer_size));
		content.add(titleBox,BorderLayout.NORTH);

		Box menuBox=Box.createVerticalBox();
		menuScroll=new JScrollPane(menu);
		//menuScroll.lock=this;
		menuScroll.setAlignmentX(0.5f);
		menuBox.add(menuScroll);
		{
			Box tmp=Box.createHorizontalBox();
			backBut=new JButton("Back");
			Dimension dim=new Dimension(backBut.getPreferredSize().width+larrow.getIconWidth()+backBut.getIconTextGap(),backBut.getPreferredSize().height);
			tmp.add(backBut=new ConstSizeJButton("Back",larrow,dim));
			backBut.setEnabled(false);
			backBut.addActionListener(this);
			backBut.setToolTipText("Returns to previous control");
			tmp.add(refreshBut=new JButton("Refresh"));
			refreshBut.setEnabled(false);
			refreshBut.addActionListener(this);
			refreshBut.setToolTipText("Requests a menu refresh (handy if item names are dynamic)");
			tmp.setAlignmentX(0.5f);
			menuBox.add(tmp);
		}
		content.add(menuBox,BorderLayout.CENTER);

		Box statusbox=Box.createVerticalBox();
		statusbox.add(Box.createVerticalStrut(spacer_size));
		{
			JSeparator sep=new JSeparator(SwingConstants.HORIZONTAL);
			statusbox.add(sep);
		}
		statusbox.add(Box.createVerticalStrut(spacer_size-2));
		{
			JPanel tmp=new JPanel(new BorderLayout());
			{
				Box tmp2=Box.createHorizontalBox();
				estopComm=new EStopListener(comm._host,EStopListener.defPort);
				estopComm.addUpdatedListener(this);
				estop=new EStopPanel(estopComm,true);
				estop.setMyDim(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
				//estop.setMargin(new Insets(0,0,0,0));
				tmp2.add(estop);
				tmp2.add(Box.createHorizontalStrut(spacer_size));
				tmp.add(tmp2,BorderLayout.WEST);
			}
			status=new MyLabel("Connecting...");
			tmp.add(status,BorderLayout.CENTER);
			{
				Box tmp2=Box.createHorizontalBox();
				estopBut=new JButton("Stop!");
				estopBut.setText(estopComm.getEStop()?"Un-Stop":"Stop!");
				estopBut.addActionListener(this);
				estopBut.setToolTipText("Toggle the emergency stop status");
				estopBut.setEnabled(false);
				tmp2.add(estopBut);
				tmp2.add(Box.createHorizontalStrut(spacer_size));
				reconnectBut=new JButton(carrows);
				reconnectBut.setPreferredSize(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
				reconnectBut.addActionListener(this);
				reconnectBut.setToolTipText("Drop current connection and try again.");
				tmp2.add(reconnectBut);
				tmp.add(tmp2,BorderLayout.EAST);
			}
			statusbox.add(tmp);
		}
		content.add(statusbox, BorderLayout.SOUTH);

		Box p=Box.createVerticalBox();
		p.add(new JLabel("Send Input:"));
		inputFieldHistory=new Vector();
		for(int i=0; i<MAX_STORE_INPUT_HIST; i++)
			inputFieldHistory.add(prefs.get("inputFieldHistory"+i,""));
		inputFieldLocation=inputFieldHistory.size();
		p.add(inputField=new JTextField());
		inputField.setEnabled(false);
		inputField.addActionListener(this);
		inputField.addKeyListener(this);
		inputField.setPreferredSize(new Dimension(35,inputField.getPreferredSize().height));
		inputField.setToolTipText("Text from here is passed to selected controls, or the current control if no selection");

		p.add(Box.createVerticalStrut(spacer_size));
		{
			JSeparator sep=new JSeparator(SwingConstants.HORIZONTAL);
			sep.setMaximumSize(new Dimension(sep.getMaximumSize().width,spacer_size));
			p.add(sep);
		}
		p.add(Box.createVerticalStrut(spacer_size-2));

		{
			Box tmp=Box.createHorizontalBox();
			tmp.add(Box.createHorizontalGlue());
			JButton but;
		 	but=new JButton("Raw");
			but.setToolTipText("Raw vision");
			but.addActionListener(this);
			but.setActionCommand("raw");
			tmp.add(but);
			but=new JButton("Seg");
			but.setToolTipText("Segmented vision");
			but.addActionListener(this);
			but.setActionCommand("rle");
			tmp.add(but);
			but=new JButton("Depth");
			but.setToolTipText("Depth vision");
			but.addActionListener(this);
			but.setActionCommand("depth");
			tmp.add(but);
			tmp.add(Box.createHorizontalGlue());
			tmp.setAlignmentX(0.0f);
			p.add(tmp);
		}

		p.add(Box.createVerticalStrut(spacer_size));
		{
			Box tmp=Box.createHorizontalBox();
			tmp.add(new JLabel("Teleop: "));
			JButton but;
		 	but=new JButton("H");
			but.setToolTipText("Head Controller");
			but.addActionListener(this);
			but.setActionCommand("HeadRC");
			but.setPreferredSize(new Dimension(35,22));
			tmp.add(but);
		 	but=new JButton("W");
			but.setToolTipText("Walk Controller");
			but.addActionListener(this);
			but.setActionCommand("WalkRC");
			but.setPreferredSize(new Dimension(35,22));
			tmp.add(but);
		 	but=new JButton("A");
			but.setToolTipText("Arm Controller");
			but.addActionListener(this);
			but.setActionCommand("ArmRC");
			but.setPreferredSize(new Dimension(35,22));
			tmp.add(but);
			tmp.setAlignmentX(0.0f);
			p.add(tmp);
		}

		p.add(Box.createVerticalStrut(spacer_size));
		{
			Box tmp=Box.createHorizontalBox();
			tmp.add(new JLabel("Sketch: "));
			JButton but;
		 	but=new JButton("C");
			but.setToolTipText("Display Camera Sketch Space");
			but.addActionListener(this);
			but.setActionCommand("CameraSketchSpace");
			but.setPreferredSize(new Dimension(35,22));
			tmp.add(but);
		 	but=new JButton("L");
			but.setToolTipText("Display Local Sketch Space");
			but.addActionListener(this);
			but.setActionCommand("LocalSketchSpace");
			but.setPreferredSize(new Dimension(35,22));
			tmp.add(but);
		 	but=new JButton("W");
			but.setToolTipText("Display World Sketch Space");
			but.addActionListener(this);
			but.setActionCommand("WorldSketchSpace");
			but.setPreferredSize(new Dimension(35,22));
			tmp.add(but);
			tmp.setAlignmentX(0.0f);
			p.add(tmp);
		}

		p.add(Box.createVerticalStrut(spacer_size));
		{
			JSeparator sep=new JSeparator(SwingConstants.HORIZONTAL);
			sep.setMaximumSize(new Dimension(sep.getMaximumSize().width,spacer_size));
			p.add(sep);
		}
		p.add(Box.createVerticalStrut(spacer_size-2));

		p.add(new JLabel("Scripts:"));
		scripts=new JList();
		scripts.setFixedCellWidth(165);
		scripts.addMouseListener(new JListDoubleClick(this));
		scripts.addKeyListener(new ReturnFilter(this));
		scripts.addKeyListener(new DelFilter(this));
		scripts.setModel(scriptsModel);
		{
			JScrollPane tmp=new JScrollPane(scripts);
			tmp.setAlignmentX(0);
			p.add(tmp);
		}

		{
			Box bbox=Box.createHorizontalBox();
			bbox.setAlignmentX(0);
			bbox.add(Box.createHorizontalGlue());
			JButton tmp;
			bbox.add(tmp=new JButton("Add"));
			tmp.setActionCommand("addbookmark");
			tmp.addActionListener(this);
			tmp.setToolTipText("Adds a shortcut to the current item; saved when window closed");
			bbox.add(tmp=new JButton("Edit"));
			new AutoDisableListener(tmp,scripts);
			tmp.setActionCommand("editbookmark");
			tmp.addActionListener(this);
			tmp.setToolTipText("Allows you to edit a script");

			bbox.add(tmp=new JButton("Accessibility"));
			tmp.setActionCommand("changecolors");
			tmp.addActionListener(this);
			tmp.setToolTipText("Options for alternate displays");

			bbox.add(Box.createHorizontalGlue());
			p.add(bbox);
		}

		{
			JPanel p2=new JPanel(new BorderLayout());
			p2.add(p,BorderLayout.EAST);
			p2.add(Box.createHorizontalStrut(spacer_size),BorderLayout.WEST);
			content.add(p2, BorderLayout.EAST);
		}

		getContentPane().add(content,BorderLayout.CENTER);
		EscFilter esc=new EscFilter(backBut);
		addKeyListener(esc);
		inputField.addKeyListener(esc);
		menu.addKeyListener(esc);

		addWindowListener(new CloseVisionAdapter(this));

		setLocation(prefs.getInt("ControllerGUI.location.x",50),prefs.getInt("ControllerGUI.location.y",50));
		int numScripts=prefs.getInt("ControllerGUI.numScripts",-1);
		if(numScripts>-1) {
			for(int i=0; i<numScripts; i++)
				scriptsModel.addElement(new ScriptEntry(prefs.get("ControllerGUI.script"+i+".title","unknown"),prefs.get("ControllerGUI.script"+i+".cmd","")));
		} else {
			//uninitialized, supply some default scripts
			scriptsModel.addElement(new ScriptEntry("Quality Video","!set vision.rawcam.transport=tcp\n!set vision.rawcam.compression=none\n!set vision.rawcam.y_skip=1\n!set vision.rawcam.uv_skip=1\n!set vision.rawcam.interval=1500"));
			scriptsModel.addElement(new ScriptEntry("Smooth Video","!set vision.rawcam.y_skip=2\n!set vision.rawcam.uv_skip=3\n!set vision.rawcam.interval=0\n!set vision.rawcam.compression=jpeg\n!set vision.rawcam.transport=udp"));
		}
		pack();
		title.setMaximumSize(new Dimension(15+25,title.getHeight()));
		if (System.getProperty("os.name").toLowerCase().indexOf("mac") == -1) {
			// On OS X, we set the Dock icon from the launcher, don't want to override the default minimized window preview
			// On other OSes, we do want to set the window icon, which shows up in the task switcher and task bar
			try {
				// only available in Java 6
				//setIconImage(javax.imageio.ImageIO.read(new java.io.File("images/ControllerIcon.png")));
				// use reflection:
				Class fileparam[] = new Class[1];
				fileparam[0] = Image.class;
				java.lang.reflect.Method m = ControllerGUI.class.getMethod("setIconImage",fileparam);
				if(m!=null) {
					m.invoke(this,javax.imageio.ImageIO.read(new java.io.File("images/ControllerIcon.png")));
				}
			} catch(java.io.IOException e) {
			} catch(java.lang.NoSuchMethodException e) {
				e.printStackTrace();
			} catch(java.lang.IllegalAccessException e) {
				e.printStackTrace();
			} catch(java.lang.reflect.InvocationTargetException e) {
				e.printStackTrace();
			}
		}
		setVisible(true);
		getRootPane().setMinimumSize(getRootPane().getSize());
	}

	class CloseVisionAdapter extends WindowAdapter {
		ControllerGUI gui;
		CloseVisionAdapter(ControllerGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			prefs.putInt("ControllerGUI.location.x",getLocation().x);
			prefs.putInt("ControllerGUI.location.y",getLocation().y);
			//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
			//prefs.putInt("ControllerGUI.location.x",getLocation().x+getInsets().left);
			//prefs.putInt("ControllerGUI.location.y",getLocation().y+getInsets().top);
			prefs.putInt("ControllerGUI.numScripts",scriptsModel.getSize());
			for(int i=0; i<scriptsModel.getSize(); i++) {
				prefs.put("ControllerGUI.script"+i+".title",((ScriptEntry)scriptsModel.get(i)).title);
				prefs.put("ControllerGUI.script"+i+".cmd",((ScriptEntry)scriptsModel.get(i)).cmd);
			}
			for(int i=0; i<MAX_STORE_INPUT_HIST; i++)
				prefs.put("inputFieldHistory"+i,(String)inputFieldHistory.get(inputFieldHistory.size()-MAX_STORE_INPUT_HIST+i));
			gui.comm.removeDynObjs();
			gui.comm.kill();
			estop.close();
			while(gui.comm.isConnected())
				try { Thread.sleep(50); System.out.print("."); } catch(Exception ex) {}
		}
	}

	public void setVisible(boolean b) {
		super.setVisible(b);
		if(b)
			menu.requestFocus();
	}

    public static void main(String args[]) throws java.lang.InterruptedException {
	if (args.length == 0 || args.length>3 || 
	    args.length>0 && (args[0].equals("-h") || args[0].equals("--help"))) {
	    System.out.println("Usage: ControllerGUI host [port] [-g]");
	    System.out.println("       If port is unspecified, it will default to "+ControllerListener.defPort);
	    System.exit(2);
	}
	int port = ControllerListener.defPort;
	boolean gamepadEnabled = false;
	if (args.length == 3) {
	    port = Integer.parseInt(args[1]);
	    gamepadEnabled = args[2].equals("-g");
	} else if (args.length == 2) {
	    if ( args[1].equals("-g") )
		gamepadEnabled = true;
	    else
		port = Integer.parseInt(args[1]);
	}
	ControllerGUI controllerGUI = new ControllerGUI(args[0], port, gamepadEnabled);
	controllerGUI.addWindowListener(new WindowAdapter() {
		public void windowClosing(WindowEvent e) { System.exit(0);}
	    });
    }

  public void keyPressed(KeyEvent e) {
    if (e.getComponent()==inputField) {
      int key=e.getKeyCode();
      if (key==KeyEvent.VK_UP || key==KeyEvent.VK_KP_UP) {
        inputFieldLocation--;
        if (inputFieldLocation < 0) {
          java.awt.Toolkit.getDefaultToolkit().beep();
          inputFieldLocation=0;
        }
        inputField.setText((String)inputFieldHistory.get(inputFieldLocation));
      } else if (key==KeyEvent.VK_DOWN || key==KeyEvent.VK_KP_DOWN) {
        inputFieldLocation++;
        if (inputFieldLocation == inputFieldHistory.size()) {
          inputField.setText("");
        } else if (inputFieldLocation > inputFieldHistory.size()) {
          java.awt.Toolkit.getDefaultToolkit().beep();
          inputFieldLocation=inputFieldHistory.size();
        } else
          inputField.setText((String)inputFieldHistory.get(inputFieldLocation));
      }
    }
  }
  public void keyReleased(KeyEvent e) { }
  public void keyTyped(KeyEvent e) { }
}

//multi line tooltips from Zafir Anjum http://www.codeguru.com/java/articles/122.shtml
class JMultiLineToolTip extends JToolTip {
	private static final String uiClassID = "ToolTipUI";

	String tipText;
	JComponent component;

	public JMultiLineToolTip() { updateUI(); }
	public void updateUI() { setUI(MultiLineToolTipUI.createUI(this)); }

	public int getColumns() { return columns; }
	public void setColumns(int columns) {
		this.columns = columns;
		this.fixedwidth = 0;
	}

	public int getFixedWidth() { return fixedwidth; }
	public void setFixedWidth(int width) {
		this.fixedwidth = width;
		this.columns = 0;
	}

	protected int columns = 0;
	protected int fixedwidth = 0;
}

class MultiLineToolTipUI extends BasicToolTipUI {
	static MultiLineToolTipUI sharedInstance = new MultiLineToolTipUI();
	Font smallFont;
	JToolTip tip;
	protected CellRendererPane rendererPane;
	private JTextArea textArea ;

	public static ComponentUI createUI(JComponent c) {
		return sharedInstance;
	}

	public MultiLineToolTipUI() {
		super();
		textArea = new JTextArea();
		smallFont = new Font("sanserif",0,11);
	}

	public void installUI(JComponent c) {
		super.installUI(c);
		tip = (JToolTip)c;
		rendererPane = new CellRendererPane();
		c.add(rendererPane);
	}

	public void uninstallUI(JComponent c) {
		super.uninstallUI(c);
		c.remove(rendererPane);
		rendererPane = null;
	}

	public void paint(Graphics g, JComponent c) {
		String tipText = ((JToolTip)c).getTipText();
		textArea.setText(tipText );
		textArea.setFont(smallFont);
		textArea.setMargin(new Insets(4,6,4,6));
		Dimension size = c.getSize();
		textArea.setBackground(c.getBackground());
		rendererPane.paintComponent(g, textArea, c, 1, 1, size.width - 1, size.height - 1, true);
	}

	public Dimension getPreferredSize(JComponent c) {
		String tipText = ((JToolTip)c).getTipText();
		if (tipText == null)
			return new Dimension(0,0);
		textArea.setText(tipText );
		textArea.setFont(smallFont);
		textArea.setMargin(new Insets(4,6,4,6));
		rendererPane.removeAll();
		rendererPane.add(textArea );
		textArea.setWrapStyleWord(true);
		int width = ((JMultiLineToolTip)c).getFixedWidth();
		int columns = ((JMultiLineToolTip)c).getColumns();

		if( columns > 0 )
		{
			textArea.setColumns(columns);
			textArea.setSize(0,0);
			textArea.setLineWrap(true);
			textArea.setSize( textArea.getPreferredSize() );
		}
		else if( width > 0 )
		{
			textArea.setLineWrap(true);
			Dimension d = textArea.getPreferredSize();
			d.width = width;
			d.height++;
			textArea.setSize(d);
		}
		else
			textArea.setLineWrap(false);

		Dimension dim = textArea.getPreferredSize();

		dim.height += 1;
		dim.width += 1;
		return dim;
	}

	public Dimension getMinimumSize(JComponent c) {
		return getPreferredSize(c);
	}

	public Dimension getMaximumSize(JComponent c) {
		return getPreferredSize(c);
	}
}
