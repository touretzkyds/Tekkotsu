package org.tekkotsu.mon;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.beans.*;
import java.util.*;
import java.util.prefs.Preferences;

//Admittedly a little sloppy, but I don't want to spend forever on this...

public class EditScriptGUI extends JFrame implements ActionListener, ListDataListener /*ComponentListener*/ {
	DefaultListModel list;
	int scriptIndex;
	ScriptEntry script;
	static Preferences prefs = Preferences.userNodeForPackage(WalkGUI.class);
	int height=0;
	JTextField title;
	JTextArea command;
	JButton upBut;
	JButton downBut;
	boolean deleteOnCancel;

	public EditScriptGUI(DefaultListModel list, int scriptIndex, boolean deleteOnCancel) {
		super("TekkotsuMon: Edit Script");
		this.scriptIndex=scriptIndex;
		this.list=list;
		this.deleteOnCancel=deleteOnCancel;
		list.addListDataListener(this);
		script=(ScriptEntry)list.get(scriptIndex);
		title.setText(script.title);
		command.setText(script.cmd);
		checkMoveButtons();
		pack();
		setLocation(prefs.getInt("EditScriptGUI.location.x",50),prefs.getInt("EditScriptGUI.location.y",50));
		setVisible(true);
		height=getBounds().height;
		title.setMaximumSize(new Dimension(Integer.MAX_VALUE,title.getSize().height));
//		titleLabel.setMaximumSize(new Dimension(Integer.MAX_VALUE,titleLabel.getSize().height));
	}
	
	public void checkMoveButtons() {
		scriptIndex=list.indexOf(script);
		upBut.setEnabled((scriptIndex!=0));
		downBut.setEnabled((scriptIndex!=list.getSize()-1));
	}

	public void frameInit() {
		super.frameInit();

		int strutsize=10;
		int sepsize=5;
		getContentPane().setLayout(new BorderLayout());
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.EAST);
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.WEST);
		getContentPane().add(Box.createVerticalStrut(strutsize),BorderLayout.NORTH);
		getContentPane().add(Box.createVerticalStrut(strutsize),BorderLayout.SOUTH);
		
		{
			Box tmp = Box.createVerticalBox();
			{
				Box tmp2=Box.createHorizontalBox();
				JLabel titleLabel=new JLabel("Title: ");
				tmp2.add(titleLabel);
				tmp2.add(title=new JTextField());
				tmp.add(tmp2);
			}
			tmp.add(Box.createVerticalStrut(strutsize));
			{
				Box tmp2=Box.createHorizontalBox();
				JLabel cmdLabel=new JLabel("Commands: ");
				tmp2.add(cmdLabel);
				command=new JTextArea();
				command.setLineWrap(true);
				command.setWrapStyleWord(true); 
				command.setColumns(35);
				command.setRows(6);
				{
					JScrollPane tmp3=new JScrollPane(command);
					tmp2.add(tmp3);
				}
				tmp.add(tmp2);
			}
			tmp.add(Box.createVerticalStrut(strutsize));
			{
				Box tmp2=Box.createHorizontalBox();
				JButton but;
				tmp2.add(but=upBut=new JButton("Delete"));
				but.setActionCommand("delete");
				but.addActionListener(this);
				tmp2.add(Box.createHorizontalStrut(strutsize));
				tmp2.add(Box.createHorizontalGlue());
				tmp2.add(but=upBut=new JButton("Move Up"));
				but.setActionCommand("up");
				but.addActionListener(this);
				tmp2.add(Box.createHorizontalStrut(strutsize));
				tmp2.add(but=downBut=new JButton("Move Down"));
				but.setActionCommand("down");
				but.addActionListener(this);
				tmp2.add(Box.createHorizontalStrut(strutsize));
				tmp2.add(Box.createHorizontalGlue());
				tmp2.add(but=new JButton("Cancel"));
				but.setActionCommand("cancel");
				but.addActionListener(this);
				tmp2.add(Box.createHorizontalStrut(strutsize));
				tmp2.add(but=new JButton("OK"));
				but.setActionCommand("ok");
				but.addActionListener(this);
				tmp.add(tmp2);
			}
			getContentPane().add(tmp,BorderLayout.CENTER);
		}
		setResizable(false);
		
//		addComponentListener(this);
		addWindowListener(new CloseEditScriptGUIAdapter(this));
	}

	public void actionPerformed(ActionEvent e) {
		if(e.getActionCommand().equals("ok")) {
			if(title.getText().equals("")) {
				int result=JOptionPane.showConfirmDialog(this,"An empty title will delete the script.  Continue?","Warning: Really Delete?",JOptionPane.OK_CANCEL_OPTION,JOptionPane.WARNING_MESSAGE);
				if(result==JOptionPane.CANCEL_OPTION)
					return;
				list.remove(scriptIndex);
			} else {
				script.title=title.getText();
				script.cmd=command.getText();
				list.add(scriptIndex,list.remove(scriptIndex));
			}
			close();
		} else if(e.getActionCommand().equals("cancel")) {
			if(deleteOnCancel)
				list.remove(scriptIndex);
			close();
		} else if(e.getActionCommand().equals("up")) {
			list.add(scriptIndex,list.remove(scriptIndex-1));
		} else if(e.getActionCommand().equals("down")) {
			list.add(scriptIndex,list.remove(scriptIndex+1));
		} else if(e.getActionCommand().equals("delete")) {
			int result=JOptionPane.showConfirmDialog(this,"This will delete the script.  Continue?","Warning: Really Delete?",JOptionPane.OK_CANCEL_OPTION,JOptionPane.WARNING_MESSAGE);
			if(result==JOptionPane.CANCEL_OPTION)
				return;
			list.remove(scriptIndex);
			close();
		} else {
			System.out.println("EditScriptGUI: Unknown action event");
		}
	}

	public void close() {
		prefs.putInt("EditScriptGUI.location.x",getLocation().x);
		prefs.putInt("EditScriptGUI.location.y",getLocation().y);
		//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
		//prefs.putInt("EditScriptGUI.location.x",getLocation().x+getInsets().left);
		//prefs.putInt("EditScriptGUI.location.y",getLocation().y+getInsets().top);
		dispose();
	}

/*	public void componentResized(ComponentEvent e) {
		if (e.getID()==ComponentEvent.COMPONENT_RESIZED) {
//			if(height!=0)
//				setBounds(getBounds().x,getBounds().y,getBounds().width,height);
			System.out.println(getInsets());
			System.out.println(getBounds());
		}
	}
	public void componentHidden(ComponentEvent e) { }
	public void componentMoved(ComponentEvent e) { }
	public void componentShown(ComponentEvent e) { }
*/	
	
	public void contentsChanged(ListDataEvent e) {checkMoveButtons(); }
	public void intervalAdded(ListDataEvent e) {checkMoveButtons(); }
	public void intervalRemoved(ListDataEvent e) {checkMoveButtons(); }

	class CloseEditScriptGUIAdapter extends WindowAdapter {
		EditScriptGUI gui;
		CloseEditScriptGUIAdapter(EditScriptGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}

	static public void main(String s[]) {
		DefaultListModel list=new DefaultListModel();
		list.addElement(new ScriptEntry("test","blah blah blah"));
		list.addElement(new ScriptEntry("test2","blah blah blah"));
		list.addElement(new ScriptEntry("test3","blah blah blah"));
		list.addElement(new ScriptEntry("test4","blah blah blah"));
		EditScriptGUI gui=new EditScriptGUI(list,0,true);
		gui.addWindowListener(new WindowAdapter() { public void windowClosing(WindowEvent e) { System.exit(0); } 	});
	}
}

