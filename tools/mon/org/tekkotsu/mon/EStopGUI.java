package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.util.prefs.Preferences;


public class EStopGUI extends JFrame implements ActionListener, EStopListener.UpdatedListener {
	static int defPort=10053;
	JLabel status;
	JButton reconnectBut;
	EStopPanel estop;
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
	static Preferences prefs = Preferences.userNodeForPackage(EStopGUI.class);

	static public void main(String s[]) {
		int port=defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		String[] args=new String[s.length-1];
		for(int i=0; i<s.length-1; i++)
			args[i-1]=s[i];
		JFrame frame=new EStopGUI(s[0],port,args);
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) { System.exit(0); }
			});
	}

	public static void usage() {
		System.out.println("Usage: java EStopGUI host [port]");
		System.out.println("			 if port is not specified, it defaults to "+EStopListener.defPort);
		System.exit(2);
	}

	public EStopGUI(String host, int port, String args[]) {
		super("TekkotsuMon: EStop Controller");
		init(new EStopListener(host,port));
	}

	public EStopGUI(EStopListener comm) {
		super();
		init(comm);
	}

	protected void init(EStopListener comm) {
		int spacer_size=10;
		Container root = getContentPane();
		root.setLayout(new BorderLayout());
		root.add(estop=new EStopPanel(comm),BorderLayout.CENTER);
		root.add(Box.createVerticalStrut(spacer_size),BorderLayout.NORTH);
		root.add(Box.createHorizontalStrut(spacer_size),BorderLayout.EAST);
		root.add(Box.createHorizontalStrut(spacer_size),BorderLayout.WEST);
		
		{
			Box statusbox=Box.createVerticalBox();
			statusbox.add(Box.createVerticalStrut(spacer_size));
			{
				JSeparator sep=new JSeparator(SwingConstants.HORIZONTAL);
				statusbox.add(sep);
			}
			statusbox.add(Box.createVerticalStrut(spacer_size-2));
			{
				JPanel tmp=new JPanel(new BorderLayout());
				status=new JLabel("Connecting...");
				estopUpdated(comm);
				tmp.add(Box.createHorizontalStrut(spacer_size),BorderLayout.WEST);
				tmp.add(status,BorderLayout.CENTER);
				{
					Box tmp2=Box.createHorizontalBox();
					reconnectBut=new JButton(carrows);
					reconnectBut.setPreferredSize(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
					reconnectBut.addActionListener(this);
					reconnectBut.setToolTipText("Drop current connection and try again.");
					tmp2.add(reconnectBut);
					tmp2.add(Box.createHorizontalStrut(spacer_size));
					tmp.add(tmp2,BorderLayout.EAST);
				}
				statusbox.add(tmp);
			}
			statusbox.add(Box.createVerticalStrut(spacer_size));
			root.add(statusbox, BorderLayout.SOUTH);
		}
		addWindowListener(new CloseEStopAdapter(this));
		estop.setPreferredSize(new Dimension(150,150));
		pack();
		estop.setPreferredSize(null);
		estop.comm.addUpdatedListener(this);
		setLocation(prefs.getInt("EStopGUI.location.x",50),prefs.getInt("EStopGUI.location.y",50));
		setVisible(true);
	}

	public void actionPerformed(ActionEvent evt) {
		if(evt.getSource()==reconnectBut) {
			estop.close();
			estop.open();
		}
	}
	class CloseEStopAdapter extends WindowAdapter {
		EStopGUI gui;
		CloseEStopAdapter(EStopGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}
	public void close() {
		prefs.putInt("EStopGUI.location.x",getLocation().x);
		prefs.putInt("EStopGUI.location.y",getLocation().y);
		//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
		//prefs.putInt("EStopGUI.location.x",getLocation().x+getInsets().left);
		//prefs.putInt("EStopGUI.location.y",getLocation().y+getInsets().top);
		estop.remove();
		estop.comm.removeUpdatedListener(this);
		dispose();
	}
	public void estopUpdated(EStopListener l) {
		if(l.isConnected()) {
			status.setText("Connected.");
		} else {
			status.setText("Reconnecting...");
		}
	}
}
