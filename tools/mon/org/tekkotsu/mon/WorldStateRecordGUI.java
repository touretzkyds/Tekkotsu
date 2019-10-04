package org.tekkotsu.mon;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.util.prefs.Preferences;
import java.io.*;

public class WorldStateRecordGUI extends JFrame implements ActionListener, Listener.ConnectionListener, WorldStateJointsListener.UpdatedListener {
	JLabel status;
	JButton reconnectBut,recordBut,stopBut;
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
	static Preferences prefs = Preferences.userNodeForPackage(WorldStateRecordGUI.class);
	WorldStateJointsListener comm;
	String saveBaseName;
	String savePath;
	long firstFrameTime=-1;

	static public void main(String s[]) {
		int port=WorldStateJointsListener.defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		String[] args=new String[s.length-1];
		for(int i=0; i<s.length-1; i++)
			args[i-1]=s[i];
		JFrame frame=new WorldStateRecordGUI(s[0],port,args);
		frame.addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) { System.exit(0); }
			});
	}

	public static void usage() {
		System.out.println("Usage: java WorldStateRecordGUI host [port]");
		System.out.println("			 if port is not specified, it defaults to "+WorldStateJointsListener.defPort);
		System.exit(2);
	}

	public WorldStateRecordGUI(String host, int port, String args[]) {
		super("TekkotsuMon: WorldState Recorder");
		init(WorldStateJointsListener.getCommonInstance(host,port));
	}

	public WorldStateRecordGUI(WorldStateJointsListener comm) {
		super();
		init(comm);
	}

	class RecordButton extends JButton {
		public void paint(Graphics g) {
			super.paint(g);
			Rectangle r=new Rectangle(0,0,getSize().width,getSize().height);
			if(isEnabled())
				g.setColor(new Color(255,0,0,255));
			else
				g.setColor(new Color(255,0,0,64));
			float margin=.15f;
			int hgap=(int)(r.width*margin);
			int vgap=(int)(r.height*margin);
			g.fillOval(r.x+hgap,r.y+vgap,r.width-hgap*2,r.height-vgap*2);
		}
	}

	class StopButton extends JButton {
		public void paint(Graphics g) {
			super.paint(g);
			Rectangle r=new Rectangle(0,0,getSize().width,getSize().height);
			if(isEnabled())
				g.setColor(new Color(0,0,0,255));
			else
				g.setColor(new Color(0,0,0,64));
			float margin=.25f;
			int hgap=(int)(r.width*margin);
			int vgap=(int)(r.height*margin);
			g.fillRect(r.x+hgap,r.y+vgap,r.width-hgap*2,r.height-vgap*2);
		}
	}

	protected void init(WorldStateJointsListener comm) {
		this.comm=comm;
		int spacer_size=10;
		Container root = getContentPane();
		root.setLayout(new BorderLayout());
		root.add(Box.createVerticalStrut(spacer_size),BorderLayout.NORTH);
		root.add(Box.createHorizontalStrut(spacer_size),BorderLayout.EAST);
		root.add(Box.createHorizontalStrut(spacer_size),BorderLayout.WEST);
		
		{
			GridLayout gl=new GridLayout(1,2);
			gl.setHgap(spacer_size);
			gl.setVgap(spacer_size);
			JPanel tmp=new JPanel(gl);
			recordBut=new RecordButton();
			recordBut.setPreferredSize(new Dimension(96,96));
			recordBut.setToolTipText("Connect and begin recording data stream");
			recordBut.addActionListener(this);
			tmp.add(recordBut);
			stopBut=new StopButton();
			stopBut.setPreferredSize(new Dimension(96,96));
			stopBut.setEnabled(false);
			stopBut.setToolTipText("Stop recording data stream");
			stopBut.addActionListener(this);
			tmp.add(stopBut);
			root.add(tmp,BorderLayout.CENTER);
		}

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
				status=new JLabel("Waiting to record...");
				worldStateUpdated(comm);
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
		addWindowListener(new CloseWorldStateAdapter(this));
		//worldState.setPreferredSize(new Dimension(150,150));
		pack();
		//worldState.setPreferredSize(null);
		setLocation(prefs.getInt("WorldStateRecordGUI.location.x",50),prefs.getInt("WorldStateRecordGUI.location.y",50));
		setVisible(true);
	}

	public void actionPerformed(ActionEvent evt) {
		if(evt.getSource()==reconnectBut) {
			//worldState.close();
			//worldState.open();
		} else if(evt.getSource()==recordBut) {
			File cursavepath = new File(prefs.get("cursavepath",""));
			JFileChooser dia=new JFileChooser(cursavepath);
			dia.setDialogTitle("Save Sensor Data Sequence...");
			Component cur=this;
			while(cur.getParent()!=null)
				cur=cur.getParent();
			status.setText("Waiting for destination...");
			if(dia.showSaveDialog(cur)==JFileChooser.APPROVE_OPTION) {
				prefs.put("cursavepath",dia.getCurrentDirectory().getPath());
				saveBaseName=dia.getSelectedFile().getName();
				savePath=dia.getSelectedFile().getParent();
				firstFrameTime=-1;
				recordBut.setEnabled(false);
				stopBut.setEnabled(true);
				status.setText("Connecting...");
				comm.addConnectionListener(this);
				comm.addUpdatedListener(this);
			}
		} else if(evt.getSource()==stopBut) {
			stopBut.setEnabled(false);
			recordBut.setEnabled(true);
			status.setText("Waiting to record...");
			comm.removeUpdatedListener(this);
			comm.removeConnectionListener(this);
			saveBaseName=null;
			firstFrameTime=-1;
		}
	}
	class CloseWorldStateAdapter extends WindowAdapter {
		WorldStateRecordGUI gui;
		CloseWorldStateAdapter(WorldStateRecordGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}
	public void close() {
		prefs.putInt("WorldStateRecordGUI.location.x",getLocation().x);
		prefs.putInt("WorldStateRecordGUI.location.y",getLocation().y);
		//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
		//prefs.putInt("WorldStateRecordGUI.location.x",getLocation().x+getInsets().left);
		//prefs.putInt("WorldStateRecordGUI.location.y",getLocation().y+getInsets().top);
		//worldState.remove();
		comm.removeConnectionListener(this);
		comm.removeUpdatedListener(this);
		dispose();
	}
	public void onConnected() {
		status.setText("Connected.");
	}
	public void onDisconnected() {
		status.setText("Waiting to record...");
	}
	public void worldStateUpdated(WorldStateJointsListener l) {
		if(!l.isConnected()) {
			status.setText("Waiting to record...");
		} else {
			status.setText("Recording...");
			Joints j=comm.getData();
			if(firstFrameTime==-1)
				firstFrameTime=j.timestamp;
			StringBuffer base=new StringBuffer(saveBaseName);
			long c=j.timestamp-firstFrameTime;
			int digits=6;
			for(int s=(int)Math.pow(10,digits-1); s>=1; s/=10) {
				base.append(c/s);
				c-=(c/s)*s;
			}
			try {
				PrintStream ps=new PrintStream(new FileOutputStream(savePath+File.separator+base+".pos"));
				ps.println("#POS");
				ps.println("condensed "+j.model);
				ps.println("meta-info = "+j.timestamp+" "+j.frame);
				ps.print("outputs =");
				for(int i=0; i<j.positions.length; i++)
					ps.print(" "+j.positions[i]);
				ps.print("\nbuttons =");
				for(int i=0; i<j.buttons.length; i++)
					ps.print(" "+j.buttons[i]);
				ps.print("\nsensors =");
				for(int i=0; i<j.sensors.length; i++)
					ps.print(" "+j.sensors[i]);
				ps.print("\npidduties =");
				for(int i=0; i<j.duties.length; i++)
					ps.print(" "+j.duties[i]);
				ps.println("\n#END");
				ps.close();
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
	}
}
