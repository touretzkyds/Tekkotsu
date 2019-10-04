package org.tekkotsu.mon;

import java.awt.event.*;
import javax.swing.*;
import javax.swing.filechooser.FileFilter;
import java.lang.String;
import java.util.LinkedList;
import java.awt.*;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.image.IndexColorModel;
import java.util.Date;
import java.io.PrintWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.util.prefs.Preferences;
import java.io.File;


public class VisionGUI extends JFrame implements ActionListener, VisionUpdatedListener {
	VisionPanel vision;
	JCheckBox aspectBut;
	JButton freezeBut;
	JButton saveImageBut;
	JLabel status;
	JButton reconnectBut;
	float mspf=0;
	float mspfGamma=.9f;
	long lastFrameTime=0;
	boolean isRaw=false;
	boolean isRLE=false;
        boolean isDepth=false;
	boolean connected=false;
	boolean isFreezeFrame=false;
	JRadioButton rgbMode,yuvMode;
	final static ImageIcon carrows = new ImageIcon("images/chasingarrows.png");
	ImageSequenceWriterThread imgWrite=null;
	String state="Connecting...";
	static Preferences prefs = Preferences.userNodeForPackage(VisionGUI.class);

	public class StatusUpdateThread extends Thread {
		VisionGUI gui;
		public StatusUpdateThread(VisionGUI gui) { super("StatusUpdate"); this.gui=gui; setDaemon(true);}
		public void run() {
			VisionListener l=gui.vision.getListener();
			while(true) {
				String status=gui.state;
				String recording=recordReport();
				String fps=fpsReport();
				if(fps.length()>0)
					status+=" "+fps;
				if(recording.length()>0)
					status+="; "+recording;
				gui.status.setText(status);
				try {
					sleep(100);
				} catch(Exception ex) {break;}
			}
		}
		public String recordReport() {
			if(gui.imgWrite!=null && !gui.imgWrite.isAlive())
				gui.imgWrite=null;
			if(gui.imgWrite!=null && !gui.imgWrite.stopping) {
				String ans;
				if(!gui.imgWrite.stopping) {
					ans="Rec: ";
					if(imgWrite.imgBufs.size()!=0)
						ans+="Free="+imgWrite.imgBufs.size();
				} else
					ans="Writing";
				ans+=" Queue="+imgWrite.q.size()/4;
				return ans;
			} else
				return "";
		}
		public String fpsReport() {
				if(connected && gui.mspf>.001) {
					int rnd=(int)(10000/gui.mspf);
					return rnd/10.0f+" fps";
				} else
					return "";
		}
	}

	public class SaveSequenceDialog extends JDialog implements ActionListener {
		JFileChooser dia;
		boolean success;
		JCheckBox positionsBut;
		JButton saveBut;
		JButton cancelBut;
		VisionGUI gui;
		SaveSequenceDialog(VisionGUI vg, File curpath) {
			super(vg,"Save Image Sequence...");
			gui=vg;
			{
				Box tmp=Box.createVerticalBox();
				dia=new JFileChooser(curpath);
				//dia.setControlButtonsAreShown(false);
				dia.setApproveButtonText("Save Here");
				dia.setDialogType(JFileChooser.SAVE_DIALOG);
				dia.addActionListener(this);
				tmp.add(dia);
				{
					Box tmp2=Box.createHorizontalBox();
					tmp2.add(Box.createHorizontalStrut(10));
					positionsBut=new JCheckBox("Save joint positions as well",!VisionGUI.prefs.get("save_positions","0").equals("0"));
					positionsBut.addActionListener(this);
					tmp2.add(positionsBut);
					tmp2.add(Box.createHorizontalGlue());
					tmp.add(tmp2);
				}
				/*
				tmp.add(Box.createVerticalStrut(5));
				{
					Box tmp2=Box.createHorizontalBox();
					tmp2.add(Box.createHorizontalGlue());
					saveBut=new JButton("Save Here");
					saveBut.addActionListener(this);
					tmp2.add(saveBut);
					tmp2.add(Box.createHorizontalStrut(5));
					cancelBut=new JButton("Cancel");
					cancelBut.addActionListener(this);
					tmp2.add(cancelBut);
					tmp2.add(Box.createHorizontalStrut(10));
					tmp.add(tmp2);
					}*/
				tmp.add(Box.createVerticalStrut(10));
				getContentPane().add(tmp,BorderLayout.CENTER);
				getRootPane().setDefaultButton(saveBut);
			}
			pack();
			success=false;
		}
		public void actionPerformed(ActionEvent e) {
			/*if(e.getSource()==saveBut) {
				success=true;
				dia.approveSelection();
				dispose();
			} else if(e.getActionCommand().equals("Cancel")) {
				dia.cancelSelection();
				dispose();
				} else */
			if(e.getSource()==positionsBut) {
				VisionGUI.prefs.put("save_positions",positionsBut.isSelected()?"1":"0");
			} else if(e.getSource()==dia) {
				if(e.getActionCommand().equals("ApproveSelection")) {
					boolean isRGB=gui.vision.getConvertRGB(); 
					boolean haveJPEGs=gui.vision.getListener().hasRawJPEG();
					String name=dia.getSelectedFile().getName();
					boolean hasExtension=name.lastIndexOf(".")!=-1;
					boolean toJPEG=name.endsWith(".jpeg") || name.endsWith(".jpg");
					if(!toJPEG && haveJPEGs) {
						if(hasExtension) {
							int n=JOptionPane.showConfirmDialog(this,"The original data stream consists of JPEG images, are you sure you\nwant to save in a different format?","Convert formats?",JOptionPane.OK_CANCEL_OPTION);
							if(n!=JOptionPane.OK_OPTION)
								return;
						} else {
							//silently default to JPEG if left unspecified
							dia.setSelectedFile(new File(dia.getSelectedFile().getPath()+".jpg"));
							hasExtension=toJPEG=true;
						}
					}
					if(toJPEG) {
						if(!isRGB) {
							int n=JOptionPane.showConfirmDialog(this,"JPEGs store YUV data internally, so it is recommended to save JPEGs as\n\"RGB\" images so the YUV data can be extracted directly from the\ninternal storage, avoiding color space conversions.\n\nDo you want to switch to \"RGB\" mode? (recommended)\n","Save 'RGB' jpeg?",JOptionPane.YES_NO_CANCEL_OPTION);
							if(n==JOptionPane.YES_OPTION)
								gui.rgbMode.doClick();
							if(n==JOptionPane.CANCEL_OPTION)
								return;
						}
					}
					success=true;
					dispose();
				} else if(e.getActionCommand().equals("CancelSelection")) {
					dispose();
				}
			}
		}
		public File getCurrentDirectory() { return dia.getCurrentDirectory(); }
		public File getSelectedFile() { return dia.getSelectedFile(); }
	}
	
	public static void main(String s[]) {
		int port=-1;
		if(s.length<2)
			VisionPanel.usage();
		String[] args=new String[s.length-1];
		for(int i=0; i<args.length; i++)
			args[i]=s[i+1];
		VisionGUI gui=new VisionGUI(s[0],args);
		gui.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) { System.exit(0); } });
	}
				
	public void actionPerformed(ActionEvent e) {
		if(e.getActionCommand().compareTo("YUV")==0) {
			vision.setConvertRGB(false);
		} else if(e.getActionCommand().compareTo("RGB")==0) {
			vision.setConvertRGB(true);
		} else if(e.getActionCommand().compareTo("freeze")==0) {
			isFreezeFrame=!isFreezeFrame;
			if(!isFreezeFrame) {
				state="Reconnecting...";
				vision.open();
			} else {
				state="Disconnecting...";
				vision.close();
			}
		} else if(e.getActionCommand().compareTo("aspect")==0) {
			vision.setLockAspectRatio(((JCheckBox)e.getSource()).isSelected());
		} else if(e.getActionCommand().compareTo("seq")==0) {
			File cursavepath = new File(prefs.get("cursavepath",""));
			SaveSequenceDialog dia = new SaveSequenceDialog(this,cursavepath);
			dia.setVisible(true);
			dia.addWindowListener(new WindowAdapter() {
					public void windowClosed(WindowEvent e) {
						SaveSequenceDialog dia=(SaveSequenceDialog)e.getWindow();
						dia.removeWindowListener(this);
						prefs.put("cursavepath",dia.getCurrentDirectory().getPath());
						if(dia.success)
							dia.gui.startSaveSequence(dia.getSelectedFile(),dia.positionsBut.isSelected());
					}
				});
		} else if(e.getActionCommand().compareTo("stopseq")==0) {
			saveImageBut.setText("Save Image Sequence");
			saveImageBut.setToolTipText("Saves to a series of files - use .jpg or .png extension to choose format; #'s will be replaced with index, otherwise timecode is appended");
			saveImageBut.setActionCommand("seq");
			if(imgWrite!=null)
				imgWrite.interrupt();
		} else if(e.getActionCommand().compareTo("img")==0) {
			File cursavepath = new File(prefs.get("cursavepath",""));
			SaveSequenceDialog dia = new SaveSequenceDialog(this,cursavepath);
			dia.setVisible(true);
			dia.addWindowListener(new WindowAdapter() {
					public void windowClosed(WindowEvent e) {
						SaveSequenceDialog dia=(SaveSequenceDialog)e.getWindow();
						dia.removeWindowListener(this);
						prefs.put("cursavepath",dia.getCurrentDirectory().getPath());
						if(dia.success)
							dia.gui.saveImage(dia.getSelectedFile(),dia.positionsBut.isSelected());
					}
				});
		} else if(e.getSource()==reconnectBut) {
			vision.kill();
			vision.open();
		}
	}

	void startSaveSequence(File file, boolean positions) {
		imgWrite=new ImageSequenceWriterThread(vision.getListener(), saveImageBut, !vision.getConvertRGB(), positions);
		saveImageBut.setText("Stop Saving Sequence");
		saveImageBut.setToolTipText("Click to stop buffering new frames - already captured frames will continue to be written (unless you close the window)");
		saveImageBut.setActionCommand("stopseq");
		String base=file.getName();
		String format;
		if(base.lastIndexOf('.')==-1) {
			format="png";
		} else {
			int i=base.lastIndexOf(".");
			format=base.substring(i+1);
			base=base.substring(0,i);
		}
		int first=base.indexOf('#');
		int last=base.lastIndexOf('#');
		boolean appendTime=(first==-1);
		imgWrite.setDirectory(file.getParent());
		if(first!=-1)
			imgWrite.setName(base.substring(0,first),last-first+1,base.substring(last+1),format);
		else
			imgWrite.setName(base,0,"",format);
		imgWrite.start();
	}

	void saveImage(File file, boolean savePose) {
		String base=file.getName();
		String format;
		if(base.lastIndexOf('.')==-1) {
			format="png";
		} else {
			int i=base.lastIndexOf(".");
			format=base.substring(i+1);
			base=base.substring(0,i);
		}
		try {
			FileOutputStream fileout=new FileOutputStream(file.getParent()+File.separator+base+"."+format);
			if(vision.getConvertRGB() && vision.getListener().hasRawJPEG() && (format.equalsIgnoreCase("jpg") || format.equalsIgnoreCase("jpeg"))) {
				byte[] jpeg=vision.getListener().getJPEG();
				int len=vision.getListener().getJPEGLen();
				fileout.write(jpeg,0,len);
			} else {
				ImageIO.write(vision.getListener().getImage(),format,fileout);
			}
		} catch(Exception ex) {ex.printStackTrace();}
		if(savePose) {
			format="pos";
			try {
				FileOutputStream fileout=new FileOutputStream(file.getParent()+File.separator+base+"."+format);
				fileout.write(vision.getListener().getSensors().getBytes());
			} catch(Exception ex) {ex.printStackTrace();}
		}
	}

	class CloseVisionAdapter extends WindowAdapter {
		VisionGUI gui;
		CloseVisionAdapter(VisionGUI gui) {this.gui=gui;}
		public void windowClosing(WindowEvent e) {
			gui.close();
		}
	}

	public void close() {
		String name="VisionGUI"+(isRaw?".raw":"")+(isRLE?".rle":"")+".location";
		prefs.putInt(name+".x",getLocation().x);
		prefs.putInt(name+".y",getLocation().y);
		//I think I had needed to add getInsets() to keep the window from moving when reopening it, but now it *causes* it to move... weird.  what changed?
		//prefs.putInt(name+".x",getLocation().x+getInsets().left);
		//prefs.putInt(name+".y",getLocation().y+getInsets().top);
		this.vision.kill();
		if(this.imgWrite!=null && this.imgWrite.isAlive()) {
			if(!this.imgWrite.isInterrupted() && !this.imgWrite.stopping)
				this.imgWrite.interrupt();
			while(this.imgWrite.isInterrupted())
				try { Thread.sleep(50); } catch(Exception ex) {}
			if(!this.imgWrite.stopping)
				System.out.println("imgWrite refuses to stop");
			this.imgWrite.interrupt(); //this second one should kill it
			while(this.imgWrite.isAlive())
				try { Thread.sleep(50); } catch(Exception ex) {}
		}
		vision.getListener().removeListener(this);
		vision.getListener().removeListener(vision);
		dispose();
	}
	public void visionUpdated(VisionListener l) {
		if(l.isConnected()!=connected) {
			connected=l.isConnected();
			if(connected) {
				freezeBut.setEnabled(true);
				freezeBut.setText("Freeze Frame");
				freezeBut.setToolTipText("Freezes current frame (disconnects from stream)");
				isFreezeFrame=false;
				saveImageBut.setEnabled(true);
				saveImageBut.setText("Save Image Sequence");
				saveImageBut.setToolTipText("Saves to a series of files - use .jpg or .png extension to choose format; #'s will be replaced with index, otherwise timecode is appended");
				saveImageBut.setActionCommand("seq");
				state="Connected.";
			} else {
				if(vision._image==null)
					saveImageBut.setEnabled(false);
				else {
					saveImageBut.setText("Save Image");
					saveImageBut.setToolTipText("Save current image shown - use .jpg or .png extension to choose format");
					saveImageBut.setActionCommand("img");
				}
				if(isFreezeFrame) {
					state="Disconnected.";
					freezeBut.setText("Unfreeze");
					freezeBut.setToolTipText("Reconnects to stream");
				} else {
					state="Reconnecting...";
					freezeBut.setEnabled(false);
				}
			}
		}
		if(connected) {
			if(lastFrameTime==0) {
				if(l.getTimeStamp()!=null)
					lastFrameTime=l.getTimeStamp().getTime();
			} else {
				long cur=l.getTimeStamp().getTime();
				mspf=mspf*mspfGamma+(cur-lastFrameTime)*(1-mspfGamma);
				lastFrameTime=cur;
			}
		} else {
			lastFrameTime=0;
		}
	}
	public void sensorsUpdated(VisionListener l) {}
	
	public VisionGUI(String host, String[] args) {
		super();
		init(host,args);
	}
	public VisionGUI(String host, int port, String[] args) {
		super();
		String[] passargs=new String[args.length+1];
		passargs[0]=String.valueOf(port);
		for(int i=0; i<args.length; i++)
			passargs[i+1]=args[i];
		init(host,passargs);
	}
	public void init(String host, String[] args) {
		int strutsize=10;
		int sepsize=5;
		getContentPane().setLayout(new BorderLayout());
		getContentPane().add(Box.createVerticalStrut(strutsize),BorderLayout.NORTH);
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.WEST);
		getContentPane().add(Box.createHorizontalStrut(strutsize),BorderLayout.EAST);
		vision=new VisionPanel(host,args);
		for(int i=0; i<args.length; i++) {
			if(args[i].toUpperCase().compareTo("RLE")==0 || args[i]==String.valueOf(VisionListener.defRLEPort)) {
				setTitle("TekkotsuMon: Vision RLE");
				isRLE=true;
				isRaw=false;
				isDepth=false;
			} else if(args[i].toUpperCase().compareTo("RAW")==0 || args[i]==String.valueOf(VisionListener.defRawPort)) {
				setTitle("TekkotsuMon: Vision Raw");
				isRaw=true;
				isRLE=false;
				isDepth=false;
			} else if(args[i].toUpperCase().compareTo("DEPTH")==0 || args[i]==String.valueOf(VisionListener.defDepthPort)) {
				setTitle("TekkotsuMon: Vision Depth");
				isRaw=false;
				isRLE=false;
				isDepth=true;
			}  else if(args[i].toUpperCase().compareTo("REG")==0 || args[i]==String.valueOf(VisionListener.defRegionPort)) {
				setTitle("TekkotsuMon: Vision Regions");
				isRaw=false;
				isRLE=true; //It's not really an RLE window, but the options are the same, so whatever
				isDepth=false;
			} 
		}
		vision.setMinimumSize(new Dimension(VisionListener.DEFAULT_WIDTH/2, VisionListener.DEFAULT_HEIGHT/2));
		vision.setPreferredSize(new Dimension(VisionListener.DEFAULT_WIDTH*2, VisionListener.DEFAULT_HEIGHT*2));
		vision.setLockAspectRatio(true);
		getContentPane().add(vision,BorderLayout.CENTER);
		{
			Box tmp2=Box.createHorizontalBox();
			tmp2.add(Box.createHorizontalStrut(strutsize));
			{
				Box tmp3=Box.createVerticalBox();
				if(!isRLE && !isDepth) {
					Box tmp4=Box.createHorizontalBox();
					ButtonGroup group=new ButtonGroup();
					rgbMode=new JRadioButton("RGB");
					rgbMode.setSelected(true);
					rgbMode.addActionListener(this);
					rgbMode.setToolTipText("Shows RGB colorspace");
					group.add(rgbMode);
					tmp4.add(rgbMode);
					yuvMode=new JRadioButton("YUV");
					yuvMode.addActionListener(this);
					yuvMode.setToolTipText("Shows YUV colorspace");
					group.add(yuvMode);
					tmp4.add(yuvMode);
					tmp3.add(tmp4);
				}
				if(isDepth) { vision.setConvertRGB(false); }
				aspectBut=new JCheckBox("Lock Aspect Ratio");
				aspectBut.setAlignmentX(0.5f);
				aspectBut.addActionListener(this);
				aspectBut.setActionCommand("aspect");
				aspectBut.setSelected(true);
				aspectBut.setToolTipText("Forces displayed image to hold transmitted image's aspect ratio");
				tmp3.add(aspectBut);
				freezeBut=new JButton("Freeze Frame");
				freezeBut.setAlignmentX(0.5f);
				freezeBut.addActionListener(this);
				freezeBut.setActionCommand("freeze");
				freezeBut.setEnabled(false);
				freezeBut.setToolTipText("Freezes current frame (disconnects from stream)");
				tmp3.add(freezeBut);
				saveImageBut=new JButton("Save Image Sequence");
				saveImageBut.setAlignmentX(0.5f);
				saveImageBut.addActionListener(this);
				saveImageBut.setActionCommand("seq");
				saveImageBut.setEnabled(false);
				saveImageBut.setToolTipText("Saves to a series of files - use .jpg or .png extension to choose format; #'s will be replaced with index, otherwise timecode is appended");
				tmp3.add(saveImageBut);
				tmp3.add(Box.createVerticalStrut(strutsize));
				tmp3.add(new JSeparator());
				tmp3.add(Box.createVerticalStrut(strutsize-sepsize));
				{
					Box tmp4=Box.createHorizontalBox();
					tmp4.add(status=new JLabel(state));
					tmp4.add(Box.createHorizontalGlue());
					reconnectBut=new JButton(carrows);
					reconnectBut.setPreferredSize(new Dimension(carrows.getIconWidth(),carrows.getIconHeight()));
					reconnectBut.addActionListener(this);
					reconnectBut.setToolTipText("Drop current connection and try again.");
					tmp4.add(reconnectBut);
					tmp3.add(tmp4);
				}
				tmp3.add(Box.createVerticalStrut(strutsize));
				tmp2.add(tmp3);
			}
			tmp2.add(Box.createHorizontalStrut(strutsize));
			getContentPane().add(tmp2,BorderLayout.SOUTH);
		}
		pack();
		addWindowListener(new CloseVisionAdapter(this));
		vision.getListener().addListener(this);
		(new StatusUpdateThread(this)).start();
		
		String name="VisionGUI"+(isRaw?".raw":"")+(isRLE?".rle":"")+".location";
		setLocation(prefs.getInt(name+".x",50),prefs.getInt(name+".y",50));

		setVisible(true);

	}
}
