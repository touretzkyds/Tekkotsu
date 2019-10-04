package org.tekkotsu.mon;

import java.awt.event.*;
import javax.swing.*;
import java.lang.String;
import java.util.LinkedList;
import java.awt.*;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.image.IndexColorModel;
import java.awt.image.DataBuffer;
import java.util.Date;
import java.io.PrintWriter;
import java.io.FileOutputStream;

public class ImageSequenceWriterThread extends Thread implements VisionUpdatedListener {
	String dir;
	String pre;
	String post;
	String fmt;
	LinkedList imgBufs=new LinkedList();
	int dig;
	LinkedList q=new LinkedList();
	int count=0;
	VisionListener listen;
	boolean stopping=false;
	JButton but;
	boolean isYUVMode;
	boolean savePositions;

	final static int JPEG_ENTRY=1;
	final static int IMGBUF_ENTRY=2;
	final static int POSITION_ENTRY=3;

	public ImageSequenceWriterThread(VisionListener l, JButton reenableBut, boolean YUVmode, boolean positions) {
		super("ImageSequenceWriter");
		isYUVMode=YUVmode;
		int initbufsize=50;
		but=reenableBut;
		savePositions=positions;
		listen=l;
		l.addListener(this);
		l.startSensors();
		BufferedImage i=l.getImage();
		if(i.getColorModel().getTransferType()!=DataBuffer.TYPE_BYTE)
			for(int j=0;j<initbufsize;j++)
				imgBufs.add(new BufferedImage(i.getWidth(),i.getHeight(),i.getType()));
		else {
			for(int j=0;j<initbufsize;j++)
				imgBufs.add(new BufferedImage(i.getWidth(),i.getHeight(),i.getType(),(IndexColorModel)i.getColorModel()));
		}
	}

	public void setDirectory(String directory) {
		dir=directory;
		if(dir.charAt(dir.length()-1)!='/')
			dir+='/';
	}

	public void setName(String prepend, int digits, String append, String format) {
		pre=prepend;
		post=append;
		fmt=format;
		dig=digits;
	}

	public void visionUpdated(VisionListener l) {
		if(fmt==null)
			return;
		if(!isYUVMode && l.hasRawJPEG() && (fmt.equalsIgnoreCase("jpg") || fmt.equalsIgnoreCase("jpeg")))
			push(l.getJPEG(),l.getJPEGLen(),l.getTimeStamp(),l.getFrameNum());
		else
			push(l.getImage(),l.getTimeStamp(),l.getFrameNum());
	}
	public void sensorsUpdated(VisionListener l) {
		//System.out.println("Sensors: "+savePositions);
		if(savePositions) {
			synchronized(q) {
				String s=new String(l.getSensors());
				Date t;
				long frameNum;
				if(!s.startsWith("#POS")) {
					System.err.println("Malformed sensor reading");
					return;
				}
				//want to extract the timestamp and frame number,
				//but I don't really want to rewrite the file parsing in Java...
				//so this is a bit hackish, but should be reasonable
				int metaoff=s.lastIndexOf("<meta-info>"); //verbose mode
				if(metaoff!=-1) {
					int timeoff=s.indexOf("timestamp",metaoff);
					if(timeoff==-1) {
						System.err.println("Could not read meta-info timestamp in sensor dump");
						return;
					}
					timeoff+=9;
					int timeend=timeoff;
					while(s.charAt(timeend)!='\r' && s.charAt(timeend)!='\n')
						timeend++;
					t=new Date(Long.parseLong(s.substring(timeoff,timeend).trim()));
					
					int frameoff=s.indexOf("framenumber",metaoff);
					if(frameoff==-1) {
						System.err.println("Could not read meta-info frame info in sensor dump");
						return;
					}
					frameoff+=11;
					int frameend=frameoff;
					while(s.charAt(frameend)!='\r' && s.charAt(frameend)!='\n')
						frameend++;
					frameNum=Long.parseLong(s.substring(frameoff,frameend).trim());
				} else {
					metaoff=s.lastIndexOf("meta-info"); //condensed mode
					if(metaoff==-1) {
						System.err.println("Could not find meta-info section in sensor dump");
						return;
					}
					metaoff+=9;
					while(s.charAt(metaoff)!='=')
						metaoff++;
					metaoff++;
					int metaend=metaoff;
					while(s.charAt(metaend)!='\r' && s.charAt(metaend)!='\n')
						metaend++;
					String[] sa=s.substring(metaoff,metaend).trim().split(" ");
					t=new Date(Long.parseLong(sa[0]));
					frameNum=Long.parseLong(sa[1]);
				}
				q.addLast(new Integer(POSITION_ENTRY));
				q.addLast(s);
				q.addLast(t);
				q.addLast(new Long(frameNum));
			}
		}
	}

	public void push(byte[] jpeg, int len, Date t, long frameNum) {
		byte[] jpegcopy = new byte[len];
		for(int i=0; i<len; i++)
			jpegcopy[i]=jpeg[i];
		synchronized(q) {
			q.addLast(new Integer(JPEG_ENTRY));
			q.addLast(jpegcopy);
			q.addLast(t);
			q.addLast(new Long(frameNum));
		}
	}

	public void push(BufferedImage i, Date t, long frameNum) {
		BufferedImage i2;
		if(imgBufs.size()>0)
			i2=(BufferedImage)imgBufs.removeFirst();
		else {
			if(i.getColorModel().getTransferType()!=BufferedImage.TYPE_BYTE_INDEXED)
				i2=new BufferedImage(i.getWidth(),i.getHeight(),i.getType());
			else
				i2=new BufferedImage(i.getWidth(),i.getHeight(),i.getType(),(IndexColorModel)i.getColorModel());
		}
		synchronized(i) {
			i.copyData(i2.getRaster());
		}
		synchronized(q) {
			q.addLast(new Integer(IMGBUF_ENTRY));
			q.addLast(i2);
			q.addLast(t);
			q.addLast(new Long(frameNum));
		}
	}

	public void run() {
		String tmp=null;
		PrintWriter log=null;
		try {
			log=new PrintWriter(new FileOutputStream(dir+pre+post+".txt"));
		} catch(Exception ex) { ex.printStackTrace(); return; }
		Date start=null;
		long startFrame=0;
		long startSensor=0;
		try {
			while(true) {
				while(q.size()>0) {
					if(interrupted()) {
						if(!stopping) {
							stopping=true;
							if(but!=null) {
								but.setEnabled(false);
								tmp=but.getText();
								but.setText("Writing...");
							}
							listen.removeListener(this);
						} else { //second interrupt... die!
							break;
						}
					}
					BufferedImage i=null;
					byte[] jpeg=null;
					String sensors=null;
					Date t;
					long num;
					int entryType;
					synchronized(q) {
						entryType=((Integer)q.removeFirst()).intValue();
						if(entryType==JPEG_ENTRY) { //is a jpeg
							jpeg=(byte[])q.removeFirst();
						} else if(entryType==IMGBUF_ENTRY) { //is a BufferedImage
							i=(BufferedImage)q.removeFirst();
						} else if(entryType==POSITION_ENTRY) { //is a sensor reading
							sensors=(String)q.removeFirst();;
						}
						t=(Date)q.removeFirst();
						num=((Long)q.removeFirst()).longValue();
					}
					if(start==null) {
						if(entryType==POSITION_ENTRY)
							continue;
						start=t;
						startFrame=num;
						log.println("First frame "+startFrame+" timestamp: "+start.getTime());
					}
					if(startSensor==0 && entryType==POSITION_ENTRY)
						startSensor=num;
					long dt=t.getTime()-start.getTime();
					long df=num-(entryType==POSITION_ENTRY?startSensor:startFrame);
					String name=getFileName(dt,df,sensors!=null);
					if(log!=null)
						log.println(name+"\t"+count+"\t"+df+"\t"+dt);
					FileOutputStream fileout=new FileOutputStream(dir+name);
					if(i!=null) {
						ImageIO.write(i,fmt,fileout);
						imgBufs.addLast(i);
					} else if(jpeg!=null) {
						fileout.write(jpeg);
					} else if(sensors!=null) {
						fileout.write(sensors.getBytes());
					}
					count++;
				}
				if(stopping)
					break;
				sleep(1000/30);
			}
		} catch(Exception ex) {
			listen.removeListener(this);
			if(!(ex instanceof InterruptedException))
				ex.printStackTrace();
		}
		listen.stopSensors();
		if(log!=null)
			log.close();
		if(but!=null && tmp!=null) {
			but.setEnabled(true);
			if(but.getText().compareTo("Writing...")==0)
				but.setText(tmp);
		}
	}

	protected String getFileName(long dt,long num,boolean isSensor) {
		String ans=pre;
		long c=num;
		int digits=dig;
		if(digits==0) {
			c=dt;
			digits=6;
		}
		for(int s=(int)Math.pow(10,digits-1); s>=1; s/=10) {
			ans+=c/s;
			c-=(c/s)*s;
		}
		ans+=post+"."+(isSensor?"pos":fmt);
		return ans;
	}
}
