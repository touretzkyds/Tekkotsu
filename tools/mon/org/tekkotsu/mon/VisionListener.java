package org.tekkotsu.mon;

import java.awt.image.BufferedImage;
import java.util.Vector;
import java.util.Date;

import java.io.*;
import java.net.*;
import java.awt.image.BufferedImage;
import java.awt.image.Raster;
import java.awt.image.IndexColorModel;
import java.util.Date;
import javax.imageio.ImageIO;
import javax.imageio.ImageReader;
import javax.imageio.stream.ImageInputStream;
import javax.imageio.stream.MemoryCacheImageInputStream;

public interface VisionListener {
	public static final int ENCODE_COLOR=0;
	public static final int ENCODE_SINGLE_CHANNEL=1;
        public static final int ENCODE_DEPTH=2;
	public static final int COMPRESS_NONE=0;
	public static final int COMPRESS_JPEG=1;
	public static final int CHAN_Y=0;
	public static final int CHAN_U=1;
	public static final int CHAN_V=2;
	public static final int CHAN_Y_DY=3;
	public static final int CHAN_Y_DX=4;
	public static final int CHAN_Y_DXDY=5;

	static ImageReader jpegReader=(ImageReader)ImageIO.getImageReadersByFormatName("jpeg").next();

	final static int DEFAULT_WIDTH=176;
	final static int DEFAULT_HEIGHT=144;
	static int defRawPort=10011;
	static int defRLEPort=10012;
        static int defRegionPort=10013;
        static int defDepthPort=10014;

	public void addListener(VisionUpdatedListener l);
	public void removeListener(VisionUpdatedListener l);

	void addConnectionListener(Listener.ConnectionListener listener);
	void removeConnectionListener(Listener.ConnectionListener listener);

	void fireVisionUpdate();
	public boolean isConnected();

	public void startSensors();
	public void stopSensors();

	public Date getTimeStamp();
	public long getFrameNum();
	public IndexColorModel getColorModel();

	public boolean hasRawJPEG();
	public byte[] getJPEG();
	public int getJPEGLen();
	
	public byte[] getData();
	public String getSensors();
	public void setConvertRGB(boolean b);
	public boolean getConvertRGB();

	public int[] getYUVPixels();

	//This still uses RGB pixels just in case you want to display the
	//intensity value into hues instead of black/white
	public int[] getIntensityPixels();

	public int[] getRGBPixels();

	public BufferedImage getImage();

	public void needConnection();
	public void startThread();
	public void kill();
	public void run();
	public void close();
	public void setPort(int port);
	public void setHostPort(String host, int port);

	public long getBytesRead(); 
}

