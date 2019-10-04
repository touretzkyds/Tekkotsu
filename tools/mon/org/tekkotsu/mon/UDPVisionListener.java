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

public class UDPVisionListener extends UDPListener implements VisionListener {
  DatagramSocket mysock;
	
  InputStream in;

	boolean updatedFlag;
	Date timestamp;
	long frameNum=0;

	protected Vector listeners = new Vector();
	protected boolean updating;
	
	public void addListener(VisionUpdatedListener l) { listeners.add(l); needConnection(); }
	public void removeListener(VisionUpdatedListener l) { listeners.remove(l); }
	public void fireVisionUpdate() {
		updating=true;
		for(int i=0; i<listeners.size() && updating; i++)
			((VisionUpdatedListener)listeners.get(i)).visionUpdated(this);
		updating=false;
	}
	public void fireSensorUpdate() {
		updating=true;
		for(int i=0; i<listeners.size() && updating; i++)
			((VisionUpdatedListener)listeners.get(i)).sensorsUpdated(this);
		updating=false;
	}

	public Date getTimeStamp() { return timestamp; }
	public long getFrameNum() { return frameNum; }
	public IndexColorModel getColorModel() { return cmodel; }

	public boolean hasData() {
		return updatedFlag;
	}
 
	public boolean isConnected() {
		return _isConnected;
	}

	int channels=3;
	int width=DEFAULT_WIDTH;
	int height=DEFAULT_HEIGHT;
	int pktSize=width*height*channels;
	int oldformat=PACKET_VISIONRAW_FULL;
	int format;
	int compression;
	int chan_id;

	byte[] _data=new byte[pktSize];
	byte[] _outd=new byte[pktSize];
	byte[] _tmp=new byte[pktSize*2];
	byte[] _jpeg=new byte[pktSize*2];
	byte[] _newjpeg=new byte[pktSize*2];
	String sensors;
	boolean isJPEG=false;
	int jpegLen=0;
	int newjpegLen=0;
	boolean isIndex=false;
	boolean badCompressWarn=false;
	int[] _pixels=new int[width*height];
	BufferedImage img=new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);
	int bytesRead;
	boolean convertRGB=true;
	IndexColorModel cmodel;

	public String getSensors() { return sensors; }

	public boolean hasRawJPEG() { return isJPEG; }
	public byte[] getJPEG() { return _jpeg; }
	public int getJPEGLen() { return jpegLen; }
	
	public String readLoadSaveString(InputStream in) throws java.io.IOException {
		int creatorLen=readInt(in);
		if(!_isConnected) return ""; //System.out.println("creatorLen=="+creatorLen);
		String creator=new String(readBytes(in,creatorLen));
		if(!_isConnected) return "";
		if(readChar(in)!='\0') {
			System.err.println("Misread LoadSave string? len="+creatorLen+" str="+creator);
			Throwable th=new Throwable();
			th.printStackTrace();
		}
		return creator;
	}

	public boolean readChannel(InputStream in, int c, int chanW, int chanH) throws java.io.IOException {
		readBytes(_tmp,in,chanW*chanH);
		if(!_isConnected) return false;
		return upsampleData(c,chanW,chanH);
	}
	public boolean upsampleData(int c, int chanW, int chanH) {
		if(chanW==width && chanH==height) {
			//special case: straight copy if image and channel are same size
			for(int y=0;y<height;y++) {
				int datarowstart=y*width*channels+c;
				int tmprowstart=y*chanW;
				for(int x=0;x<width;x++)
					_data[datarowstart+x*channels]=_tmp[tmprowstart+x];
			}
			return true;
		}
		//otherwise this channel is subsampled, need to blow it up
		//we'll linearly interpolate between pixels
		//METHOD A:
		//hold edges, interpolate through middle:
		//  if we have 2 samples, scaling up to 4
		//   index: 0   1    2   3
		// maps to: 0  1/3  2/3  1
		/*
		float xsc=(chanW-1)/(float)(width-1);
		float ysc=(chanH-1)/(float)(height-1);
		for(int y=0;y<height-1;y++) {
			int datarowstart=y*width*channels+c;
			float ty=y*ysc;
			int ly=(int)ty; //lower pixel index
			float fy=ty-ly; //upper pixel weight
			int tmprowstart=ly*chanW;
			for(int x=0;x<width-1;x++) {
				float tx=x*xsc;
				int lx=(int)tx; //lower pixel index
				float fx=tx-lx; //upper pixel weight

				float lv=((int)_tmp[tmprowstart+lx]&0xFF)*(1-fx)+((int)_tmp[tmprowstart+lx+1]&0xFF)*fx;
				float uv=((int)_tmp[tmprowstart+lx+chanW]&0xFF)*(1-fx)+((int)_tmp[tmprowstart+lx+1+chanW]&0xFF)*fx;
				_data[datarowstart+x*channels]=(byte)(lv*(1-fy)+uv*fy);
			}
			_data[datarowstart+(width-1)*channels]=_tmp[tmprowstart+chanW-1];
		}
		int datarowstart=width*(height-1)*channels+c;
		int tmprowstart=chanW*(chanH-1);
		for(int x=0;x<width-1;x++) {
			float tx=x*xsc;
			int lx=(int)tx; //lower pixel index
			float fx=tx-lx; //upper pixel weight
			_data[datarowstart+x*channels]=(byte)(((int)_tmp[tmprowstart+lx]&0xFF)*(1-fx)+((int)_tmp[tmprowstart+lx+1]&0xFF)*fx);
		}
		_data[datarowstart+(width-1)*channels]=_tmp[tmprowstart+chanW-1];
		*/
		
		//Unfortunately, pixels are simply interleaved, starting at the
		//top right.  So, Method A will stretch things to the bottom-right
		//a bit.  This method holds left edge and spacing, so it lines up
		//better with what's being transmitted (but the bottom right edges
		//wind up smeared)
		//METHOD B:
		//  if we have 2 samples, scaling up to 4
		//   index: 0   1    2   3
		// maps to: 0  1/2   1   1  <-- this last one would be 3/2, so we have to replicate 1
		float xsc=(chanW)/(float)(width);
		float ysc=(chanH)/(float)(height);
		int xgap=Math.round(1.0f/xsc);
		int ygap=Math.round(1.0f/ysc);
		for(int y=0;y<height-ygap;y++) {
			int datarowstart=y*width*channels+c;
			float ty=y*ysc;
			int ly=(int)ty; //lower pixel index
			float fy=ty-ly; //upper pixel weight
			int tmprowstart=ly*chanW;
			for(int x=0;x<width-xgap;x++) {
				float tx=x*xsc;
				int lx=(int)tx; //lower pixel index
				float fx=tx-lx; //upper pixel weight

				float lv=(_tmp[tmprowstart+lx]&0xFF)*(1-fx)+(_tmp[tmprowstart+lx+1]&0xFF)*fx;
				float uv=(_tmp[tmprowstart+lx+chanW]&0xFF)*(1-fx)+(_tmp[tmprowstart+lx+1+chanW]&0xFF)*fx;
				_data[datarowstart+x*channels]=(byte)(lv*(1-fy)+uv*fy);
			}
			for(int x=width-xgap;x<width;x++) {
				float lv=(_tmp[tmprowstart+chanW-1]&0xFF);
				float uv=(_tmp[tmprowstart+chanW-1+chanW]&0xFF);
				_data[datarowstart+x*channels]=(byte)(lv*(1-fy)+uv*fy);
			}
		}
		for(int y=height-ygap;y<height;y++) {
			int datarowstart=y*width*channels+c;
			int tmprowstart=chanW*(chanH-1);
			for(int x=0;x<width-xgap;x++) {
				float tx=x*xsc;
				int lx=(int)tx; //lower pixel index
				float fx=tx-lx; //upper pixel weight

				float lv=(_tmp[tmprowstart+lx]&0xFF)*(1-fx)+(_tmp[tmprowstart+lx+1]&0xFF)*fx;
				_data[datarowstart+x*channels]=(byte)(lv);
			}
			for(int x=width-xgap;x<width;x++)
				_data[datarowstart+x*channels]=_tmp[tmprowstart+chanW-1];
		}
		
		return true;
	}

	public boolean readJPEGChannel(InputStream in, int c, int chanW, int chanH) throws java.io.IOException {
		int len=readInt(in);
		newjpegLen=len;
		//System.out.println("len="+len);
		if(!_isConnected) return false;
		if(len>=_newjpeg.length) {
			System.out.println("Not enough tmp room");
			return false;
		}
		readBytes(_newjpeg,in,len);
		if(!_isConnected) return false;
		if(len>chanW*chanH*channels) {
			if(!badCompressWarn) {
				badCompressWarn=true;
				System.out.println("Compressed image is larger than raw would be... :(");
			}
		} else {
			if(badCompressWarn) {
				badCompressWarn=false;
				System.out.println("...ok, compressed image is smaller than raw now... :)");
			}
		}

		try {
			ImageInputStream jpegStream=new MemoryCacheImageInputStream(new ByteArrayInputStream(_newjpeg));
			jpegReader.setInput(jpegStream); 
			Raster decoded=jpegReader.readRaster(0,null);
			int off=c;
			for(int y=0; y<chanH; y++)
				for(int x=0; x<chanW; x++) {
					_data[off]=(byte)decoded.getSample(x,y,0);
					off+=channels;
				}
		} catch(Exception ex) { ex.printStackTrace(); }
		return true;
	}

	public boolean readJPEG(InputStream in, int chanW, int chanH) throws java.io.IOException {
		int len=readInt(in);
		newjpegLen=len;
		//System.out.println("len="+len);
		if(!_isConnected) return false;
		if(len>=_newjpeg.length) {
			System.out.println("Not enough tmp room");
			return false;
		}
		readBytes(_newjpeg,in,len);
		if(!_isConnected) return false;
		if(len>chanW*chanH*channels) {
			if(!badCompressWarn) {
				badCompressWarn=true;
				System.out.println("Compressed image is larger than raw would be... :(");
			}
		} else {
			if(badCompressWarn) {
				badCompressWarn=false;
				System.out.println("...ok, compressed image is smaller than raw now... :)");
			}
		}

		try {
			ImageInputStream jpegStream=new MemoryCacheImageInputStream(new ByteArrayInputStream(_newjpeg));
			jpegReader.setInput(jpegStream); 
			Raster decoded=jpegReader.readRaster(0,null);
			int off=0;
			for(int y=0; y<chanH; y++)
				for(int x=0; x<chanW; x++) {
					_data[off++]=(byte)decoded.getSample(x,y,0);
					_data[off++]=(byte)decoded.getSample(x,y,1);
					_data[off++]=(byte)decoded.getSample(x,y,2);
				}
		} catch(Exception ex) { ex.printStackTrace(); }
		return true;
	}

	byte[] colormap = new byte[256*3];
	public boolean readColorModel(InputStream in) throws java.io.IOException {
		int len=readInt(in);
		//System.out.println("len="+len);
		if(!_isConnected) return false;
		readBytes(colormap,in,len*3);
		if(!_isConnected) return false;
		//we'll do this stupid thing because we can't change an existing color model, and we don't want to make a new one for each frame
		// (btw, java is stupid)
		boolean makeNew=false;
		if(cmodel==null || len!=cmodel.getMapSize()) {
			makeNew=true;
		} else {
			int off=0;
			for(int i=0; i<len; i++) {
				if((byte)cmodel.getRed(i)!=colormap[off++] || (byte)cmodel.getGreen(i)!=colormap[off++] || (byte)cmodel.getBlue(i)!=colormap[off++]) {
					makeNew=true;
					break;
				}
			}
		}
		if(makeNew) {
			//System.out.println("new color model");
			cmodel=new IndexColorModel(7,len,colormap,0,false);
		}
		return true;
	}
	
	public boolean readIndexedColor(InputStream in, int chanW, int chanH) throws java.io.IOException {
		readBytes(_data,in,chanW*chanH);
		if(!_isConnected) return false;
		if(!readColorModel(in)) return false;
		if(!_isConnected) return false;
		isIndex=true;
		return true;
	}

	public boolean readRLE(InputStream in, int chanW, int chanH) throws java.io.IOException {
		int len=readInt(in);
		if(!_isConnected) return false;
		readBytes(_tmp,in,len*5);
		if(!_isConnected) return false;

		int dpos=0;
		int curx=0, cury=0;
		for (; len>0 && cury<chanH;) {
			byte color=_tmp[dpos++];
			int x=((int)_tmp[dpos++]&0xFF);
			x|=((int)_tmp[dpos++]&0xFF)<<8;
			int rlen=((int)_tmp[dpos++]&0xFF);
			rlen|=((int)_tmp[dpos++]&0xFF)<<8;
			//System.out.println(color + " "+x + " "+rlen);
			len--;
			if (x < curx)
				return false;
			
			for (; curx < x; curx++)
				_data[cury*width+curx]=0;
			
			if (curx+rlen>width)
				return false;
			
			for (; rlen>0; rlen--, curx++)
				_data[cury*width+curx]=color;
			if (curx==width) {
				cury++;
				curx=0;
			}
		}
		if(!readColorModel(in)) return false;
		if(!_isConnected) return false;
		isIndex=true;
		return true;
	}

	
	public boolean readRegions(InputStream in, int chanW, int chanH) throws java.io.IOException {
		//clear the _data array
		for(int h=0; h<chanH; h++) {
			for(int w =0; w<chanW; w++) {
				_data[chanW*h+w]=0;
			}
		}
  
		int numColors=readInt(in);
		if(!_isConnected) return false;
		for(int curColor = 0; curColor < numColors; curColor++) {
			int numRegions = readInt(in);
			if(!_isConnected) return false;
            
			readBytes(_tmp,in,36*numRegions);
			if(!_isConnected) return false;
            	
			int dpos=0;
			for (int i = 0; i<numRegions; i++) {
				byte color= _tmp[dpos];
				dpos +=4;
				int x1 = byteToInt(_tmp,dpos);
				dpos +=4;
				int y1 = byteToInt(_tmp,dpos);
				dpos +=4;
				int x2 = byteToInt(_tmp,dpos);
				dpos +=4;
				int y2 = byteToInt(_tmp,dpos);
				//The  data of the centroid, area and run_start are ignored
				dpos +=20; //isn't nessescary, but now it nicely adds up to 36
				if (  x1 > chanW || y1 > chanH || x2 > chanW || y2 > chanH
							|| x1 > x2 || y1 > y2
							|| x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0
							)
					return false;
				
				//Fill the data array with the bounding boxes..
				//..the top and bottom lines
				for (int tb = x1; tb <= x2; tb++) {
					_data[y1*width+tb]=color;
					_data[y2*width+tb]=color;
				}
				//..the left and right lines
				for (int lr = y1; lr <= y2; lr++) {
					_data[lr*width+x1]=color;
					_data[lr*width+x2]=color;
				}
			}
			readBytes(_tmp,in,12); //read out the min_area, total_area and merge_threshhold and ignore them:)
		}
		if(!readColorModel(in)) return false;
		if(!_isConnected) return false;
		isIndex=true;
		return true;
	}


	public void connected(DatagramSocket UDPsocket, DatagramPacket incoming) {
		_isConnected=true;
		mysock = UDPsocket;
		while(listeners==null) {
			System.out.println("Assert: Bad race condition -- shouldn't be happening");
			Thread.yield();
		}
		try {
			fireConnected();
			while(!mysock.isClosed()) {
				in = new ByteArrayInputStream(incoming.getData());
				String type = readLoadSaveString(in);
				if(!_isConnected) break; //System.out.println("Got type="+type);
				if(!type.equals("TekkotsuImage")) {
					if(type.startsWith("#POS\n")) {
						sensors=type;
						//System.out.println("got sensors");
						fireSensorUpdate();
						if(destroy)
							break;
						mysock.receive(incoming);
						continue;
					} else if(!type.equals("CloseConnection"))
						System.err.println("Unrecognized type: "+type);
					else {
						System.out.println("Got Close connection packet");
						_isConnected=false;
					}
					break;
				}
				format=readInt(in);
				if(!_isConnected) break; //System.out.println("Got format="+format);
				compression=readInt(in);
				if(!_isConnected) break; //System.out.println("Got compression="+compression);
				int newWidth=readInt(in);
				if(!_isConnected) break; //System.out.println("Got newWidth="+newWidth);
				int newHeight=readInt(in);
				if(!_isConnected) break; //System.out.println("Got newHeight="+newHeight);
				long timest=readInt(in);
				if(timest<0)
					timest+=(1L<<32);
				if(!_isConnected) break; //System.out.println("Got timest="+timest);
				frameNum=readInt(in);
				if(frameNum<0)
					frameNum+=(1L<<32);
				if(!_isConnected) break; //System.out.println("Got frameNum="+frameNum);
				
				if (format!=oldformat || newWidth!=width || newHeight!=height) {
					width=newWidth;
					height=newHeight;
					synchronized (_outd) {
						switch (format) {
						case ENCODE_COLOR:
							channels=3;
							pktSize=width*height*channels;
							break;
						case ENCODE_SINGLE_CHANNEL:
							channels=1;
							pktSize=width*height*channels;
							break;
						default:
							System.err.println("VisionRawListener: unknown packet type "+format);
							throw new java.lang.NoSuchFieldException("fake exception");
						}
						_data=new byte[pktSize];
						_outd=new byte[pktSize];
						_tmp=new byte[pktSize];
						_jpeg=new byte[pktSize*2<2000?2000:pktSize*2];
						_newjpeg=new byte[pktSize*2<2000?2000:pktSize*2];
						_pixels=new int[width*height];;
						img=new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);
						oldformat=format;
					}
				}
				
				boolean failed=false;
				for(int i=0; i<channels; i++) {
					String creator = readLoadSaveString(in);
					if(!_isConnected) break; //System.out.println("Got creator="+creator);
					if(!creator.equals("FbkImage")) {
						System.err.println("Unrecognized creator: "+creator);
						failed=true; break;
					} else {
						int chanwidth=readInt(in);
						if(!_isConnected) break; //System.out.println("Got chanwidth="+chanwidth);
						int chanheight=readInt(in);
						if(!_isConnected) break; //System.out.println("Got chanheight="+chanheight);
						
						if(chanwidth>width || chanheight>height) {
							System.err.println("channel dimensions exceed image dimensions");
							failed=true; break;
						}
						
						int layer=readInt(in);
						if(!_isConnected) break; //System.out.println("Got layer="+layer);
						chan_id=readInt(in);
						if(!_isConnected) break; //System.out.println("Got chan_id="+chan_id);
				
						String fmt=readLoadSaveString(in);
						if(!_isConnected) break; //System.out.println("Got fmt="+fmt);
						if(fmt.equals("blank")) {
							isJPEG=false;
							isIndex=false;
							int useChan=(channels==1)?i:chan_id;
							int off=useChan;
							for(int y=0; y<height; y++)
								for(int x=0; x<width; x++) {
									_data[off]=(byte)(convertRGB?0x80:0);
									off+=channels;
								}
						} else if(fmt.equals("RawImage")) {
							isJPEG=false;
							isIndex=false;
							int useChan=(channels==1)?i:chan_id;
							if(!readChannel(in,useChan,chanwidth,chanheight)) { failed=true; System.err.println("UDPVisionListener channel read failed"); break; }
						} else if(fmt.equals("JPEGGrayscale")) {
							isIndex=false;
							int useChan=(channels==1)?i:chan_id;
							if(!readJPEGChannel(in,useChan,chanwidth,chanheight)) { failed=true; System.err.println("UDPVisionListener JPEGGreyscale channel read failed"); break; }
							isJPEG=(channels==1);
						} else if(fmt.equals("JPEGColor")) {
							isIndex=false;
							if(format==ENCODE_SINGLE_CHANNEL)
								System.err.println("WTF? ");
							if(!readJPEG(in,chanwidth,chanheight)) { failed=true; System.err.println("UDPVisionListener JPEGColor channel read failed"); break; }
							i=channels;
							isJPEG=true;
						} else if(fmt.equals("SegColorImage")) {
							isJPEG=false;
							isIndex=true;
							if(!readIndexedColor(in,chanwidth,chanheight)) { failed=true; System.err.println("UDPVisionListener SegColor read failed"); break; }
						} else if(fmt.equals("RLEImage")) {
							isJPEG=false;
							isIndex=true;
							if(!readRLE(in,chanwidth,chanheight)) { failed=true; System.err.println("UDPVisionListener RLEImage read failed"); break; }
						} else if(fmt.equals("RegionImage")) {
							isJPEG=false;
							isIndex=true;
							if(!readRegions(in,chanwidth,chanheight)) { failed=true; System.err.println("UDPVisionListener RegionImage read failed"); break; }
						} else {
							isJPEG=false;
							isIndex=false;
							System.err.println("Unrecognized format: "+fmt);
							failed=true; break;
						}
					}
				}
				if(failed || !_isConnected) {
					System.err.println("UDPVisionListener connection lost");
					break;
				}
				
				synchronized(_outd) {
					byte[] temp=_data;
					_data=_outd;
					_outd=temp;
					if(isJPEG) {
						temp=_newjpeg;
						_newjpeg=_jpeg;
						_jpeg=temp;
						int tempi=newjpegLen;
						newjpegLen=jpegLen;
						jpegLen=tempi;
					}
					timestamp=new Date(timest);
				}
				updatedFlag=true;
				fireVisionUpdate();
				if(destroy) //close requested
					close(); //try to request final sensor frame again (might've dropped packet)
				mysock.receive(incoming);
			}
		} catch (Exception ex) {
    } finally {
      fireDisconnected();
    }
		
		try { mysock.close(); } catch (Exception ex) { }
		if(!_isConnected) {
			if(!destroy)
				System.out.println("Waiting 5 seconds to reconnect");
			try { Thread.sleep(5000); } catch (Exception ex) {}
		}
		_isConnected=false;
		fireVisionUpdate();
	}
	
	public byte[] getData() {
		synchronized (_outd) {
			updatedFlag=false;
			return _outd;
		}
	}

	public void setConvertRGB(boolean b) { 
		if(b!=convertRGB) {
			convertRGB=b;
			updatedFlag=true;
		}
	}
	public boolean getConvertRGB() { return convertRGB; }

	public int[] getYUVPixels() {
		synchronized(_outd) {
			byte[] data=getData();
			int offset=0;
			for (int i=0; i<width*height; i++) {
				int y=(int)data[offset++]&0xFF;
				int u=(int)data[offset++]&0xFF;
				int v=(int)data[offset++]&0xFF;
				_pixels[i]=(255<<24) | (y<<16) | (u<<8) | v;
			}
		}
		return _pixels;
	}

	//This still uses RGB pixels just in case you want to display the
	//intensity value into hues instead of black/white
	public int[] getIntensityPixels() {
		synchronized(_outd) {
			byte[] data=getData();
			int offset=0;
			if(!getConvertRGB()) {
				for (int i=0; i<width*height; i++) {
					int z=(int)data[offset++]&0xFF;
					_pixels[i]=(255<<24) | (z<<16) | (z<<8) | z;
				}
			} else if(chan_id==CHAN_Y || chan_id==CHAN_Y_DY || chan_id==CHAN_Y_DX || chan_id==CHAN_Y_DXDY ) {
				for (int i=0; i<width*height; i++) {
					int z=(int)data[offset++]&0xFF;
					_pixels[i]=pixelYUV2RGB(z,0x80,0x80);
				}
			} else if(chan_id==CHAN_U) {
				for (int i=0; i<width*height; i++) {
					int z=(int)data[offset++]&0xFF;
					_pixels[i]=pixelYUV2RGB(0x80,z,0x80);
				}
			} else if(chan_id==CHAN_V) {
				for (int i=0; i<width*height; i++) {
					int z=(int)data[offset++]&0xFF;
					_pixels[i]=pixelYUV2RGB(0x80,0x80,z);
				}
			}
		}		
		return _pixels;
	}

	public int[] getRGBPixels() {
		synchronized(_outd) {
			byte[] data=getData();
			int offset=0;
			for (int i=0; i<width*height; i++) {
				int y=(int)data[offset++]&0xFF;
				int u=(int)data[offset++]&0xFF;
				int v=(int)data[offset++]&0xFF;
				_pixels[i]=pixelYUV2RGB(y, u, v);
			}
		}
		return _pixels;
	}

	static final int pixelYUV2RGB(int y, int u, int v) {
		u=u*2-255;
		v=v-128;
		int b=y+u;
		int r=y+v;
		v=v>>1;
		u=(u>>2)-(u>>4);
		int g=y-u-v;
		if (r<0) r=0; if (g<0) g=0; if (b<0) b=0;
		if (r>255) r=255; if (g>255) g=255; if (b>255) b=255;
		return (255<<24) | (r<<16) | (g<<8) | b;
	}

	public BufferedImage getImage() {
		if(!updatedFlag)
			return img;
		if(isIndex) {
			byte[] data=getData();
			if(cmodel==null)
				return img;
			if(img.getWidth()!=width || img.getHeight()!=height || img.getType()!=BufferedImage.TYPE_BYTE_INDEXED)
				img=new BufferedImage(width,height,BufferedImage.TYPE_BYTE_INDEXED,cmodel);
			img.getRaster().setDataElements(0,0,width,height,data);
		} else {
			int[] pix;
			if(img.getWidth()!=width || img.getHeight()!=height || img.getType()!=BufferedImage.TYPE_INT_RGB)
				img=new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);
			synchronized(_outd) {
				if(format==ENCODE_COLOR)
					pix=getConvertRGB()?getRGBPixels():getYUVPixels();
				else
					pix=getIntensityPixels();
				img.setRGB(0,0,width,height,pix,0,width);
			}
		}
		return img;
	}

	public void close() {
		if(mysock==null || mysock.isClosed())
			return;
		destroy=true;
		byte[] buf = (new String("refreshSensors\n")).getBytes();
		try {
			mysock.send(new DatagramPacket(buf,buf.length));
		} catch(Exception e) { e.printStackTrace(); }
		//wait until the final sensors comes in so if user hits save image we can save the corresponding sensors too
	}

	public void startSensors() {
		byte[] buf = (new String("startSensors\n")).getBytes();
		try {
			mysock.send(new DatagramPacket(buf,buf.length));
		} catch(Exception e) { e.printStackTrace(); }
	}

	public void stopSensors() {
		byte[] buf = (new String("stopSensors\n")).getBytes();
		try {
			mysock.send(new DatagramPacket(buf,buf.length));
		} catch(Exception e) { e.printStackTrace(); }
	}

	public UDPVisionListener() { super(); }
	public UDPVisionListener(int port) { super(port); }
	public UDPVisionListener(String host, int port) { super(host,port); }
}
