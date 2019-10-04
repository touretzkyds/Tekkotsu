import java.net.ServerSocket;
import java.net.Socket;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;

public abstract class Listener implements Runnable {
	public Listener() { _port=-1; _isConnected=false; }
	public Listener(int port) { this(); setPort(port); }
	public Listener(String host, int port) { this(); setHostPort(host, port); }

	public void setPort(int port) {
		_isServer=true;
		_port=port;
		startThread();
	}

	public void setHostPort(String host, int port) {
		_isServer=false;
		_host=host;
		_port=port;
		startThread();
	}

	public void startThread() {
		destroy=false;
		_listenerThread=new Thread(this);
		_listenerThread.start();
	}

	public void run() {
		if (_port >= 0) {
			if (_isServer)
				runServer();
			else
				runConnect();
		} else {
			System.out.println("can't start Listener without [host],port");
		}
	}

	public void kill() {
		destroy=true;
		_isConnected=false;
		if(_listenerThread!=null)
			_listenerThread.interrupt();
		close();
	}

	public void frameTimer() {
		_frametimer_numframes++;
		if (System.currentTimeMillis()-_frametimer_timer>1000) {
			System.out.println("updated at "+_frametimer_numframes+"hz");
			_frametimer_numframes=0;
			_frametimer_timer=System.currentTimeMillis();
		}
	}

	public double readDouble(InputStream in) throws IOException {
		return Double.longBitsToDouble(readLong(in));
	}

	public void writeDouble(OutputStream out, double x) throws IOException {
		writeLong(out,Double.doubleToLongBits(x));
	}

	public long readLong(InputStream in) throws IOException {
		int read=0;
		int last=0;
		byte[] buf=new byte[8];
		while (read<8 && last>=0) { last=in.read(buf,read,8-read); read+=last; }
		if(last<0)
			_isConnected=false;
		return (b2l(buf[7])<<56) | (b2l(buf[6])<<48) |
					 (b2l(buf[5])<<40) | (b2l(buf[4])<<32) |
					 (b2l(buf[3])<<24) | (b2l(buf[2])<<16) |
					 (b2l(buf[1])<< 8) | b2l(buf[0]);
	}

	public void writeLong(OutputStream out, long x) throws IOException {
		int bytelen=8;
		byte[] buf=new byte[bytelen];
		for(int i=0; i<bytelen; i++)
			buf[i]=(new Long((x>>(8*i)) & 0xff)).byteValue();
		out.write(buf,0,bytelen);
	}

	public float readFloat(InputStream in) throws IOException {
		return Float.intBitsToFloat(readInt(in));
	}
	
	public void writeFloat(OutputStream out, float x) throws IOException {
		writeInt(out,Float.floatToIntBits(x));
	}

	public int readInt(InputStream in) throws IOException {
		int read=0;
		int last=0;
		byte[] buf=new byte[4];
		while (read<4 && last>=0) { last=in.read(buf,read,4-read); read+=last; }
		if(last<0)
			_isConnected=false;
		return (b2i(buf[3])<<24) | (b2i(buf[2])<<16) |
					 (b2i(buf[1])<< 8) | b2i(buf[0]);
	}
	
	public void writeInt(OutputStream out, int x) throws IOException {
		int bytelen=4;
		byte[] buf=new byte[bytelen];
		for(int i=0; i<bytelen; i++)
			buf[i]=(new Integer((x>>(8*i)) & 0xff)).byteValue();
		out.write(buf,0,bytelen);
	}

  public byte[] readBytes(InputStream in, int bytes) throws IOException {
    byte[] ret=new byte[bytes];
    readBytes(ret, in, bytes);
    return ret;
  }

	public void readBytes(byte[] buf, InputStream in, int bytes) throws IOException {
		int read=0;
		int last=0;
		while (read<bytes && last>=0) {
			last=in.read(buf, read, bytes-read);
			read+=last;
		}
		if(last<0)
			_isConnected=false;
	}

	public char readChar(InputStream in) throws IOException {
		return (char)in.read();
	}

	public void writeChar(OutputStream out, char c) throws IOException {
		out.write(c);
	}

	public String readLine(InputStream in) throws java.io.IOException{
		StringBuffer sbuf=new StringBuffer();
		int x=in.read();
		if(x==-1) {
			_isConnected=false;
			return sbuf.toString();
		}
		char c=(char)x;
		while(c!='\n') {
			sbuf.append(c);
			x=in.read();
			if(x==-1) {
				_isConnected=false;
				return sbuf.toString();
			}
			c=(char)x;
		}
		return sbuf.toString();
	}
	
	public int b2i(byte b) { return (b>=0)?(int)b:((int)b)+256; }
	public long b2l(byte b) { return (b>=0)?(long)b:((long)b)+256; }

	public abstract void runServer();
	public abstract void runConnect();
	public abstract void close();

	public boolean _isServer;
	public int _port;
	public String _host;
	public boolean _isConnected;
	public volatile Thread _listenerThread;
	public volatile boolean destroy=false;

	public int _frametimer_numframes=0;
	public long _frametimer_timer=System.currentTimeMillis();

  public static final int PACKET_TEXT=0;
  public static final int PACKET_VISIONRAW_HALF=1;
  public static final int PACKET_VISIONRAW_FULL=2;
  public static final int PACKET_VISIONRAW_YFULL_UVHALF=3;
  public static final int PACKET_VISIONRAW_Y_ONLY=4;
  public static final int PACKET_VISIONRAW_Y_LH_ONLY=5;
  public static final int PACKET_VISIONRAW_Y_HL_ONLY=6;
  public static final int PACKET_VISIONRAW_Y_HH_ONLY=7;
  public static final int PACKET_VISIONRAW_U_ONLY=8;
  public static final int PACKET_VISIONRAW_V_ONLY=9;
  public static final int PACKET_VISIONRLE_FULL=10;
  public static final int PACKET_WORLDSTATEJOINTS=11;
  public static final int PACKET_WORLDSTATEPIDS=12;
  public static final int PACKET_WORLDSTATEBUTTONS=13;
  public static final int PACKET_WMCLASS=14;
}
