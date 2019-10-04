package org.tekkotsu.mon;

import java.net.ServerSocket;
import java.net.Socket;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;

import java.util.ArrayList;
import java.util.List;
import java.io.ByteArrayOutputStream;

public abstract class Listener implements Runnable {
	public Listener() { _port=-1; _isConnected=false; }
	public Listener(int port) { this(); setPort(port); }
	public Listener(String host, int port) { this(); setHostPort(host, port); }

	protected long bytesRead = 0;
	protected long bytesWritten = 0;
	private boolean countersEnabled = true; 
  
	private static final ConnectionListener[] EMPTY_LISTENER_ARRAY =
		new ConnectionListener[0];
	private final List listeners = new ArrayList();
	private ConnectionListener[] cachedListeners = EMPTY_LISTENER_ARRAY;

	/**
	 * Notifies of connection state change.
	 */
	public interface ConnectionListener {
		/**
		 * Fires when connection has been established. 
		 */
		void onConnected();
		
		/**
		 * Fires when connection has been closed.
		 */
		void onDisconnected();
	}
	
	/**
	 * Adds a connection listener.
	 * 
	 * @param listener listener to add.
	 */
	public void addConnectionListener(ConnectionListener listener) {
		listeners.add(listener);
		cachedListeners = 
			(ConnectionListener[]) listeners.toArray(EMPTY_LISTENER_ARRAY);
		needConnection();
	}
	
	/**
	 * Removes a connection listener.
	 *
	 * @param listener listener to remove.
	 */
	public void removeConnectionListener(ConnectionListener listener) {
		while (listeners.remove(listener)) {}
		cachedListeners =
			(ConnectionListener[]) listeners.toArray(EMPTY_LISTENER_ARRAY);
	}
	
	/**
	 * Notifies listeners that the connection has been established. 
	 */
	protected void fireConnected() {
		for (int i = 0, len = cachedListeners.length; i < len; i++) {
			cachedListeners[i].onConnected();
		}
	}
	
	/**
	 * Notifies listeners that the connection has been closed. 
	 */
	protected void fireDisconnected() {
		for (int i = 0, len = cachedListeners.length; i < len; i++) {
			cachedListeners[i].onDisconnected();
		}
	}
	
	/**
	 * Counts the number of bytes read so far.
	 * 
	 * @return number of bytes read.
	 */
	public long getBytesRead() {
		return bytesRead;
	}
	
	/**
	 * Counts the number of bytes written so far.
	 * 
	 * @return number of bytes written.
	 */
	public long getBytesWritten() {
		return bytesWritten;
	}

	/**
	 * Enables/disables read/write counters.
	 * 
	 * @param enabled <code>true</code> to enable counters, <code>false</code> --
	 *				to disable.
	 */
	public void setReadWriteCountersEnabled(boolean enabled) {
		this.countersEnabled = enabled;
	}
	
	/**
	 * Checks whether read/write counters are enabled.
	 *
	 * @return <code>true</code> if the counters are enabled, <code>false</code>
	 *				 otherwise.
	 */
	public boolean isReadWriteCountersEnabled() {
		return countersEnabled;
	}
	
	public void setPort(int port) {
		_isServer=true;
		_port=port;
		startThread();
	}

	public void setHostPort(String host, int port) {
		_isServer=false;
		_host=host;
		_port=port;
		//spawning threads in the constructor (or functions called from the constructor) is a bad idea:
		//startThread(); // don't do it!
	}

	public void needConnection() {
		if(_listenerThread==null)
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
		_listenerThread=null;
	}

	public void kill() {
		destroy=true;
		_isConnected=false;
		if(_listenerThread!=null)
			_listenerThread.interrupt();
		close();
		_listenerThread=null;
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
		byte[] buf  = readBytes(in, 8);
		return (b2l(buf[7])<<56) | (b2l(buf[6])<<48) |
					 (b2l(buf[5])<<40) | (b2l(buf[4])<<32) |
					 (b2l(buf[3])<<24) | (b2l(buf[2])<<16) |
					 (b2l(buf[1])<< 8) | b2l(buf[0]);
	}

	public void writeLong(OutputStream out, long x) throws IOException {
		int bytelen=8;
		byte[] buf=new byte[bytelen];
		for(int i=0; i<bytelen; i++)
			buf[i]= (byte) ((x>>(8*i)) & 0xff);
		writeBytes(out, buf);
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
		byte[] buf=readBytes(in, 4);
		return (b2i(buf[3])<<24) | (b2i(buf[2])<<16) |
					 (b2i(buf[1])<< 8) | b2i(buf[0]);
	}

	public long readUnsignedInt(InputStream in) throws IOException {
		long ans=readInt(in);
		if(ans<0)
			ans+=(1L<<32);
		return ans;
	}
	
	public short readShort(InputStream in) throws IOException {
		int read=0;
		int last=0;
		byte[] buf=readBytes(in, 2);
		return (short) ((b2i(buf[1])<< 8) | b2i(buf[0]));
	}
	
	public void writeShort(OutputStream out, short x) throws IOException {
		int bytelen=2;
		byte[] buf=new byte[bytelen];
		for(int i=0; i<bytelen; i++)
			buf[i]= (byte) ((x>>(8*i)) & 0xff);
		writeBytes(out, buf);
	}
	
	public void writeInt(OutputStream out, int x) throws IOException {
		int bytelen=4;
		byte[] buf=new byte[bytelen];
		for(int i=0; i<bytelen; i++)
			buf[i]= (byte) ((x>>(8*i)) & 0xff);
		writeBytes(out, buf);
	}
	
	public void writeBytes(OutputStream out, byte[] buf) throws IOException {
		out.write(buf);
		if (isReadWriteCountersEnabled()) {
			bytesWritten += buf.length;
		}
	}

  public byte[] readBytes(InputStream in, int bytes) throws IOException {
    byte[] ret=new byte[bytes];
    readBytes(ret, in, bytes);
    return ret;
  }

	public void readBytes(byte[] buf, InputStream in, int bytes) throws IOException {
		int read=0;
		int last=0;
		while (read<bytes) {
			last=in.read(buf, read, bytes-read);
			if (last == -1) {
				_isConnected = false;
				break;
			}
			read+=last;
			if (isReadWriteCountersEnabled()) {
				bytesRead += last;
			}
		}
	}
	
	public byte readByte(InputStream in) throws IOException {
		final int value = in.read();
		if (value == -1) {
			throw new IOException("Failed to read: end of stream detected");
		}
		if (isReadWriteCountersEnabled()) {
			bytesRead++;
		}
		return (byte) value;
	}
	
	public void writeByte(OutputStream out, byte b) throws IOException {
		out.write(b);
		if (isReadWriteCountersEnabled()) {
			bytesWritten++;
		}
	}

	public char readChar(InputStream in) throws IOException {
		if (isReadWriteCountersEnabled()) {
			bytesRead++;
		}
		int c=in.read();
		if(c==-1)
			_isConnected=false;
		return (char)c;
	}

	public void writeChar(OutputStream out, char c) throws IOException {
		out.write(c);
		if (isReadWriteCountersEnabled()) {
			bytesWritten++;
		}
	}

	public String readLine(InputStream in) throws java.io.IOException{
		ByteArrayOutputStream bs = new ByteArrayOutputStream();
		int x=in.read();
		if(x==-1) {
			_isConnected=false;
			return "";
		}
		while((char)x!='\n') {
			bs.write(x);
			x=in.read();
			if(x==-1) {
				_isConnected=false;
				return new String(bs.toByteArray(),"UTF-8");
			}
			if (isReadWriteCountersEnabled()) {
				bytesRead++;
			}
		}
		return new String(bs.toByteArray(),"UTF-8");
	}
	
	public int b2i(byte b) { return (b>=0)?(int)b:((int)b)+256; }
	public long b2l(byte b) { return (b>=0)?(long)b:((long)b)+256; }

    //Convert a 4 byte int, located in an array with a given offset to an integer.
    public int byteToInt(byte[] buf, int offset) {
    return (b2i(buf[offset+3])<<24) | (b2i(buf[offset+2])<<16) |
           (b2i(buf[offset+1])<< 8) | b2i(buf[offset]);
    }

    
	protected abstract void runServer();
	protected abstract void runConnect();
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


