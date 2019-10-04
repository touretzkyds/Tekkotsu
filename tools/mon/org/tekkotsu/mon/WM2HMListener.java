package org.tekkotsu.mon;

// A Listener for the WorldModel2 height map data
import java.io.InputStream;
import java.net.Socket;

// Much of this code has been copied from WorldStateJointsListener.java.
// I'm not too certain how it works, but I think a fair part of it has to do
// with synchronization.
public class WM2HMListener extends TCPListener {
  boolean _updatedFlag=false;
  WM2HMData _data;
  WM2HMData _outd;

  // Evidently this is the meat of the class
  public void connected(Socket socket) {
    _isConnected = true;
    _data = new WM2HMData();
    _outd = new WM2HMData();

    try {
      // CONNECT
      InputStream in=socket.getInputStream();
      fireConnected();
      // READ FOREVER
      for(;;) {
	// read in all WM2 data
	for(int i=0; i<WorldModel2Conf.HM_CELL_COUNT; ++i) {
	  _data.HM_height[i] = readFloat(in);
	  _data.HM_trav[i] = readFloat(in);
	  _data.HM_confidence[i] = readFloat(in);
	  _data.HM_color[i] = readInt(in);
	}

	// this must be some lock thing. I really don't know why it works
	// this way.
	synchronized(_outd) {
	  WM2HMData temp = _data;
	  _data = _outd;	// why are we saving the old data?
	  _outd = temp;
	  _updatedFlag = true;
	}
      }
    } catch(Exception e) {
    } finally {
      fireDisconnected();
    }

    // DISCONNECT
    try { socket.close(); } catch(Exception e) {}
    _isConnected = false;
  }


  // Some state inquiry functions
  public boolean hasData() { return _updatedFlag; }
  public boolean isConnected() { return _isConnected; }


  // Data extraction
  public WM2HMData getData() {
    synchronized(_outd) {
      _updatedFlag = false;
      return _outd;
    }
  }

  // Constructors
  public WM2HMListener() { super(); needConnection(); }
  public WM2HMListener(int port) { super(port); needConnection(); }
  public WM2HMListener(String host, int port) { super(host, port); needConnection(); }
}
