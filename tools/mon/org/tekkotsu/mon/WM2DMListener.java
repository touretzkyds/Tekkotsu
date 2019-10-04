package org.tekkotsu.mon;

// A Listener for the WorldModel2 depth map data
import java.io.InputStream;
import java.net.Socket;

// Much of this code has been copied from WorldStateJointsListener.java.
// I'm not too certain how it works, but I think a fair part of it has to do
// with synchronization.
public class WM2DMListener extends TCPListener {
  boolean _updatedFlag=false;
  WM2DMData _data;
  WM2DMData _outd;

  // Evidently this is the meat of the class
  public void connected(Socket socket) {
    _isConnected = true;
    _data = new WM2DMData();
    _outd = new WM2DMData();

    try {
      // CONNECT
      InputStream in=socket.getInputStream();
      fireConnected();
      // READ FOREVER
      for(;;) {
	// read in all WM2 data
	for(int i=0; i<WorldModel2Conf.DM_CELL_COUNT; ++i) {
	  _data.DM_depth[i] = readFloat(in);
	  _data.DM_confidence[i] = readFloat(in);
	  _data.DM_color[i] = readInt(in);
	}

	// this must be some lock thing. I really don't know why it works
	// this way.
	synchronized(_outd) {
	  WM2DMData temp = _data;
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
  public WM2DMData getData() {
    synchronized(_outd) {
      _updatedFlag = false;
      return _outd;
    }
  }

  // Constructors
  public WM2DMListener() { super(); needConnection(); }
  public WM2DMListener(int port) { super(port); needConnection(); }
  public WM2DMListener(String host, int port) { super(host, port); needConnection(); }
}
