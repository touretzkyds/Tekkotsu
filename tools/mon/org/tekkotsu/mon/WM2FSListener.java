package org.tekkotsu.mon;

// A Listener for the WorldModel2 FastSLAM data
import java.io.InputStream;
import java.net.Socket;

// Much of this code has been copied from WorldStateJointsListener.java.
// I'm not too certain how it works, but I think a fair part of it has to do
// with synchronization.
public class WM2FSListener extends TCPListener {
  boolean _updatedFlag=false;
  WM2FSData _data;
  WM2FSData _outd;

  // Evidently this is the meat of the class
  public void connected(Socket socket) {
    _isConnected = true;
    _data = new WM2FSData();
    _outd = new WM2FSData();

    try {
      // CONNECT
      InputStream in=socket.getInputStream();
      fireConnected();
      // READ FOREVER
      for(;;) {
	// read in all particle positions
	for(int i=0; i<WorldModel2Conf.AFS_NUM_PARTICLES; ++i) {
	  _data.FS_particlesPos[i][0] = readFloat(in);
	  _data.FS_particlesPos[i][1] = readFloat(in);
	  _data.FS_particlesPos[i][2] = readFloat(in);
	}

	// read in robot coordinates
	_data.FS_x = readFloat(in);
	_data.FS_y = readFloat(in);
	_data.FS_theta = readFloat(in);

	// read in landmark locations and covariances
	for(int i=0; i<WorldModel2Conf.AFS_NUM_LANDMARKS; ++i) {
	  _data.FS_landmarkPos[i][0] = readFloat(in);
	  _data.FS_landmarkPos[i][1] = readFloat(in);

	  _data.FS_covariance[i][0] = readFloat(in);
	  _data.FS_covariance[i][1] = _data.FS_covariance[i][2] = readFloat(in);
	  _data.FS_covariance[i][3] = readFloat(in);
	}

	// this must be some lock thing. I really don't know why it works
	// this way.
	synchronized(_outd) {
	  WM2FSData temp = _data;
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
  public WM2FSData getData() {
    synchronized(_outd) {
      _updatedFlag = false;
      return _outd;
    }
  }

  // Constructors
  public WM2FSListener() { super(); needConnection(); }
  public WM2FSListener(int port) { super(port); needConnection(); }
  public WM2FSListener(String host, int port) { super(host, port); needConnection(); }
}
