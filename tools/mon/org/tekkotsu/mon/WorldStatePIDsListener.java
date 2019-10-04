package org.tekkotsu.mon;

import java.io.InputStream;
import java.net.Socket;

public class WorldStatePIDsListener extends TCPListener {
  boolean _updatedFlag=false;
  PIDs _data;
  PIDs _outd;

  public void connected(Socket socket) {
    _isConnected=true;
    _data=new PIDs();
    _outd=new PIDs();
    try {
      InputStream in=socket.getInputStream();
      fireConnected();
      while (true) {
        _data.timestamp=readInt(in);
        for (int i=0; i<18; i++)
          _data.P[i]=readFloat(in);
        for (int i=0; i<18; i++)
          _data.I[i]=readFloat(in);
        for (int i=0; i<18; i++)
          _data.D[i]=readFloat(in);

        synchronized(_outd) {
          PIDs temp=_data;
          _data=_outd;
          _outd=temp;
          _updatedFlag=true;
        }
      }
    } catch (Exception ex) {
    } finally {
      fireDisconnected();
    }

    try { socket.close(); } catch (Exception ex) { }
    _isConnected=false;
  }
 
  public boolean hasData() {
    return _updatedFlag;
  }

  public PIDs getData() {
    synchronized (_outd) {
      _updatedFlag=false;
      return _outd;
    }
  }

  public boolean isConnected() {
    return _isConnected;
  }

  public WorldStatePIDsListener() { super(); needConnection(); }
  public WorldStatePIDsListener(int port) { super(port); needConnection(); }
  public WorldStatePIDsListener(String host, int port) { super(host,port); needConnection(); }
}
