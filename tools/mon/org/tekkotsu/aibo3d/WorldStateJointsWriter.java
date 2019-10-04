package org.tekkotsu.aibo3d;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import org.tekkotsu.mon.TCPListener;
import org.tekkotsu.mon.Joints;
import java.net.SocketException;

public class WorldStateJointsWriter extends TCPListener {
  OutputStream _out;

  public void connected(Socket socket) {
    try {
      _out=socket.getOutputStream();
			
			//never actually receives anything...
			InputStream sin=socket.getInputStream();
			while (true) {
				String msgtype=readLine(sin);
			}
    } catch(Exception e) {if((SocketException)e==null) e.printStackTrace();}

		try { socket.close(); } catch (Exception ex) { }

		_isConnected=false;
  }

  public boolean hasData() {
    return false;
  }

  public Joints getData() {
    return null;
  }

  public void close() {
    _isConnected=false;
    super.close();
  }

  public void write(float[] f) {
    try {
      for (int i=0; i<f.length; i++)
        writeFloat(_out, f[i]);
    } catch (Exception ex) { close(); }
  }

  public boolean isConnected() {
    return _isConnected;
  }

  public WorldStateJointsWriter() { super(); }
  public WorldStateJointsWriter(int port) { super(port); }
  public WorldStateJointsWriter(String host, int port) { super(host,port); }
}
