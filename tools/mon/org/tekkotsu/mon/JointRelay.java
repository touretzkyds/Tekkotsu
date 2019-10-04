package org.tekkotsu.mon;

import java.io.InputStream;
import java.net.Socket;

public class JointRelay extends TCPListener {
  boolean _updatedFlag=false;
  Joints _data;
  Joints _outd;
    JointRequestor requestor;

  public void connected(Socket socket) {
    _isConnected=true;
    _data=new Joints();
    _outd=new Joints();
    try {
      InputStream in=socket.getInputStream();
      fireConnected();
      while (true) {
        _data.timestamp=readInt(in);
	int count;
	count=readInt(in);
	_data.positions=new float[count];
        for (int i=0; i<count; i++)
          _data.positions[i]=readFloat(in);
	count=readInt(in);
	_data.sensors=new float[count];
        for (int i=0; i<count; i++)
          _data.sensors[i]=readFloat(in);
	count=readInt(in);
	_data.buttons=new float[count];
        for (int i=0; i<count; i++)
          _data.buttons[i]=readFloat(in);
	count=readInt(in);
	_data.duties=new float[count];
        for (int i=0; i<count; i++)
          _data.duties[i]=readFloat(in);

        synchronized(_outd) {
          Joints temp=_data;
          _data=_outd;
          _outd=temp;
          _updatedFlag=true;
	  if(requestor!=null){
	      requestor.dataArrival(_outd);
	  }
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

  public Joints getData() {
    synchronized (_outd) {
      _updatedFlag=false;
      return _outd;
    }
  }

  public boolean isConnected() {
    return _isConnected;
  }

  public JointRelay() { super(); }
  public JointRelay(int port) { super(port); }
  public JointRelay(String host, int port) { super(host,port); }
    public JointRelay(String host, int port,JointRequestor ob) { super(host,port);
    registerRequestor(ob);
    }
    public void registerRequestor(JointRequestor ob){
	requestor=ob;
    }
}
