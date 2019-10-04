package org.tekkotsu.mon;

import java.lang.Integer;
import java.lang.String;
import java.io.PrintStream;
import java.io.InputStream;
import java.net.Socket;
import java.util.Vector;
import java.net.SocketException;

public class EStopListener extends TCPListener {
	// The command output stream
	static int defPort=10053;
	PrintStream out;
	boolean stopped=true;
	boolean data=false;
	Socket mysock;
	Vector listeners=new Vector();
	
	public interface UpdatedListener {
		public void estopUpdated(EStopListener mc);
	}

	public void addUpdatedListener(UpdatedListener mcl) { listeners.add(mcl); needConnection(); }
	public	void removeUpdatedListener(UpdatedListener mcl) {
		listeners.remove(mcl);
		if(listeners.size()==0)
			kill();
	}
	void fireUpdated() {
		data=true;
		if(listeners==null) {
			System.out.println("wtf?");
			listeners=new Vector();
		}
		for(int i=0;i<listeners.size();i++)
			((UpdatedListener)listeners.get(i)).estopUpdated(this);
	}
	
  // Connect to control socket
  public void connected(Socket socket) {
    mysock = socket;
    try {
      out = new PrintStream(mysock.getOutputStream());
			fireUpdated();
			out.print("refresh\n");
			InputStream sin=socket.getInputStream();
			fireConnected();
			while (true) {
				String msgtype=readLine(sin);
				if(!_isConnected) break;
				if(msgtype.equals("on")) {
					if(!stopped) {
						stopped=true;
						fireUpdated();
					}
				} else if(msgtype.equals("off")) {
					if(stopped) {
						stopped=false;
						fireUpdated();
					}
				} else {
					System.out.println("Estop Unknown message: "+msgtype);
				}
			}
    } catch (SocketException e) {
    } catch (Exception e) {
      e.printStackTrace();
    } finally {
      fireDisconnected();
    }

		try { socket.close(); } catch (Exception ex) { }

		_isConnected=false;
		fireUpdated();
		//The sleep is to get around the socket still listening after being closed thing
		//if(!destroy)
		//System.out.println("EStopGUI - connection closed... reconnect after 5 seconds");
		//try { Thread.sleep(5000); } catch (Exception ex) {}
  }

  // Disconnect from control socket
  public void close() {
		//    try { mysock.close(); } catch(Exception e) {}
    //_isConnected = false;
    super.close();
		//we'll fire an event to the listeners when the readLine in connected finally fails
  }

	public void setEStop(boolean b) {
		if(!isConnected())
			return;
		if(b!=stopped) {
			out.print(b?"stop\n":"start\n");
		}
	}

	public boolean getEStop() {
		return stopped;
	}

	public void toggleEStop() {
		setEStop(!stopped);
	}

  // Some state inquiry functions
  public boolean hasData() { return data; }
  public boolean isConnected() { return _isConnected; }

  // Constructors
  public EStopListener() { super(); }
  public EStopListener(int port) { super(port); }
  public EStopListener(String host, int port) { super(host, port); }
}
