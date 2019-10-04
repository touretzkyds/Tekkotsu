package org.tekkotsu.mon;

// Sends "Rover" command data from TekkotsuMon to the AIBO.
import java.lang.Integer;
import java.lang.String;
import java.lang.System;
import java.io.OutputStream;
import java.io.InputStream;
import java.net.Socket;
import javax.swing.Timer;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Vector;
import java.net.SocketException;

// The class itself. Brilliant that even though it does the talking,
// it extends TCP*Listener*.
public class RoverListener extends TCPListener {
  // The command output stream
  OutputStream out;
  Socket mysock;
	Vector listeners=new Vector();
	
	public interface RoverUpdatedListener {
		public void RoverUpdated(RoverListener mc);
	}

	void addRoverUpdatedListener(RoverUpdatedListener mcl) { listeners.add(mcl); needConnection(); }
	void removeRoverUpdatedListener(RoverUpdatedListener mcl) { listeners.remove(mcl); }
	void fireRoverUpdated() {
		for(int i=0;i<listeners.size();i++)
			((RoverUpdatedListener)listeners.get(i)).RoverUpdated(this);
	}

  // Connect to control socket
  public void connected(Socket socket) {
    mysock = socket;
    try {
			mysock.setTcpNoDelay(true);
		 try {
			mysock.setTrafficClass(0x10);
		 } catch(SocketException e) {}
			out = mysock.getOutputStream();
			InputStream sin=socket.getInputStream();
			fireRoverUpdated();
			fireConnected();
			while (true) { //not that we expect input, but will block until the socket is closed
				String msgtype=readLine(sin);
				if(!_isConnected) break;
			}
    } catch(SocketException e) {
    } catch(Exception e) {
      e.printStackTrace();
    } finally {
      fireDisconnected();
    }

		try { socket.close(); } catch (Exception ex) { }

		_isConnected=false;
		fireRoverUpdated();
		//The sleep is to get around the socket still listening after being closed thing
		if(!destroy)
			System.out.println("Rover - connection closed... reconnect after 5 seconds");
		try { Thread.sleep(5000); } catch (Exception ex) {}
  }

  // Disconnect from control socket
  public void close() {
		//    try { mysock.close(); } catch(Exception e) {}
    //_isConnected = false;
    super.close();
		//we'll fire an event to the listeners when the readLine in connected finally fails
  }

  // Send a Rover command
  public void sendCommand(String command) {
    if (out == null)
      return;
    command=command+"\n";
    try {
      out.write(command.getBytes());
    } catch(Exception e) { close(); return; }
  }

  // Some state inquiry functions
  public boolean hasData() { return false; }
  public boolean isConnected() { return _isConnected; }

  // Constructors
  public RoverListener() { super(); }
  public RoverListener(int port) { super(port); }
  public RoverListener(String host, int port) { super(host, port); }
}
