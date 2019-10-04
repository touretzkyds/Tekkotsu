package org.tekkotsu.mon;

// Sends "HeadPoint" command data from TekkotsuMon to the AIBO.
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
public class HeadPointListener extends TCPListener implements ActionListener {
  // The command output stream
  OutputStream out;
  Socket mysock;
	double tilt=0;
	double pan=0;
	double roll=0;
	Vector listeners=new Vector();
	
	public interface HeadPointUpdatedListener {
		public void headPointUpdated(HeadPointListener mc);
	}

	void addHeadPointUpdatedListener(HeadPointUpdatedListener mcl) { listeners.add(mcl); needConnection(); }
	void removeHeadPointUpdatedListener(HeadPointUpdatedListener mcl) { listeners.remove(mcl); }
	void fireHeadPointUpdated() {
		for(int i=0;i<listeners.size();i++)
			((HeadPointUpdatedListener)listeners.get(i)).headPointUpdated(this);
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
			fireHeadPointUpdated();
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
		fireHeadPointUpdated();
		//The sleep is to get around the socket still listening after being closed thing
		if(!destroy)
			System.out.println("HeadPoint - connection closed... reconnect after 5 seconds");
		try { Thread.sleep(5000); } catch (Exception ex) {}
  }

  // Disconnect from control socket
  public void close() {
		//    try { mysock.close(); } catch(Exception e) {}
    //_isConnected = false;
    super.close();
		//we'll fire an event to the listeners when the readLine in connected finally fails
  }

	public void actionPerformed(ActionEvent e) {
		if(_isConnected) {
			sendCommand("t",tilt);
			sendCommand("p",pan);
			sendCommand("r",roll);
		}
	}

  // Send a headPoint command
  public void sendCommand(String command, double param) {
		
    if (out == null) {
      return;
    }
    // Extract command byte
    byte cmdbytes[] = command.getBytes();
		if(cmdbytes[0]=='t')
			tilt=param;
		else if(cmdbytes[0]=='p')
			pan=param;
		else if(cmdbytes[0]=='r')
			roll=param;

    // Construct the command sequence
    byte sequence[] = new byte[5];
    // The commmand byte is the first byte in cmdbytes. The remaining
    // four bytes belong to the parameter. We have to convert the parameter
    // (which we send as a float, not a double) to MIPS byte order thanks to
    // (ahem) prior design decisions.
    sequence[0] = cmdbytes[0];
    int pbits = Float.floatToIntBits((float) param);
    Integer i;
    i = new Integer((pbits >> 24) & 0xff); sequence[4] = i.byteValue();
    i = new Integer((pbits >> 16) & 0xff); sequence[3] = i.byteValue();
    i = new Integer((pbits >>  8) & 0xff); sequence[2] = i.byteValue();
    i = new Integer(pbits & 0xff);	   sequence[1] = i.byteValue();
    // Now write the whole command.
    try {
      out.write(sequence, 0, 5);
    } catch(Exception e) { close(); return; }
  }

  // Some state inquiry functions
  public boolean hasData() { return false; }
  public boolean isConnected() { return _isConnected; }

  // Constructors
  public HeadPointListener() { super(); }
  public HeadPointListener(int port) { super(port); }
  public HeadPointListener(String host, int port) { super(host, port); }
}
