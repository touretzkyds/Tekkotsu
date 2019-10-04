package org.tekkotsu.mon;

// Sends "Mecha" command data from TekkotsuMon to the AIBO.
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
public class MechaController extends TCPListener implements ActionListener {
  // The command output stream
  OutputStream out;
  Socket mysock;
	Timer t=new Timer(1000,this);
	double forward=0;
	double strafe=0;
	double rotate=0;
	Vector listeners=new Vector();
	
	public interface MechaUpdatedListener {
		public void mechaUpdated(MechaController mc);
	}

	void addMechaUpdatedListener(MechaUpdatedListener mcl) { listeners.add(mcl); needConnection(); }
	void removeMechaUpdatedListener(MechaUpdatedListener mcl) { listeners.remove(mcl); }
	void fireMechaUpdated() {
		for(int i=0;i<listeners.size();i++)
			((MechaUpdatedListener)listeners.get(i)).mechaUpdated(this);
	}

  // Connect to control socket
  public void connected(Socket socket) {
    mysock = socket;
		t.start();
		fireMechaUpdated();
    try {
      out = mysock.getOutputStream();
			InputStream sin=socket.getInputStream();
			fireConnected();
			while (true) { //not that we expect input, but will block until the socket is closed
				String msgtype=readLine(sin);
				if(!_isConnected) break;
			}
    } catch (SocketException e) {
    } catch (Exception e) {
      e.printStackTrace();
    } finally {
      fireDisconnected();
    }

		try { socket.close(); } catch (Exception ex) { }

		_isConnected=false;
		fireMechaUpdated();
		//The sleep is to get around the socket still listening after being closed thing
		if(!destroy)
			System.out.println("WalkGUI - connection closed... reconnect after 5 seconds");
		try { Thread.sleep(5000); } catch (Exception ex) {}
  }

  // Disconnect from control socket
  public void close() {
		//    try { mysock.close(); } catch(Exception e) {}
    //_isConnected = false;
		t.stop();
    super.close();
		//we'll fire an event to the listeners when the readLine in connected finally fails
  }

	public void actionPerformed(ActionEvent e) {
		if(_isConnected) {
			sendCommand("f",forward);
			sendCommand("s",strafe);
			sendCommand("r",rotate);
		}
	}

  // Send a mecha command
  public void sendCommand(String command, double param) {
		if (out == null) {
			return;
		}
		t.restart();
		
    // Extract command byte
    byte cmdbytes[] = command.getBytes();
		if(cmdbytes[0]=='f')
			forward=param;
		else if(cmdbytes[0]=='s')
			strafe=param;
		else if(cmdbytes[0]=='r')
			rotate=param;

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
  public MechaController() { super(); }
  public MechaController(int port) { super(port); }
  public MechaController(String host, int port) { super(host, port); }
}
