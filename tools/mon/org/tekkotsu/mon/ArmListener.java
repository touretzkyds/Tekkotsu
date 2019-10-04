package org.tekkotsu.mon;

// Sends "Arm" command data from TekkotsuMon to the robot.
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
import javax.swing.JSlider;

// The class itself. Brilliant that even though it does the talking,
// it extends TCP*Listener*.
public class ArmListener extends TCPListener implements ActionListener {
    // The command output stream
    OutputStream out;
    InputStream sin;
    Socket mysock;	
    Vector listeners=new Vector();
	
    public interface ArmUpdatedListener {
	public void ArmUpdated(ArmListener mc);
	public void ArmUpdated(String mc);
    }
    void addArmUpdatedListener(ArmUpdatedListener mcl) { listeners.add(mcl); needConnection();}
    void removeArmUpdatedListener(ArmUpdatedListener mcl) { listeners.remove(mcl); }
	
    void fireArmUpdated() {		
	for(int i=0;i<listeners.size();i++)
	    ((ArmUpdatedListener)listeners.get(i)).ArmUpdated(this);
    }
	
    void fireArmUpdated(String msg) {
	for(int i=0;i<listeners.size();i++){
	    ((ArmUpdatedListener)listeners.get(i)).ArmUpdated(msg);
	}
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
	    sin = socket.getInputStream();
	    fireArmUpdated();
	    fireArmUpdated("Connected");
	    while (true) { 
		String msgrec=readLine(sin);
		fireArmUpdated(msgrec);
		if(!_isConnected) {					
		    break;
		}
	    }
	} catch(SocketException e) {} 
	catch(Exception e) {
	    e.printStackTrace();
	} finally {
	    fireDisconnected();
	}
	try { socket.close(); } catch (Exception ex) {}
	_isConnected=false;
	fireArmUpdated();
	//The sleep is to get around the socket still listening after being closed thing
	if(!destroy)
	    System.out.println("Arm - connection closed... reconnect after 5 seconds");
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
	    //sendCommand("s",shoulder);
	    //sendCommand("e",elbow);
	    //sendCommand("w",wrist);
	    //System.out.println("YOYO");
	}
    }
	
	

    // Send a Arm command
    public void sendCommand(String command, int cmdno, double param) {	
	if (out == null) {
	    return;
	}

	// Extract command byte
	byte cmdbytes[] = command.getBytes();

	// Construct the command sequence
	byte sequence[] = new byte[9];
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
	i = new Integer((pbits >>  0) & 0xff); sequence[1] = i.byteValue();
		
	i = new Integer((cmdno >> 24) & 0xff); sequence[8] = i.byteValue();
	i = new Integer((cmdno >> 16) & 0xff); sequence[7] = i.byteValue();
	i = new Integer((cmdno >>  8) & 0xff); sequence[6] = i.byteValue();
	i = new Integer((cmdno >>  0) & 0xff); sequence[5] = i.byteValue();
		
	// Now write the whole command.
	try {
	    out.write(sequence, 0, 9);
	} catch(Exception e) { close(); return; }
    }

    // Send a Arm command
    public void sendCommand(String command, int cmdno, double param1, double param2, double param3) {
		
	if (out == null)
	    return;

	// Extract command byte
	byte cmdbytes[] = command.getBytes();

	// Construct the command sequence
	byte sequence[] = new byte[19];
	// The commmand byte is the first byte in cmdbytes. The remaining
	// four bytes belong to the parameter. We have to convert the parameter
	// (which we send as a float, not a double) to MIPS byte order thanks to
	// (ahem) prior design decisions.
	sequence[0] = cmdbytes[0];
	sequence[5] = cmdbytes[0];
	sequence[10] = cmdbytes[0];
    
	int pbits = Float.floatToIntBits((float) param1);
	Integer i;
	i = new Integer((pbits >> 24) & 0xff); sequence[4] = i.byteValue();
	i = new Integer((pbits >> 16) & 0xff); sequence[3] = i.byteValue();
	i = new Integer((pbits >>  8) & 0xff); sequence[2] = i.byteValue();
	i = new Integer((pbits >>  0) & 0xff); sequence[1] = i.byteValue();
    
	pbits = Float.floatToIntBits((float) param2);
	i = new Integer((pbits >> 24) & 0xff); sequence[9] = i.byteValue();
	i = new Integer((pbits >> 16) & 0xff); sequence[8] = i.byteValue();
	i = new Integer((pbits >>  8) & 0xff); sequence[7] = i.byteValue();
	i = new Integer((pbits >>  0) & 0xff); sequence[6] = i.byteValue();
		
	pbits = Float.floatToIntBits((float) param3);
	i = new Integer((pbits >> 24) & 0xff); sequence[14] = i.byteValue();
	i = new Integer((pbits >> 16) & 0xff); sequence[13] = i.byteValue();
	i = new Integer((pbits >>  8) & 0xff); sequence[12] = i.byteValue();
	i = new Integer((pbits >>  0) & 0xff); sequence[11] = i.byteValue();
		
	i = new Integer((cmdno >> 24) & 0xff); sequence[18] = i.byteValue();
	i = new Integer((cmdno >> 16) & 0xff); sequence[17] = i.byteValue();
	i = new Integer((cmdno >>  8) & 0xff); sequence[16] = i.byteValue();
	i = new Integer((cmdno >>  0) & 0xff); sequence[15] = i.byteValue();
	// Now write the whole command.
	try {
	    out.write(sequence, 0, 19);
	} catch(Exception e) { close(); return; }
    }
	
    public void ArmStatusUpdate() {
	fireArmUpdated();
	fireConnected();
    }
    // Some state inquiry functions
    public boolean hasData() { return false; }
    public boolean isConnected() { return _isConnected; }

    // Constructors
    public ArmListener() { super(); }
    public ArmListener(int port) { super(port); }
    public ArmListener(String host, int port) { super(host, port); }
}
