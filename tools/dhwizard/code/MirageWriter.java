import java.io.*;
import java.net.*;
import java.util.Map;
import java.util.Set;

public class MirageWriter {
	protected Socket mirageSocket;
	protected PrintWriter out;
	protected String msgName;
	protected String server;

	public MirageWriter(String name, String server) throws UnknownHostException, IOException {
		this.server = server;
		msgName = name;
		mirageSocket = new Socket(server, 19785);
		out = new PrintWriter(mirageSocket.getOutputStream(), true);
	}

	//Transmits the robot's structure to Mirage
	public void sendOpeningMessage(JointNode root) {
		String msg = "";
		msg += "<messages>\n";
		msg += "<plist version=\"1.0\"><dict>\n";
		msg += "<key>ID</key> <string>"+msgName+"</string>\n";
		msg += "<key>Location</key> <array>\n<real>0</real>\n<real>0</real>\n<real>0</real>\n</array>\n";
		msg += "<key>Model</key> <array>\n";
		msg += root.toXMLString();
		msg += "</array>";
		msg += "</dict></plist>\n";
		out.println(msg);
	}

	//Updates the robot's position by transmitting the joint angles
	public void sendUpdateMessage(String jointName, int angle) {
		String msg = "";
		msg += "<plist version=\"1.0\"><dict>\n";
		msg += "<key>ID</key> <string>"+msgName+"</string>\n";
		msg += "<key>Positions</key> <dict>\n";
		msg += "<key>"+jointName+"</key> <real>"+(angle/180.*Math.PI)+"</real>\n";
		msg += "</dict>\n</dict></plist>";
		out.println(msg);
	}

	public void sendAllAngles(Map<String,Integer> angles) {
		Set<String> keys = angles.keySet();
		for(String key : keys) {
			sendUpdateMessage(key, angles.get(key));
		}
	}

	//Closes the connection with Mirage
	private void die() {
		out.close();
		try {
			mirageSocket.close();
		} catch (IOException e) {
			System.out.println("Couldn't close the socket");
			System.exit(1);
		}
	}
		
	public void sendCloseMessage() {
		out.println("</messages>");
		die();
	}

	protected String getValuesString(JointNode j, String jointName) {
		
		String s = "";
		if (j.name == null) return s;
		if (j.name.equals(jointName)){
		
		s = "<key>"+j.name+"</key> <real>";
		if (j.type.equals("revolute")) {
			s += j.theta;
		} else {
			s += j.d;
		}
		s += "</real>\n";
		}
		for (int i = 0; i < j.children.size(); i++) {
			s += getValuesString((JointNode)(j.children.get(i)), jointName);
		}
		return s;
	}

}
