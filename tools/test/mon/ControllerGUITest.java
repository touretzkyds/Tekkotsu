import java.io.*;
import java.net.Socket;
import java.util.Vector;

public class ControllerGUITest extends TCPListener {
	boolean _updatedFlag=false;
	Vector _menus;
	PrintStream _out;

	void connected(Socket socket) {
		_isConnected=true;
		System.out.println("Connected.");
		try {
			_out=new PrintStream(socket.getOutputStream());
			InputStream sin=socket.getInputStream();
			while(true) {
				System.out.println("Sending...");
				_out.println("refresh");
				_out.println("Test menu");
				Vector menu=(Vector)_menus.lastElement();
				_out.println(menu.size());
				for(int i=0; i<menu.size(); i++) {
					_out.println(i%2);
					_out.println(menu.get(i));
					_out.println(menu.get(i)+" doesn't do much");
				}
				System.out.println("Sent.");
				synchronized(_menus) {
					_updatedFlag=true;
				}
				String response=readLine(sin);
				System.out.println("response: "+response);
				if(!_isConnected) break;
			}
		} catch (Exception ex) { }

		System.out.println("Disconnecting");

		try { socket.close(); } catch (Exception ex) { }
		_isConnected=false;

		System.out.println("Disconnected");
	}
 
	public boolean hasData() {
		return _updatedFlag;
	}

	public boolean isConnected() {
		return _isConnected;
	}

	public static void main(String s[]) {
		ControllerGUITest t=new ControllerGUITest();
		Vector menu=new Vector();
		menu.add("Hello");
		menu.add("Out");
		menu.add("There in");
		menu.add("TV Land!");
		t._menus=new Vector();
		t._menus.add(menu);
		t.setPort(10020);
	}

	public ControllerGUITest() { super(); }
	public ControllerGUITest(int port) { super(port); }
	public ControllerGUITest(String host, int port) { super(host,port); }
}
