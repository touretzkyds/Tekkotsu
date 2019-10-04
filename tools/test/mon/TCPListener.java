import java.net.ServerSocket;
import java.net.Socket;

public abstract class TCPListener extends Listener {
	public abstract void connected(Socket socket);

	public void runServer() {
		Thread me = Thread.currentThread();
		try { _serverSocket=new ServerSocket(_port); }
		catch (Exception ex) {
			System.out.println("port "+_port+": "+ex);
			return;
		}

		while (me == _listenerThread && !destroy) {
			try {
				_socket=_serverSocket.accept();
				connected(_socket);
			} catch (Exception ex) { }
		}
	}

	public void runConnect() {
		int attempts=0;
		Thread me = Thread.currentThread();
		while (me==_listenerThread && !destroy) {
			if(attempts==0) {
				System.out.println("["+_port+"] connecting ...");
			}
			try {
				_socket=new Socket(_host,_port);
				System.out.println("["+_port+"] connected");
				attempts=0;
				_isConnected=true;
			} catch (Exception ex) {}
			if(_isConnected) {
				connected(_socket);
				if(!destroy)
					System.out.println("["+_port+"] disconnected");
          System.out.println("["+_port+"] attempting to reestablish ..");
			}
			attempts++;
			if(destroy) {
				System.out.println("["+_port+"] connection closed");
				break;
			}
			try {
				Thread.sleep(500);
			} catch (Exception ex) {}
		}
	}

	public void close() {
		_listenerThread=null;
		_isConnected=false;
		try { _socket.close(); } catch (Exception ex) { }
		if (_isServer)
			try { _serverSocket.close(); } catch (Exception ex) { }
	}

	public TCPListener() { super(); }
	public TCPListener(int port) { super(port); }
	public TCPListener(String host, int port) { super(host,port); }

	Socket _socket;
	ServerSocket _serverSocket;
}
