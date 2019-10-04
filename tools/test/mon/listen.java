import java.net.*;
import java.io.*;

public class listen extends TCPListener {
	public static void main(String[] args) {
		if(args.length<1) {
			usage(args);
			System.exit(2);
		}
		new listen((new Integer(args[0])).intValue());
	}
	
	public static void usage(String[] args) {
		System.out.println("Usage: java listen port");
		System.out.println("       This will listen on <port> until a connection occurs.");
		System.out.println("       Any output is sent to the console, and any console");
		System.out.println("       input will be sent to the remote host, terminated by");
		System.out.println("       newline");
	}
	
	public listen() { super(); }
	public listen(int port) { super(port); }
	
	public class send implements Runnable {
		public Socket socket;
		public void run() {
			try {
				PrintStream out = new PrintStream(socket.getOutputStream());
				BufferedReader in = new BufferedReader( new InputStreamReader( System.in ) );
 				while (!socket.isOutputShutdown()) {
					out.println(in.readLine());
				}
			} catch(Exception e) {if((SocketException)e==null) e.printStackTrace();}
	
			try { socket.close(); } catch (Exception ex) { }
			System.out.println("[Lost output...]");
		}
	}
	public class receive implements Runnable {
		public Socket socket;
		public void run() {
			try {
				InputStream sin=socket.getInputStream();
				while (!socket.isInputShutdown()) {
					System.out.print((char)sin.read());
				}
			} catch(Exception e) {if((SocketException)e==null) e.printStackTrace();}
	
			try { socket.close(); } catch (Exception ex) { }
			System.out.println("[Lost input...]");
		}
	}

	public void runServer() {
		System.out.println("[Listening...]");
		super.runServer();
	}
	
	public void connected(Socket socket) {
		System.out.println("[Connected...]");
		receive recv=new receive();
		send snd=new send();
		recv.socket=snd.socket=socket;
		(new Thread(recv)).start();
		(new Thread(snd)).start();
	}

}