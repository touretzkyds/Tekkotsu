package org.tekkotsu.mon;

import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.io.PrintStream;
import java.net.Socket;

public class TextListener extends TCPListener {
  String _data="";
  PrintStream _out;

  public void connected(Socket socket) {
    _isConnected=true;
    try {
      BufferedReader in=new BufferedReader(new InputStreamReader(
                          socket.getInputStream()));
      _out=new PrintStream(socket.getOutputStream());
      fireConnected();
      while (true) {
        String read=in.readLine();
        if (read==null) break;
        synchronized (_data) { _data=_data+read+"\n"; }
      }
    } catch (Exception ex) {
    } finally {
      fireDisconnected();
    }

    try { socket.close(); } catch (Exception ex) { }
    _isConnected=false;
  }
 
  public boolean hasData() {
    return _data.length()!=0;
  }

  public String getData() {
    String ret;
    synchronized (_data) { 
      ret=_data;
      _data="";
    }
    return ret;
  }

  public void write(String s) {
    if (_isConnected) {
      _out.print(s);
      _out.flush();
    }
  }

  public boolean isConnected() {
    return _isConnected;
  }

  public TextListener() { super(); }
  public TextListener(int port) { super(port); }
  public TextListener(String host, int port) { super(host,port); }
}
