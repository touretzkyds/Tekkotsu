package org.tekkotsu.mon;

import java.net.ServerSocket;
import java.net.Socket;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;
import net.java.games.input.ControllerListener;
import net.java.games.input.ControllerEvent;
import net.java.games.input.LinuxEnvironmentPlugin;
import net.java.games.util.plugins.Plugin;
import java.net.Socket;
import java.io.OutputStream;
import java.io.IOException;
import java.net.SocketException;
import java.io.PrintStream;
import java.lang.*;

public class GamepadListener extends TCPListener implements ControllerListener {
//Add mapping of Componenets to ints
  static int defPort=10041;

  public GamepadListener() { super();}
  public GamepadListener(int port) {super(port); }
  public GamepadListener(String host, int port) { super(host,port); }

  private void init() {
    ce = ControllerEnvironment.getDefaultEnvironment();
    ce.addControllerListener(this);
    LinuxEnvironmentPlugin lep = new LinuxEnvironmentPlugin();
    Controller[] cs = lep.getControllers();
    for (Controller c : cs) {
      if (c.getType() == Controller.Type.GAMEPAD) {
	System.out.println("Gamepad controller found");
        gamepad = c;
        event_queue = gamepad.getEventQueue();
        break;
      }
    }
}

public void connected(Socket socket) {
  if ( noJavaGamesClass ) return;
  try {
    out = new PrintStream(socket.getOutputStream());
    fireConnected();
    if (round == 0) {
    	init();
    }
    while (true) {
      processEvents();
    }
  } catch (SocketException e) {
  } catch (NullPointerException e) {
  } catch (NoClassDefFoundError e) {
      noJavaGamesClass = true;
      System.out.println("net.java.games not on CLASSPATH: no game controller available");
  } catch (Exception e) {
    e.printStackTrace();
  } finally {
    fireDisconnected();
  }

  try { socket.close(); } catch (Exception ex) { }

  _isConnected=false;
  if (countdown > 0 ) {
    System.out.println("Reconnect controller in " + countdown + " seconds");
    countdown -= 1;
  }
  else if (countdown == 0 ) {
    System.out.println("Please reconnect controller now");
    countdown -= 1;
  }
  round = (round + 1) % 3;
  try{ Thread.sleep(1000); } catch (Exception e) {}
}

  public void controllerRemoved(ControllerEvent ev) {
    System.out.println("Detected remove");
    if (gamepad == ev.getController()) {
        gamepad = null;
    }
  }

  public void controllerAdded(ControllerEvent ev) {
    System.out.println("Detected add");
    if (gamepad == null) {
        gamepad = ev.getController();
    }
  }
  private void processEvents() {
    if ( ! gamepad.poll()) {
        gamepad = null;
        event_queue = null;
	countdown = 20;
	System.out.println("Connection to controller lost.\nPlease wait 20 seconds before plugging the controller back in");
	return;
    }
    Event event = new Event();
    while(event_queue.getNextEvent(event)) {
      Component comp = event.getComponent();
      //Get component id for first byte of packet
      float value = event.getValue();
           	sendCommand(comp.getName(), value);
           try {
        Thread.sleep(5);
      } catch (Exception ex) {}
    }

    //sendJoystick();


  }
  private void sendJoystick() {
    Component[] comps = gamepad.getComponents();
    for (Component comp : comps) {
       if (comp.getName() == "x" || comp.getName() == "y"
	 || comp.getName() == "rx" || comp.getName() == "ry") {
         sendCommand(comp.getName(), comp.getPollData());
         try {
         Thread.sleep(5);
         } catch (Exception ex) {}
       }
    }
  }
  private byte getCommandId(String component) {
    switch (component) {
      case "X":
        return (byte) 1;

      case "Y":
        return (byte) 2;

      case "A":
        return (byte) 3;

      case "B":
        return (byte) 4;

      case "Start_Button":
        return (byte) 5;

      case "Select_Button":
        return (byte) 6;

      case "x":
        return (byte) 7;

      case "y":
        return (byte) 8;

      case "z":
        return (byte) 0;

      case "rx":
        return (byte) 9;

      case "ry":
        return (byte) 10;

      case "rz": //TODO add triggers to alexes code
        return (byte) 0;

      case "Left Thumb":
        return (byte) 13;

      case "Right Thumb":
        return (byte) 14;

      case "pov": //TODO represented incorrectly for now
        return (byte) 11;

      default:
        return 12;
    }
  }
  
  private void sendCommand(String component, float value) {
    byte id = getCommandId(component);


    byte[] buf = new byte[5];
    if (id == 11) {
         if (Math.abs(value-0.25) < .000001) { //UP
            id = 12;
            value = 1;
            
         }

         else if (Math.abs(value-0.5) < .000001) { //RIGHT
            value = -1;
         }
         else if (Math.abs(value-0.75) < .000001) { //Dowin
            id = 12;
            value = -1;
         }
         else if (Math.abs(value-1.0) < .000001) { //LEFT
            value = 1;
         }
         else {
            value = 0;
            sendCommand("lpov", value);
         }
      }
     else {

      if (Math.abs(value) < .14) {
          value = 0;
      }
    }
    int bits = Float.floatToRawIntBits(value);
    
    buf[0] = id;
    buf[4] = (byte)(0xFF & bits);
    buf[3] = (byte)(((0xFF << 8) & bits) >> 8);
    buf[2] = (byte)(((0xFF << 16) & bits) >> 16);
    buf[1] = (byte)(((0xFF << 24) & bits) >> 24);
    try {
      writeBytes(out,buf);
    } catch (IOException e) {
      System.out.println("Failed to send gamepad instructions");
    }
  
  }

  private boolean noJavaGamesClass = false;
  private PrintStream out;
  private ControllerEnvironment ce;
  private Controller gamepad;
  private EventQueue event_queue;
  private int countdown = -2;
  private int round = 0;
}
