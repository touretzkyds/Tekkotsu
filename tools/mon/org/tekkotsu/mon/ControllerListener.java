package org.tekkotsu.mon;

import java.io.*;
import java.net.Socket;
import java.util.Vector;
import java.util.Iterator;
import java.lang.Integer;
import java.util.HashMap;
import java.lang.Class;
import java.lang.reflect.Method;
import java.lang.reflect.Constructor;
import java.net.SocketException;

public class ControllerListener extends TCPListener {
    boolean _updatedFlag=true;
    Vector _titles;
    Vector _menus=new Vector();
    String _status=new String();
    PrintStream _out;
    ControllerGUI gui;
    HashMap dynObjs=new HashMap();
    HashMap dynObjSrcs=new HashMap();
    HashMap dynObjPorts=new HashMap();
    InputStream sin;
    int _connectCount=0;
    static int defPort=10020;
	
    public class MenuEntry {
	boolean hasSubmenu;
	boolean selected;
	String title;
	String description;
	MenuEntry(boolean hasSubmenu, boolean selected, String title, String description) {
	    this.hasSubmenu=hasSubmenu;
	    this.selected=selected;
	    this.title=title;
	    this.description=description;
	}
	public String toString() { return title; }
    }
	
    static String escapize(Vector path) {
	System.out.println("Escapize:"+path);
	StringBuffer p=new StringBuffer();
	System.out.println("Escapized1:"+p);
	for(int i=0; i<path.size(); i++) {
	    p.append("/");
	    System.out.println("Escapized2:"+p);
	    p.append(((String)path.get(i)).replaceAll("\\\\","\\\\\\\\").replaceAll("/","\\\\/"));
	    System.out.println("Escapized3:"+p);
	}
	System.out.println("Escapized:"+p);
	return p.toString();
    }

    public int firstSelected() {
	if(_menus.size()==0)
	    return -1;
	Vector menu=(Vector)_menus.lastElement();
	for(int i=0; i<menu.size(); i++)
	    if(((MenuEntry)menu.get(i)).selected)
		return i;
	return -1;
    }

    public Vector buildSelectionPath(int index) {
	Vector ans=buildSelectionPath();
	ans.add(((Vector)_menus.lastElement()).get(index).toString());
	return ans;
    }
		
    public Vector buildSelectionPath() {
	Vector ans=new Vector();
	for(int i=0; i<_titles.size(); i++)
	    ans.add(_titles.get(i));
	//ans.add(((Vector)_menus.get(i)).get(((Integer)_selections.get(0)).intValue()).toString());
	return ans;
    }
		
    void sendSelectionPath(Vector path) {
	try {
	    _out.println("!select");
	} catch (Exception ex) { }
    }
	
    void sendSelect() {
	try {
	    _out.println("!select");
	} catch (Exception ex) { }
    }
	
    void sendReturn() {
	try {
	    _out.println("!cancel");
	} catch (Exception ex) { }
    }
	
    void sendRefresh() {
	try {
	    _out.println("!refresh");
	} catch (Exception ex) { }
    }
	
    void sendInput(String s,Vector path) {
	try {
	    Vector cur=buildSelectionPath();
	    sendSelectionPath(path);
	    _out.println(s);
	    sendSelectionPath(cur);
	} catch (Exception ex) { }
    }
	
    void sendInput(String s) {
	try {
	    _out.println(s);
	} catch (Exception ex) { }
    }
	
    void sendSelection(int sel[]) {
	try {
	    String msg="!hilight";
	    for(int i=0; i<sel.length; i++) {
		msg+=" ";
		msg+=sel[i];
	    }
	    _out.println(msg);
	} catch (Exception ex) { }
    }

    public void connected(Socket socket) {
	_isConnected=true;
	_connectCount=0;
	_menus=new Vector();
	_menus.add(new Vector());
	_titles=new Vector();
	_titles.add("Loading...");
	//		System.out.println("connection opened");
	try {
	    sin=socket.getInputStream();
	    _out=new PrintStream(socket.getOutputStream());
	    fireConnected();
	    if(gui!=null) {
		gui.gotConnection();
		//gui.repaint();
	    }
	    _out.println("!hello");
	    _out.println("!dump_stack");
	    _out.println("!refresh");
	    while (true) {
		String msgtype=readLine(sin);
		if(!_isConnected)
		    break;
		//				System.out.println("Received: "+msgtype);
		synchronized(_menus) {
		    if(msgtype.equals("push")) {
			_menus.add(new Vector());
			_titles.add("Loading...");
		    } else if(msgtype.equals("refresh")) {
			String last=readLine(sin);
			if(!_isConnected) break;
			_titles.set(_titles.size()-1,last);
			int len=Integer.parseInt(readLine(sin));
			if(!_isConnected) break;
			Vector menu=(Vector)_menus.lastElement();
			menu.clear();
			for(;len>0;len--) {
			    int x=Integer.parseInt(readLine(sin));
			    if(x==-1) {
				_isConnected=false;
				break;
			    }
			    int sel=Integer.parseInt(readLine(sin));
			    if(sel==-1) {
				_isConnected=false;
				break;
			    }
			    last=readLine(sin);
			    if(!_isConnected) break;
			    int descript_nlines = Integer.parseInt(readLine(sin));
			    if(descript_nlines==-1)
				_isConnected=false;
			    if(!_isConnected) break;
			    String descript=readLine(sin);
			    if(!_isConnected) break;
			    for(; descript_nlines>0; descript_nlines--) {
				descript+="\n"+readLine(sin);
				if(!_isConnected) break;
			    }
			    menu.add(new MenuEntry((x>0),(sel>0),last,descript));
			}
			_status="";
			if(!_isConnected) break;
		    } else if(msgtype.equals("pop")) {
			if(_menus.size()>1) {
			    _menus.remove(_menus.size()-1);
			    _titles.remove(_titles.size()-1);
			}
		    } else if(msgtype.equals("hello")) {
			_connectCount=Integer.parseInt(readLine(sin));
			if(!_isConnected) break;
			continue;
		    } else if(msgtype.equals("stack_dump")) {
			_titles.clear();
			int depth=Integer.parseInt(readLine(sin));
			if(_menus.size()>depth) {
			    System.err.println("Bad stack_dump - more menus than titles!");
			    _isConnected=false;
			    break;
			}
			for(int i=_menus.size(); i<depth; i++)
			    _menus.insertElementAt(new Vector(),0);
			for(int i=0; i<depth && _isConnected; i++)
			    _titles.add(readLine(sin));
			if(!_isConnected) break;
		    } else if(msgtype.equals("goodbye")) {
			//closeDynObjs();
			System.out.println("Remote is shutting down.");
		    } else if(msgtype.equals("reset")) {
			_menus.clear();
			_menus.add(new Vector());
			_titles.clear();
		    } else if(msgtype.equals("status")) {
			int nlines = Integer.parseInt(readLine(sin));
			if(nlines==-1)
			    _isConnected=false;
			if(!_isConnected) break;
			_status=readLine(sin);
			if(!_isConnected) break;
			for(; nlines>0; nlines--) {
			    _status+="\n"+readLine(sin);
			    if(!_isConnected) break;
			}
		    } else if(msgtype.equals("load")) {
			String type=readLine(sin);
			if(!_isConnected) break;
			String name=readLine(sin);
			if(!_isConnected) break;
			int port=Integer.parseInt(readLine(sin));
			if(!_isConnected) break;
			String args=readLine(sin);
			if(!_isConnected) break;
			String[] argArr=parseArgs(args);
			if(!_isConnected) break;
			if(dynObjPorts.get(name)!=null) //it's already open
			    if(((Integer)dynObjPorts.get(name)).intValue()==port) //and on the same port
				continue; //so don't reopen it
			Class objClass=null;
			try {
			    objClass=Class.forName(type);
			} catch(Exception ex) { System.out.println("Could not load "+type+": "+ex); }
			if(objClass!=null) {
			    Class[] consArgClasses=new Class[3];
			    Object[] consArgs=new Object[3];
			    consArgs[0]=_host;
			    consArgs[1]=new Integer(port);
			    consArgs[2]=argArr;
			    for(int i=0;i<consArgs.length;i++)
				consArgClasses[i]=consArgs[i].getClass();
			    consArgClasses[1]=Integer.TYPE;
			    Constructor objCons=objClass.getConstructor(consArgClasses);
			    Object obj=objCons.newInstance(consArgs);
			    if(port!=0 && name.length()>0) {
				dynObjs.put(name,obj);
				dynObjPorts.put(name, new Integer(port));
				if(dynObjSrcs.get(name)==null) { //check the GUI didn't already assign a src (for VisionGUI)
				    //find out who launched this (not infallable - dynamic controls launching windows may not have stable launch points...)
				    int minsel=-1;
				    if(_menus.size()>1) {
					Vector menuitems=(Vector)_menus.get(_menus.size()-2);
					for(int i=0; i<menuitems.size(); i++)
					    if(((ControllerListener.MenuEntry)menuitems.get(i)).selected) {
						minsel=i;
						break;
					    }
					String title=((ControllerListener.MenuEntry)((Vector)_menus.get(_menus.size()-2)).get(minsel)).title;
					Vector path=buildSelectionPath();
					String cmd=new String();
					for(int i=1; i<path.size()-1; i++)
					    cmd=cmd+"\""+path.get(i)+"\" ";
					cmd=cmd+"\""+title+"\"";
					System.out.println("Launched by: "+cmd);
					dynObjSrcs.put(name,cmd);
				    }
				}
			    }
			}
		    } else if(msgtype.equals("refreshsketchworld")) {
			// refresh world sketch
			if(gui.worldSkGUI!=null)
			    gui.worldSkGUI.autoRefreshSketch();
		    } else if(msgtype.equals("refreshsketchlocal")) {
			// refresh local sketch
			if(gui.localSkGUI!=null)
			    gui.localSkGUI.autoRefreshSketch();
		    } else if(msgtype.equals("refreshsketchcamera")) {
			// refresh camera sketch
			if(gui.cameraSkGUI!=null)
			    gui.cameraSkGUI.autoRefreshSketch();
		    } else if(msgtype.equals("close")) {
			String name=readLine(sin);
			if(!_isConnected) break;
			Object obj=dynObjs.get(name);
			if(obj==null) {
			    System.out.println("ControllerGUI: error - could not close "+name+" - not loaded");
			} else {
			    try { 
				Method m=obj.getClass().getMethod("close",(java.lang.Class[])null);
				m.invoke(obj,(java.lang.Object[])null);
			    } catch (Exception e) {}
			}
			dynObjs.remove(name);
			dynObjSrcs.remove(name);
			dynObjPorts.remove(name);
		    } else {
			System.err.println("ControllerListener - Invalid message type:"+msgtype);
		    }
		    _updatedFlag=true;
		} //end of synchronized update section
		if(gui!=null) {
		    //System.out.println("Listener repaint");
		    //gui.repaint();
		    gui.updated();
		} else
		    System.out.println("null gui");
	    }
	} catch (SocketException e) {
	} catch (Exception e) {
	    e.printStackTrace();
	} finally {
	    fireDisconnected();
	}
		
	try { socket.close(); } catch (Exception ex) { }

	//		while(_updatedFlag)
	//				try { Thread.sleep(100); } catch (Exception ex) {}

	_isConnected=false;
	_updatedFlag=true;
	_status="Reconnecting.";
	if(gui!=null) {
	    gui.lostConnection();
	    //gui.repaint();
	}
	//The sleep is to get around the socket still listening after being closed thing
	//if(!destroy)
	//System.out.println("ControllerGUI - connection closed... reconnect after 5 seconds");
	//try { Thread.sleep(5000); } catch (Exception ex) {}
    }

    public void removeDynObjs() {
	Iterator it=dynObjs.values().iterator();
	while(it.hasNext()) {
	    Object obj=it.next();
	    try {
		Method m=obj.getClass().getMethod("close",(java.lang.Class[])null);
		m.invoke(obj,(java.lang.Object[])null);
	    } catch(Exception ex) {}
	}
	dynObjs.clear();
	dynObjPorts.clear();
	dynObjSrcs.clear();
    }
	
    public void closeDynObjs() {
	Iterator it=dynObjs.values().iterator();
	while(it.hasNext()) {
	    Object obj=it.next();
	    try {
		Method m=obj.getClass().getMethod("close",(java.lang.Class[])null);
		m.invoke(obj,(java.lang.Object[])null);
	    } catch(Exception ex) {}
	}
	dynObjs.clear();
	dynObjPorts.clear();
    }
	
    public boolean hasData() {
	return _updatedFlag;
    }

    public boolean isConnected() {
	return _isConnected;
    }

    public String[] parseArgs(String args) throws java.io.IOException {
	Vector v=new Vector();
	StringBuffer cur=new StringBuffer();
	boolean isDoubleQuote=false;
	boolean isSingleQuote=false;
	do {
	    for(int i=0; i<args.length(); i++) {
		char c=args.charAt(i);
		switch(c) {
		case ' ':
		case '\n':
		case '\r':
		case '\t':
		case '\f':
		    if(isSingleQuote || isDoubleQuote)
			cur.append(c);
		    else if(cur.length()>0) {
			v.add(cur.toString());
			cur.setLength(0);
		    }
		    break;
		case '\\':
		    if(i==args.length()-1) { //escaped line break
			args=readLine(sin);  //get next line and continue
			if(!isConnected()) {
			    if(cur.length()>0)
				v.add(cur.toString());
			    return makeArray(v);
			}
			if(isSingleQuote || isDoubleQuote)
			    cur.append('\n');
			i=-1;
		    } else
			cur.append(args.charAt(++i));
		    break;
		case '"':
		    if(isSingleQuote)
			cur.append(c);
		    else
			isDoubleQuote=!isDoubleQuote;
		    break;
		case '\'':
		    if(isDoubleQuote)
			cur.append(c);
		    else
			isSingleQuote=!isSingleQuote;
		    break;
		default:
		    cur.append(c);
		    break;
		}
	    } 
	} while(isDoubleQuote || isSingleQuote);
	if(cur.length()>0)
	    v.add(cur.toString());
	return makeArray(v);
    }

    protected String[] makeArray(Vector v) {
	String[] ans=new String[v.size()];
	for(int i=0; i<v.size(); i++)
	    ans[i]=(String)v.get(i);
	return ans;
    }

    public ControllerListener() { super(); }
    public ControllerListener(int port) { super(port); }
    public ControllerListener(String host, int port) { super(host,port); }
}
