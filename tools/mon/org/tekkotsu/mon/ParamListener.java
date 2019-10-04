package org.tekkotsu.mon;

// Send/receive named parameters between TekkotsuMon to the AIBO.
import java.lang.Integer;
import java.lang.String;
import java.lang.System;
import java.io.PrintStream;
import java.io.InputStream;
import java.net.Socket;
import javax.swing.Timer;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Vector;
import java.net.SocketException;
import org.xml.sax.*;
import javax.xml.parsers.*;
import javax.xml.transform.*;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.helpers.DefaultHandler;
import org.xml.sax.helpers.XMLReaderFactory;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.Iterator;

// Does communication with the robot
public class ParamListener extends TCPListener implements ActionListener {
  // The command output stream
  PrintStream out;
  Socket mysock;
	Vector listeners=new Vector();
	static DocumentBuilderFactory dbf=DocumentBuilderFactory.newInstance();
	static DocumentBuilder db;
	static TransformerFactory transf = TransformerFactory.newInstance();
	static Transformer trans;
		
	static int defPort=10055;
	
	public interface CommListener {
		public void commUpdated(ParamListener mc);
	}

	void addCommListener(CommListener mcl) { listeners.add(mcl); needConnection(); }
	void removeCommListener(CommListener mcl) { listeners.remove(mcl); }
	void fireCommUpdated() {
		for(int i=0;i<listeners.size();i++)
			((CommListener)listeners.get(i)).commUpdated(this);
	}
	
	public class IncomingParser extends DefaultHandler {
		ParamListener parent;
		HashMap params = new HashMap();
		boolean updating=false;
		IncomingParser(ParamListener p) {
			super();
			parent=p;
		}
		public void startElement(String namespace, String localname, String qName, Attributes attr) throws SAXException{
			if(localname.equals("param")) {
				String name=attr.getValue("","name");
				String value=attr.getValue("","value");
				if(name==null || value==null)
					throw new SAXException("bad parameter update command - missing name or value");
				synchronized(params) {
					params.put(name,value);
				}
			} else if(localname.equals("refresh")) {
				parent.actionPerformed(null);
			} else if(localname.equals("update")) {
				synchronized(params) {
					updating=true;
				}
			} else if(localname.equals("connection")) {
			} else {
				System.out.println("Warning: unknown element "+localname);
			}
		}
		public void endElement(String namespace, String localname, String qName) {
			if(localname.equals("param")) {
				if(!updating)
					parent.fireCommUpdated();
			} else if(localname.equals("update")) {
				synchronized(params) {
					updating=false;
					params.notifyAll();
				}
				parent.fireCommUpdated();
			}
		}
	}
	IncomingParser parser=new IncomingParser(this);

	public String getParam(String name) {
		synchronized(parser.params) {
			return (String)parser.params.get(name);
		}
	}

	public Set getParams() {
		synchronized(parser.params) {
			return parser.params.keySet();
		}
	}

  // Connect to control socket
  public void connected(Socket socket) {
    mysock = socket;
		fireCommUpdated();
    try {
      out = new PrintStream(mysock.getOutputStream());
			out.println("<connection>");
			InputStream sin=socket.getInputStream();
			fireConnected();
			XMLReader myReader = XMLReaderFactory.createXMLReader();
			myReader.setContentHandler(parser);
			myReader.parse(new InputSource(sin));
    } catch(SocketException e) {
    } catch(Exception e) {
      e.printStackTrace();
    } finally {
      fireDisconnected();
    }

		try { socket.close(); } catch (Exception ex) { }

		_isConnected=false;
		fireCommUpdated();
		//The sleep is to get around the socket still listening after being closed thing
		if(!destroy)
			System.out.println("ParamListener - connection closed... reconnect after 5 seconds");
		try { Thread.sleep(5000); } catch (Exception ex) {}
  }

  // Disconnect from control socket
  public void close() {
		if(out!=null)
			out.println("</connection>");
		//    try { mysock.close(); } catch(Exception e) {}
    //_isConnected = false;
    super.close();
		//we'll fire an event to the listeners when the readLine in connected finally fails
  }

	public void actionPerformed(ActionEvent e) {
		if(!_isConnected || out==null)
			return;
		try {
			while(true) {
				if(parser.updating)
					parser.params.wait();
				synchronized(parser.params) {
					if(parser.updating)
						continue;
					Document doc=db.newDocument();
					Element root=doc.createElement("update");
					doc.appendChild(root);
					Set entries=parser.params.entrySet();
					for(Iterator it=entries.iterator();it.hasNext();) {
						Map.Entry ent=(Map.Entry)it.next();
						Element elm=doc.createElement("param");
						elm.setAttribute("name",(String)ent.getKey());
						elm.setAttribute("value",(String)ent.getValue());
						root.appendChild(elm);
					}
					trans.transform(new DOMSource(doc),new StreamResult(out));
					out.print("\n");
					break;
				}
			}
		} catch(Exception ex) {
			System.err.println("Dropped action event");
			ex.printStackTrace();
		}
	}

  // Send a headPoint command
  public void setParam(String command, String value) {
		synchronized(parser.params) {
			parser.params.put(command,value);
		}
    if (out == null)
      return;
    try {
			Document doc=db.newDocument();
			Element elm=doc.createElement("param");
			elm.setAttribute("name",command);
			elm.setAttribute("value",value);
			doc.appendChild(elm);
			trans.transform(new DOMSource(doc),new StreamResult(out));
			out.print("\n");
    } catch(Exception e) { close(); return; }
  }

  // Some state inquiry functions
  public boolean hasData() { return false; }
  public boolean isConnected() { return _isConnected; }

  // Constructors
  public ParamListener() { super(); init(); }
  public ParamListener(int port) { super(port); init(); }
  public ParamListener(String host, int port) { super(host, port); init(); }
	void init() {
		try {
			db = dbf.newDocumentBuilder();
			trans = transf.newTransformer();
			trans.setOutputProperty(OutputKeys.OMIT_XML_DECLARATION,"yes");
    } catch(Exception e) { e.printStackTrace(); }
	}

	static public void main(String s[]) {
		int port=defPort;
		if(s.length<1)
			usage();
		if(s.length>1)
			port=Integer.parseInt(s[1]);
		ParamListener l=new ParamListener(s[0],port);
	}
	
	public static void usage() {
		System.out.println("Usage: java ParamListener host [port]");
		System.out.println("       if port is not specified, it defaults to: "+defPort);
		System.exit(2);
	}
}
