package org.tekkotsu.mon;

import java.io.InputStream;
import java.net.Socket;
import java.util.Vector;
import java.util.Map;
import java.util.HashMap;
import java.net.SocketException;

public class WorldStateJointsListener extends TCPListener {
	boolean _updatedFlag=false;
	Joints _data;
	Joints _outd;
	static final public int defPort=10031;
	
	Vector listeners=new Vector();
	public interface UpdatedListener {
		public void worldStateUpdated(WorldStateJointsListener mc);
	}
	public void addUpdatedListener(UpdatedListener mcl) { listeners.add(mcl); needConnection(); }
	public void removeUpdatedListener(UpdatedListener mcl) {
		listeners.remove(mcl);
		if(listeners.size()==0)
			kill();
	}
	void fireUpdated() {
		_updatedFlag=true;
		if(listeners==null) {
			System.out.println("wtf?");
			listeners=new Vector();
		}
		for(int i=0;i<listeners.size();i++)
			((UpdatedListener)listeners.get(i)).worldStateUpdated(this);
	}
	
	static Map commonListeners=new HashMap();
	static public WorldStateJointsListener getCommonInstance(String host) {
		return getCommonInstance(host,defPort);
	}
	static public WorldStateJointsListener getCommonInstance(String host, int port) {
		WorldStateJointsListener wsjl=(WorldStateJointsListener)commonListeners.get(host+":"+port);
		if(wsjl!=null)
			return wsjl;
		//not found, need new connection
		commonListeners.put(host+":"+port,wsjl=new WorldStateJointsListener(host,port));
		return wsjl;
	}
	
	public void connected(Socket socket) {
		_isConnected=true;
		_data=new Joints();
		_outd=new Joints();
		try {
			InputStream in=socket.getInputStream();
			fireConnected();
			StringBuffer model=new StringBuffer();
			while (true) {
				//System.out.print("reading...");
				model.delete(0,model.length());
				char c;
				while((c=readChar(in))!='\0') {
					if(!_isConnected)
						break;
					model.append(c);
				}
				_data.model=model.toString();
				//System.out.print(_data.model+"...");
				
				if(!_isConnected) break; //System.out.println("got packet from "+model);
				_data.timestamp=readUnsignedInt(in);
				if(!_isConnected) break; //System.out.println("time="+_data.timestamp);
				_data.frame=readUnsignedInt(in);
				if(!_isConnected) break; //System.out.println("frame="+_data.frame);
				
				int NumOutputs=readInt(in);
				if(!_isConnected) break; //System.out.println("outputs="+NumOutputs);
				if(_data.positions.length!=NumOutputs)
					_data.positions=new float[NumOutputs];
				for (int i=0; i<_data.positions.length; i++)
					_data.positions[i]=readFloat(in);
				if(!_isConnected) break;
				
				int NumSensors=readInt(in);
				if(!_isConnected) break; //System.out.println("sensors="+NumSensors);
				if(_data.sensors.length!=NumSensors)
					_data.sensors=new float[NumSensors];
				for (int i=0; i<_data.sensors.length; i++)
					_data.sensors[i]=readFloat(in);
				if(!_isConnected) break;
				
				int NumButtons=readInt(in);
				if(!_isConnected) break; //System.out.println("buttons="+NumButtons);
				if(_data.buttons.length!=NumButtons)
					_data.buttons=new float[NumButtons];
				for (int i=0; i<_data.buttons.length; i++)
					_data.buttons[i]=readFloat(in);
				if(!_isConnected) break;
				
				int NumPIDJoints=readInt(in);
				if(!_isConnected) break; //System.out.println("pids="+NumPIDJoints);
				if(_data.duties.length!=NumPIDJoints)
					_data.duties=new float[NumPIDJoints];
				for (int i=0; i<_data.duties.length; i++)
					_data.duties[i]=readFloat(in);
				if(!_isConnected) break;
				
				//System.out.print("updating...");
				synchronized(_outd) {
					Joints temp=_data;
					_data=_outd;
					_outd=temp;
					fireUpdated();
				}
				//System.out.println("done");
			}
			//System.out.print("closing...");
		} catch (SocketException e) {
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			fireDisconnected();
		}
		
		try { socket.close(); } catch (Exception ex) { }
		_isConnected=false;
		//System.out.println("closed");
	}
	
	public boolean hasData() {
		return _updatedFlag;
	}
	
	public Joints getData() {
		synchronized (_outd) {
			_updatedFlag=false;
			return _outd;
		}
	}
	
	public boolean isConnected() {
		return _isConnected;
	}
	
	public WorldStateJointsListener() { super(); }
	public WorldStateJointsListener(int port) { super(port); }
	public WorldStateJointsListener(String host, int port) { super(host,port); }
}
