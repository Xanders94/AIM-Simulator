package aim4.UDP;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * 
 * @author Alexander Humphry
 * sends data over UDP
 */
public class UDPSocket {
	
	protected int port;
	protected InetAddress address;
	protected DatagramSocket socket = null;
	protected DatagramPacket packet;
	//protected byte[] buffer = new byte[16384];
	protected byte[] buffer = new byte[4600];
	
	UDPSocket(int port, String address, boolean offsetPositive){
		this.port = port;
		try {
			this.address = InetAddress.getByName(address);
			if(offsetPositive){
				socket = new DatagramSocket(port + 1);
			} else {
				socket = new DatagramSocket(port - 1);
			}
		} catch (UnknownHostException | SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public int getPort(){
		return port;
	}
	public String getAddress(){
		return address.getHostAddress();
	}
	public int getBufferLength(){
		return buffer.length;
	}
	
	public void setPort(int portNumber){
		this.port = portNumber;
		try {
			this.socket = new DatagramSocket(this.port + 1);
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setAddress(String address){
		try {
			this.address = InetAddress.getByName(address);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void send(String outgoing){
		buffer = outgoing.getBytes();
		packet = new DatagramPacket(buffer, buffer.length, address, port);
		//test
		//System.out.println(outgoing);
		try {
			socket.send(packet);
			//System.out.println(outgoing);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public String recieve(){
		packet = new DatagramPacket(buffer, buffer.length);
		try {
			socket.receive(packet);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return new String(packet.getData(), 0, packet.getLength());
	}
}
