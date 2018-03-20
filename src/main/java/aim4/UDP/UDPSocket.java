package aim4.UDP;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Manages a pair of ports for the purpose of sending and receiving UDP messages over a network
 * between AIM4 and a SimCreator scenario
 * @author Alexander Humphry
 */
public class UDPSocket {
	
	protected int port;
	protected InetAddress address;
	protected DatagramSocket socket = null;
	protected DatagramPacket packet;
	//protected byte[] buffer = new byte[16384];
	protected byte[] buffer = new byte[4600];
	
	protected HashMap<Double,String> sendBuffer = new HashMap<Double,String>();
	private double latestTime = -1;
	/**
	 * Initializes a two way UDP connection interface between AIM4 and a connected network
	 * @param port the port of the target SimCreator simulator
	 * @param address the address of the target SimCreator simulator
	 * @param offsetPositive - if set to true, then sets the receiving port to +1 of the sending port number. Else, sets the receiving port to -1 of the sending port
	 */
	public UDPSocket(int port, String address, boolean offsetPositive){
		this.port = port;
		try {
			this.address = InetAddress.getByName(address);
			if(offsetPositive){
				socket = new DatagramSocket(port + 1);
			} else {
				socket = new DatagramSocket(port - 1);
			}
		} catch (UnknownHostException | SocketException e) {
			e.printStackTrace();
		}
	}
	/**
	 * 
	 * @return port responsible for sending communications
	 */
	public int getPort(){
		return port;
	}
	/**
	 * @return port responsible for receiving communications
	 */
	public int getReceivePort(){
		return this.socket.getPort();
	}
	/**
	 * @return the current network address for sending
	 */
	public String getAddress(){
		return address.getHostAddress();
	}
	/**
	 * @return the length of the current buffer
	 */
	public int getBufferLength(){
		return buffer.length;
	}
	/**
	 * Allows for the setting of the sending and receiving ports
	 * @param portNumber
	 */
	public void setPort(int portNumber, boolean offsetPositive){
		this.port = portNumber;
		try {
			if(offsetPositive){
				this.socket = new DatagramSocket(port + 1);
			} else {
				this.socket = new DatagramSocket(port - 1);
			}
			//this.socket = new DatagramSocket(this.port + 1);
		} catch (SocketException e) {
			e.printStackTrace();
		}
	}
	/**
	 * Allows for the changing of the destination IP address targeted by the sent UDP packets
	 * @param address
	 */
	public void setAddress(String address){
		try {
			this.address = InetAddress.getByName(address);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
	}
	/**
	 * sends a string message over a network, provided an address and port number have been set for this instance of UDPSocket
	 * @param outgoing
	 */
	public void send(String outgoing){
		buffer = outgoing.getBytes();
		packet = new DatagramPacket(buffer, buffer.length, address, port);
		//test
		//System.out.println(outgoing);
		try {
			socket.send(packet);
			//System.out.println(outgoing);
		} catch (Exception e) {
			System.err.print(e.getMessage());
		}
	}
	/**
	 * listens on the specified receiving port (+/- sending port) for a string message
	 * @return
	 */
	public String recieve(){
		packet = new DatagramPacket(buffer, buffer.length);
		try {
			socket.receive(packet);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return new String(packet.getData(), 0, packet.getLength());
	}
}