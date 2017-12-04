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
	
	protected HashMap<Double,String> sendBuffer = new HashMap<Double,String>();
	private double latestTime = -1;
	
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
			e.printStackTrace();
		}
	}
	public void setAddress(String address){
		try {
			this.address = InetAddress.getByName(address);
		} catch (UnknownHostException e) {
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
		} catch (Exception e) {
			System.err.print(e.getMessage());
		}
	}
	public String recieve(){
		packet = new DatagramPacket(buffer, buffer.length);
		try {
			socket.receive(packet);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return new String(packet.getData(), 0, packet.getLength());
	}
	
	public void sendDelay(String outgoing, double time, double fps){
		//Initialization case
		if(latestTime == -1){
			latestTime = time;
			sendBuffer.put(time, outgoing);
		}
		//check need to interpolate
		if((time - latestTime) > (1/fps)){
			//interpolate
		}
		//send real and interpolated values
		//for(String toSend: sendBuffer.)
	}
}
/**
 * contains all 53 vehicles to send and interpolates between its values and those given to it
 * @author Alexander
 *
 */
class SendObject{
	double time;
	int numberOfEntries;
	boolean original;
	
	ArrayList<Integer> vin = new ArrayList<Integer>();
	ArrayList<Double> xCoord = new ArrayList<Double>();
	ArrayList<Double> yCoord = new ArrayList<Double>();
	ArrayList<Double> heading = new ArrayList<Double>();
	ArrayList<Double> velocityX = new ArrayList<Double>();
	ArrayList<Double> velocityY = new ArrayList<Double>();
	ArrayList<Double> steering = new ArrayList<Double>();
	ArrayList<Double> tireRoll = new ArrayList<Double>();
	
	SendObject(double time){
		this.time = time;
		this.numberOfEntries = 0;
		this.original = true;
	}
	SendObject(double time, boolean original){
		this.time = time;
		this.numberOfEntries = 0;
		this.original = original;
	}
	public void addToSet(int vin1, double xCoord1, double yCoord1, double heading1, double velocityX1, 
			double velocityY1, double steering1, double tireRoll1){
		vin.add(vin1);
		xCoord.add(xCoord1);
		yCoord.add(yCoord1);
		heading.add(heading1);
		velocityX.add(velocityX1);
		velocityY.add(velocityY1);
		steering.add(steering1);
		tireRoll.add(tireRoll1);
		numberOfEntries++;
	}
	public void addToSet(double xCoord1, double yCoord1, double heading1, double velocityX1, 
			double velocityY1, double steering1, double tireRoll1){
		vin.add(numberOfEntries);
		xCoord.add(xCoord1);
		yCoord.add(yCoord1);
		heading.add(heading1);
		velocityX.add(velocityX1);
		velocityY.add(velocityY1);
		steering.add(steering1);
		tireRoll.add(tireRoll1);
		numberOfEntries++;
	}
	public Double[] getDatasetByVin(int vin){
		Double[] result = new Double[8];
		result[0] = vin * 1.0;
		result[1] = xCoord.get(vin);
		result[2] = yCoord.get(vin);
		result[3] = heading.get(vin);
		result[4] = velocityX.get(vin);
		result[5] = velocityY.get(vin);
		result[6] = steering.get(vin);
		result[7] = tireRoll.get(vin);
		return result;
	}
	public ArrayList<SendObject> interpolateTo(SendObject target, int fps){
		ArrayList<SendObject> result = new ArrayList<SendObject>();
		//interpolation parameters
		double deltaTime = target.time - this.time;
		int interpolationPoints = (int) deltaTime * fps - 1;
		//check if interpolation needed, only required if missing frame is noticiable distance from 
		if(deltaTime * fps < 1.75){
			return result;
		}
		double timeIntervals = 1/fps; // in seconds
		
		//main loop
		for(int j = 0; j < interpolationPoints; j++){
			for(int i = 0; i < Math.min(numberOfEntries, target.numberOfEntries); i++){
				//TODO
			}
			
		}
		return result;
	}
	public String toString(){
		String outgoing = "Start#";
		for(int i = 0; i < vin.size(); i++){
			outgoing += vin.get(i)
					//+ "#" + bVehicle.getVIN() + "<=="
					+ "#" + xCoord.get(i)
					+ "#" + yCoord.get(i)
					+ "#" + heading.get(i)
					+ "#" + velocityX.get(i)
					+ "#" + velocityY.get(i)
					+ "#" + steering.get(i)
					+ "#" + tireRoll.get(i)
					+ "#";
		}
		return outgoing + "End";
	}
}
