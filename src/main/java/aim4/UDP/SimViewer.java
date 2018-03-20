package aim4.UDP;

import java.util.ArrayList;

import javafx.geometry.Point2D;

/**
 * A set of classes to hold data from UDP communication
 * @author Alexander Humphry
 * @deprecated not used in current communications solution
 */
public class SimViewer {
	String recievedString;
	
	double time;
	double timeStep;
	
	int vehiclesCompleted;
	double bytesTransmittedAvg;
	double bytesReceivedAvg;
	
	ArrayList<VehicleViewer> vehicles = new ArrayList<VehicleViewer>();
	
	public SimViewer(String recievedString) {
		this.recievedString = recievedString;
		parse(recievedString, true);
	}
	
	public String getRecievedString() {
		return recievedString;
	}

	public double getTime() {
		return time;
	}

	public double getTimeStep() {
		return timeStep;
	}

	public int getVehiclesCompleted() {
		return vehiclesCompleted;
	}

	public double getBytesTransmittedAvg() {
		return bytesTransmittedAvg;
	}

	public double getBytesReceivedAvg() {
		return bytesReceivedAvg;
	}

	public ArrayList<VehicleViewer> getVehicles() {
		return vehicles;
	}
	public String getText(){
		return recievedString;
	}
	private void parse(String recievedString, boolean simple) {
		if(recievedString.length() < 8){
			return;
		}
		String[] splitString = recievedString.split("##");
		int index = 1;
		
		while(splitString[index] != null && !splitString[index].equals("End")){
			String[] splitVehicleString = splitString[index].split("#-#");
			
			Point2D position = new Point2D(Double.parseDouble(splitVehicleString[0]),Double.parseDouble(splitVehicleString[1]));
			
			double heading = Double.parseDouble(splitVehicleString[2]);
			
			double length = Double.parseDouble(splitVehicleString[3]);
			double width = Double.parseDouble(splitVehicleString[4]);
			vehicles.add(new VehicleViewer(0, null, position, heading, 0, 0, 0, length, width));
			index++;
		}
	}
	private void parse(String recievedString) {
		int index = 6;
		
		String[] splitString = recievedString.split("##");
		//record basic simulator data
		time = Double.parseDouble(splitString[0]);
		timeStep = Double.parseDouble(splitString[1]);
		
		vehiclesCompleted = Integer.parseInt(splitString[2]);
		bytesTransmittedAvg = Double.parseDouble(splitString[3]);
		bytesReceivedAvg = Double.parseDouble(splitString[4]);
		//store vehicle information
		if(!splitString[5].equals("Vehicles")){
			return;
		}
		
		while(splitString[index] != null && !splitString[index].equals("Map")){
			String[] splitVehicleString = splitString[index].split("#-#");
			int vin = Integer.parseInt(splitVehicleString[0]);
			String type = splitVehicleString[1];
			
			Point2D position = new Point2D(Double.parseDouble(splitVehicleString[2]),Double.parseDouble(splitVehicleString[3]));
			
			double heading = Double.parseDouble(splitVehicleString[4]);
			double velocity = Double.parseDouble(splitVehicleString[5]);
			double acceleration = Double.parseDouble(splitVehicleString[6]);
			double steeringAngle = Double.parseDouble(splitVehicleString[7]);
			
			double length = Double.parseDouble(splitVehicleString[8]);
			double width = Double.parseDouble(splitVehicleString[9]);
			vehicles.add(new VehicleViewer(vin, type, position, heading, velocity, acceleration, steeringAngle, length, width));
			index++;
		}
	}
}
class VehicleViewer{


	int vin;
	String type;
	
	Point2D position;
	
	double heading;
	double velocity;
	double acceleration;
	double steeringAngle;
	
	double length;
	double width;
	
	public  VehicleViewer(int vin, String type, Point2D position, double heading, double velocity, double acceleration, double steeringAngle, double length, double width){
		
		this.vin = vin;
		this.type = type;
		
		this.position = position;
		
		this.heading = heading;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.steeringAngle = steeringAngle;
		
		this.length = length;
		this.width = width;
	}
	public int getVin() {
		return vin;
	}

	public void setVin(int vin) {
		this.vin = vin;
	}

	public String getType() {
		return type;
	}

	public void setType(String type) {
		this.type = type;
	}

	public Point2D getPosition() {
		return position;
	}

	public void setPosition(Point2D position) {
		this.position = position;
	}

	public double getHeading() {
		return heading;
	}

	public void setHeading(double heading) {
		this.heading = heading;
	}

	public double getVelocity() {
		return velocity;
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	public double getAcceleration() {
		return acceleration;
	}

	public void setAcceleration(double acceleration) {
		this.acceleration = acceleration;
	}

	public double getSteeringAngle() {
		return steeringAngle;
	}

	public void setSteeringAngle(double steeringAngle) {
		this.steeringAngle = steeringAngle;
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}

	public double getWidth() {
		return width;
	}

	public void setWidth(double width) {
		this.width = width;
	}
}
