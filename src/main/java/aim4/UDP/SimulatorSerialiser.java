package aim4.UDP;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;

import aim4.config.Debug;
import aim4.driver.AutoDriver;
import aim4.driver.ProxyDriver;
import aim4.im.IntersectionManager;
import aim4.im.v2i.V2IManager;
import aim4.im.v2i.reservation.ReservationManager;
import aim4.map.BasicMap;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.lane.Lane;
import aim4.msg.udp.Real2ProxyPVUpdate;
import aim4.sim.AutoDriverOnlySimulator;
import aim4.sim.Simulator;
import aim4.vehicle.VehicleSimView;
import aim4.vehicle.VehicleSpec;
import aim4.vehicle.VehicleSpecDatabase;
import aim4.vehicle.VinRegistry;
import aim4.vehicle.BasicVehicle;
import aim4.vehicle.ProxyVehicle;
import aim4.vehicle.ProxyVehicleSimView;
import aim4.vehicle.AutoVehicleSimView;
import aim4.vehicle.BasicAutoVehicle;

/**
 * Takes a simulator class and creates a String to send via UDP
 * warning, only works with a single 4 way intersection
 * @author Alexander Humphry
 */
public class SimulatorSerialiser {
	
	  /**
	   * 
	   * specifies the origin/destination pair for the player vehicle
	   * 
	   * @author Alexander Humphry
	   */
	  public enum ODPair {
		  NORTH_SOUTH,
		  NORTH_EAST,
		  NORTH_WEST,
		  
		  SOUTH_NORTH,
		  SOUTH_EAST,
		  SOUTH_WEST,
		  
		  EAST_WEST,
		  EAST_NORTH,
		  EAST_SOUTH,
		  
		  WEST_EAST,
		  WEST_NORTH,
		  WEST_SOUTH,
	  }
	
	String sendString;
	AutoDriverOnlySimulator sim;
	UDPSocket connection;
	
	ArrayList<VehicleSimView> vehicles;
	ArrayList<VehicleSimView> outVehicles;
	Map<Integer,VehicleSimView> vinToVehicles;
	ArrayList<Integer> overwritePos;
	VehicleSimView playerVehicle;
	
	boolean recieveEnable = false;
	boolean initialised;
	boolean hasVehicle;
	
	ODPair path = null;
	double playerBid = 0;
	String simID = null;
	double playerSpawnTime;
	
	Random rand = new Random();
	
	/**a list of all proxy vehicles created in this instance of SimulatorSerialiser
	 * Serialize
	 */
	ArrayList<ProxyVehicle> playerVehicles = new ArrayList<ProxyVehicle>();
	
	/**
	 * a mapping of vehicles to their simcreator vin values
	 */
	HashMap<Integer,VehicleSimView> vehicleMapping = new HashMap<Integer,VehicleSimView>(52);
	/**
	 * Creates a basic simulator serialiser object initialized with an instance of a simulator and a target port and network address
	 * @param sim
	 * @param port target network port
	 * @param address target network address
	 */
	public SimulatorSerialiser(AutoDriverOnlySimulator sim, int port, String address){
		this.sim = sim;
		//this.connection = new UDPSocket(2501, "192.168.1.141");
		initialised = false;
		hasVehicle = false;
		this.connection = new UDPSocket(port, address, false);
		setPlayerVehicle(null);
	}
	/**
	 * creates a simulator serialiser object with an instance of a simulator and a target port and network address.
	 * in addition, the path of an autonomously controlled player vehicle may be specified
	 * @param sim
	 * @param port target network port
	 * @param address target network address
	 * @param path
	 */
	public SimulatorSerialiser(AutoDriverOnlySimulator sim, int port, String address, ODPair path){
		this.sim = sim;
		//this.connection = new UDPSocket(2501, "192.168.1.141");
		initialised = false;
		hasVehicle = false;
		this.connection = new UDPSocket(port, address, true);
		setPlayerVehicle(null);
		this.path = path;
	}
	/**
	 * creates a simulator serialiser object with an instance of a simulator and a target port and network address.
	 * in addition, the path of an autonomously controlled player vehicle may be specified with a bid parameter which may be
	 * used when bid or priority controlled intersection policies are in operation.
	 * @param sim
	 * @param port target network port
	 * @param address target network address
	 * @param path
	 * @param bid (usually a dollar amount or priority parameter)
	 * @param simID (used for debug as a label describing the name of the connected simulator, may be left blank)
	 */
	public SimulatorSerialiser(AutoDriverOnlySimulator sim, int port, String address, ODPair path, double bid, String simID){
		this.sim = sim;
		//this.connection = new UDPSocket(2501, "192.168.1.141");
		initialised = false;
		hasVehicle = false;
		this.connection = new UDPSocket(port, address, false);
		setPlayerVehicle(null);
		this.path = path;
		this.playerBid = bid;
		this.simID = simID;
	}
	
	public Simulator getSim(){
		return sim;
	}
	public void setSim(AutoDriverOnlySimulator sim){
		this.sim = sim;
	}
	public void setBid(double bid){
		this.playerBid = bid;
	}
	public double getBid(){
		return this.playerBid;
	}
	/**
	 * @return true if any vehicles have been generated and attached to this simulator serialiser object
	 */
	public boolean hasVehicles() {
		return hasVehicle;
	}
	/**
	 * set hasVehicles to given value
	 * @param vehicles whether this instance of simulator serialiser has vehicle associated with it
	 */
	public void setHasVehicles(boolean vehicles) {
		hasVehicle = vehicles;
	}
	public ArrayList<ProxyVehicle> getProxyVehicles(){
		return playerVehicles;
	}
	public VehicleSimView getPlayerVehicle(){
		return playerVehicle;
	}
	public void setPlayerVehicle(VehicleSimView playerVehicle) {
		this.playerVehicle = playerVehicle;
	}
	/**
	 * return true if simulator serialiser has been fully initialized and given any amount of player or non player vehicles
	 * @return whether this object is initialised
	 */
	public boolean isInitialised(){
		return this.initialised;
	}
	/**
	 * communicate the current simulator's vehicle list to this instance of simulator serialiser while generating a generic player vehicle
	 * @param timeStep the current simulation time step
	 * @param vinToVehicles a map of vehicles to their vin numbers
	 * @return whether a player vehicle was generated
	 */
	public boolean setVinToVehicles(double timeStep, Map<Integer,VehicleSimView> vinToVehicles){
		this.vinToVehicles = vinToVehicles;
		
		if(recieveEnable && this.generateProxyVehicle(timeStep) != null){
			initialised = true;
			return true;
		}
		return false;
	}
	/**
	 * communicate the current simulator's vehicle list to this instance of simulator serialiser while generating a player vehicle with a specified
	 * origin/destination pair and bid to control its behavior in the intersection and spawn location
	 * @param timeStep the current simulation time step
	 * @param vinToVehicles a map of vehicles to their vin numbers
	 * @param path the origin destination pair to be assigned as the path of the player vehicle in auto mode
	 * @param bid the bid to be submitted by the player vehicle in auto mode if a bidding policy is in effect
	 * @return whether a player vehicle was generated
	 */
	public boolean setVinToVehicles(double timeStep, Map<Integer,VehicleSimView> vinToVehicles, ODPair path, double bid){
		this.vinToVehicles = vinToVehicles;
		
		if(recieveEnable && this.generateProxyVehicle(timeStep) != null){
			initialised = true;
			return true;
		} else if(!recieveEnable && !initialised){
			initialised = true;
			//this.playerVehicle = generatePlayerVehicle(this.sim, path);
			setPlayerVehicle(generatePlayerVehicle(this.sim, path, bid));
			if(getPlayerVehicle() == null) {
				return false;
			}
			return true;
		}
		return false;
	}
	/**
	 * Send the current simulator parameters to the address and port targeted by this instance of simulator serialiser.
	 * @param timeStep the current simulation time step
	 */
	public void send(double timeStep){
		connection.send(serialize(timeStep));
	}
	/**
	 * Receive and process a message from a communicating SimCreator simulator
	 * @param timeStep the current simulation time step
	 * @return whether a transmission was successfully received and communicated to the player vehicle in AIM4
	 */
	public boolean recieve(double timeStep){
		return deSerialise(connection.recieve(),timeStep);
	}
	/**
	 * inject debug input to simulate communications with a SimCreator simulator
	 * @param timeStep the current simulation time step
	 * @param debugInput message, format: [mode]#start#[auto mode engaged (1/0)]#[positionX]#[positionY]#[velocityX]#[velocityY]#[heading]#end
	 * @return whether the test transmission was successfully communicated to the player vehicle in AIM4
	 */
	public boolean recieve(double timeStep,String debugInput){
		return deSerialise(debugInput,timeStep);
	}
	
	private boolean deSerialise(String recieve,double timeStep) {
		//message format start#<auto mode engaged (true/false)>#<positionX>#<positionY>#<velocityX>#
		//<velocityY>#<heading>#end
		String[] recieveSplit = recieve.split("#");
		double heading = 0;
		double velocity = 0;
		//if no proxy vehicle exists to be updated, return false
		if(getPlayerVehicle() == null){
			return false;
		}
		BasicAutoVehicle player = (BasicAutoVehicle) getPlayerVehicle();
		if(!(player.getDriver() instanceof ProxyDriver)){
			player.setDriver(new ProxyDriver(player, sim.getMap()));
		}
		//only lead proxy vehicle if aim has been given control
		if(recieveSplit[2].equals("1")){
			//send proxy vehicle movement and position data to proxy vehicle driver
			/*Real2ProxyPVUpdate msg = new Real2ProxyPVUpdate(player.getVIN(), Double.parseDouble(recieveSplit[3]) + 157.5, Double.parseDouble(recieveSplit[4]) + 157.5,
					rollOverBearing(Double.parseDouble(recieveSplit[6]) + Math.PI), 0, 0,
					0, 0, sim.getSimulationTime());
					//Math.sqrt(Math.pow(Double.parseDouble(recieveSplit[4]),2) + Math.pow(Double.parseDouble(recieveSplit[5]),2))
			player.processReal2ProxyMsg(msg);*/
			velocity = Math.sqrt(Math.pow(Double.parseDouble(recieveSplit[5]), 2) + Math.pow(Double.parseDouble(recieveSplit[6]), 2));
			heading = Double.parseDouble(recieveSplit[7]);
			player.reposition(new Point2D.Double(Double.parseDouble(recieveSplit[3]) + 157.5, Double.parseDouble(recieveSplit[4]) + 157.5)
			, rollOverBearing(heading), 0, 0, 0, 0);
			//update current lane occupied
			//TODO player.getDriver().setCurrentLane(getClosestLane(player));
			//TODO getClosestLane(player,true);
			//move vehicle
			player.move(timeStep);
			//update current lane occupied
			setOccupiedLanes(player);
			System.err.println("Player Vehicle Position: " + player.getPosition().getX() + "/"+ player.getPosition().getY() + "/Heading:"+player.gaugeHeading()/Math.PI);

		}
		return true;
	}
	//simple version of serialize, only gives the position and heading of a vehicle
	private String serialize(double timeStep){
		String outgoing = "2#Start#";
		double heading = 0;
		double xCoord = 0;
		double yCoord = 0;
		int vin = 0;
		vehicles = new ArrayList<VehicleSimView>();
		outVehicles = new ArrayList<VehicleSimView>();
		double tireRollAngle = 0;
		boolean recieveEnabled = true; // set true to receive communications from SimCreator with regards to the player vehicle
		boolean recieveEnabledTest = false;
		//retrieve vehicles from simulator active list
		@SuppressWarnings("unused")
		int vanNumber = 0;
		//debug
		
		for(VehicleSimView vehicle : sim.getActiveVehicles()){
			vehicles.add(vehicle);
			if(vehicle.getSpec().getName().equals("VAN")){
				vanNumber++;
			}
		}
		/*int index = 0;
		if(prevVehicles != null && !prevVehicles.isEmpty()){
			for(VehicleSimView vehicle : vehicles){
				//if position hasen't changed or vehicle in't in list, add to list
				//else find if in list
				if(vehicle.getVIN() == prevVehicles.get(index).getVIN() || !vehicles.contains(prevVehicles.get(index))){
					outVehicles.add(vehicle);
				} else {
					//since out of order, find in list and swap with vehicle
					for(int i = 0; i < vehicles.size(); i++){
						if(vehicles.get(i).getVIN() == prevVehicles.get(index).getVIN()){
							Collections.swap(vehicles, index, i);
							outVehicles.add(vehicles.get(index));
							
							break;
						}
					}
				}
				index++;
			}
		}
		outVehicles = vehicles;
		*/
		//retrieve simulator communication if proxyVehicle defined in AIM4
		if((getPlayerVehicle() != null) && recieveEnabled){
			this.recieve(timeStep);
		} else if((getPlayerVehicle() != null) && recieveEnabledTest){
			this.recieve(timeStep,"1#start#1#40#5#0#0#");
		}
		if(getPlayerVehicle() != null);
		//compute offset in front of proxy vehicle for usable set
		//compile list of usable vehicles and
		//map vehicles to set VIN numbers for simcreator model assignment
		//calculate offset for visibility circle around proxy vehicle
		if(!playerVehicles.isEmpty()){
			Point2D proxyPosOffset = getProximityOffset(0,500.0);
			//create list of vehicles to send to simcreator
			orderMapping(getClosestVehiclesToPoint(vehicles, 1300.0, proxyPosOffset.getX(), proxyPosOffset.getY()));
		}/* else if(false && playerVehicle != null) {
			Point2D proxyPosOffset = getProximityOffset(playerVehicle,500.0);
			orderMapping(getClosestVehiclesToPoint(vehicles, 1300.0, proxyPosOffset.getX(), proxyPosOffset.getY()));
		} */else {
			orderMapping(getClosestVehiclesToPoint(vehicles, 1000.0, 157.5, 157.5));
		}
		//check for missing vehicles from prev time around
		VehicleSimView vehicle = null;
		//place new vehicles if 
		for(int i = 0; i < 52; i++){
			if(!vehicleMapping.containsKey(i)){
				continue;
			} else {
				vehicle = vehicleMapping.get(i);
				vin = i;
			}
			if(!(vehicle instanceof BasicVehicle || vehicle instanceof BasicAutoVehicle)){
				continue;
			}
			BasicVehicle bVehicle = (BasicVehicle) vehicle;
			//format, VIN#-#positionX#-#positionY#-#heading#-#velocity#-#acceleration#-#steeringAngle#-#dataTransmitted#-#dataReceived
			//#-#length#-#width#-#bufferFactor#-#timeBufferFactor#--#
			
			/*process coordinates*/
			/*-160 comes from a specific scenario (4 lanes 4 way intersection)*/
			xCoord = bVehicle.getCenterPoint().getX() - 157.5;
			yCoord = bVehicle.getCenterPoint().getY() - 157.5;
			
			/*process heading (yaw) parameter*/
			if(bVehicle.gaugeHeading() < 0.000000000001){
				heading = 0;
			} else {
				heading = bVehicle.gaugeHeading();
			}
			double steering;
			try{
				steering = bVehicle.getSteeringAngle();
			} catch (Exception e){
				steering = 0.0;
			}
			//convert local velocity to global x and y velocities
			double velocityX = 0.0;
			double velocityY = 0.0;
			velocityY = bVehicle.gaugeVelocity() * Math.sin(heading);
			velocityX = bVehicle.gaugeVelocity() * Math.cos(heading);
			//assuming 17in wheel calc tire roll over time step
			double length = bVehicle.gaugeVelocity() * timeStep;
			double angle = (length * 360) / (4 * Math.PI * 0.2159);
			tireRollAngle = Math.toRadians(angle);
			//limit outgoing numbers to 3dp, assume number no larger than 10,000.9999
			//start# = 6, end = 3, vehicle = 53 * (7 + 11 * 7), proxy# = 6, total max packet = 4467 bytes, packet = 
			outgoing += vin
					//+ "#" + bVehicle.getVIN() + "<=="
					+ "#" + (float) Math.floor(xCoord * 10000)/10000
					+ "#" + (float) Math.floor(yCoord * 10000)/10000
					+ "#" + (float) Math.floor(heading * 10000)/10000
					+ "#" + (float) Math.floor(velocityX * 10000)/10000
					+ "#" + (float) Math.floor(velocityY * 10000)/10000
					+ "#" + (float) Math.floor(steering * 10000)/10000
					+ "#" + (float) Math.floor(tireRollAngle * 10000)/10000 //rotation angle of tires in radians
					+ "#";
			vin++;
		}
		//check playerVehicle isn't null and is 
		if(getPlayerVehicle() != null) {
			if(!sim.getActiveVehicles().contains(getPlayerVehicle())) {
				debugPrint(vehicle);
				setPlayerVehicle(null);
			}
		}
		//add playerVehicle
		
		if(!recieveEnabled && getPlayerVehicle() != null) {
			
			BasicVehicle bVehicle = (BasicVehicle) getPlayerVehicle();
			//format, VIN#-#positionX#-#positionY#-#heading#-#velocity#-#acceleration#-#steeringAngle#-#dataTransmitted#-#dataReceived
			//#-#length#-#width#-#bufferFactor#-#timeBufferFactor#--#
			
			/*process coordinates*/
			/*-160 comes from a specific scenario (4 lanes 4 way intersection)*/
			
			xCoord = bVehicle.getCenterPoint().getX() - 157.5;
			yCoord = bVehicle.getCenterPoint().getY() - 157.5;
			
			/*process heading (yaw) parameter*/
			if(bVehicle.gaugeHeading() < 0.000000000001){
				heading = 0 + Math.PI;
			} else {
				heading = bVehicle.gaugeHeading() + Math.PI;
			}
			/*if(false && this.connection.getAddress().equals("192.168.0.5")) {
				Debug.setVehicleColor(bVehicle.getVIN(), Color.MAGENTA);
				System.out.println("sim vehicle heading: " + heading);
			}*/
			double steering;
			try{
				steering = bVehicle.getSteeringAngle();
			} catch (Exception e){
				steering = 0.0;
			}
			//convert local velocity to global x and y velocities
			double velocityX = 0.0;
			double velocityY = 0.0;
			velocityY = bVehicle.gaugeVelocity() * Math.sin(heading);
			velocityX = bVehicle.gaugeVelocity() * Math.cos(heading);
			//assuming 17in wheel calc tire roll over time step
			double length = bVehicle.gaugeVelocity() * timeStep;
			double angle = (length * 360) / (4 * Math.PI * 0.2159);
			tireRollAngle = Math.toRadians(angle);
			//limit outgoing numbers to 3dp, assume number no larger than 10,000.9999
			//start# = 6, end = 3, vehicle = 53 * (7 + 11 * 7), proxy# = 6, total max packet = 4467 bytes, packet = 
			outgoing += 52
					//+ "#" + bVehicle.getVIN() + "<=="
					+ "#" + (float) Math.floor(xCoord * 10000)/10000
					+ "#" + (float) Math.floor(yCoord * 10000)/10000
					+ "#" + (float) Math.floor(heading * 10000)/10000
					+ "#" + (float) Math.floor(velocityX * 10000)/10000
					+ "#" + (float) Math.floor(velocityY * 10000)/10000
					+ "#" + (float) Math.floor(steering * 10000)/10000
					+ "#" + (float) Math.floor(tireRollAngle * 10000)/10000 //rotation angle of tires in radians
					+ "#";
			//vin++;
		} else if(!recieveEnabled){
			double offset = 12.0;// + 0.5 * 157.5;
			xCoord = -offset;// - 157.5;
			yCoord = offset;// - 157.5;

			//heading = 0.875 * 2 * Math.PI;
			heading = 0 + Math.PI;

			double steering = 0.0;
			//convert local velocity to global x and y velocities
			double velocityX = 0.0;
			double velocityY = 0.0;

			//assuming 17in wheel calc tire roll over time step
			double length = 0;
			double angle = (length * 360) / (4 * Math.PI * 0.2159);
			tireRollAngle = Math.toRadians(angle);
			//limit outgoing numbers to 3dp, assume number no larger than 10,000.9999
			//start# = 6, end = 3, vehicle = 53 * (7 + 11 * 7), proxy# = 6, total max packet = 4467 bytes, packet = 
			outgoing += 52
					//+ "#" + bVehicle.getVIN() + "<=="
					+ "#" + (float) Math.floor(xCoord * 10000)/10000
					+ "#" + (float) Math.floor(yCoord * 10000)/10000
					+ "#" + (float) Math.floor(heading * 10000)/10000
					+ "#" + (float) Math.floor(velocityX * 10000)/10000
					+ "#" + (float) Math.floor(velocityY * 10000)/10000
					+ "#" + (float) Math.floor(steering * 10000)/10000
					+ "#" + (float) Math.floor(tireRollAngle * 10000)/10000 //rotation angle of tires in radians
					+ "#";
		}
		
		/*if(initialised && playerVehicle != null) {
			Debug.setVehicleColor(vehicle.getVIN(), new Color(204,0,204));
		}*/
		
		//System.out.println(outgoing + "End");
		//System.out.println("number of vans in active is: " + vanNumber);
		//add information on this simulator's proxy vehicle if available
		
		//TODO disabled till connection test performed
		if(!playerVehicles.isEmpty()&& false){
			//gather proxy vehicle parameters (similar to autoVehicle above)
			BasicAutoVehicle player = playerVehicles.get(0);
			xCoord = player.getCenterPoint().getX() - 157.5;
			yCoord = player.getCenterPoint().getY() - 157.5;
			double velocityX = 0.0;
			double velocityY = 0.0;
			velocityY = player.gaugeVelocity() * Math.sin(heading);
			velocityX = player.gaugeVelocity() * Math.cos(heading);
			if(player.gaugeHeading() < 0.000000000001){
				heading = 0;
			} else {
				heading = player.gaugeHeading();
			}
			outgoing += "Proxy#"+player.getVIN()
					+ "#" + Math.floor(xCoord * 10000)/10000
					+ "#" + Math.floor(yCoord * 10000)/10000
					+ "#" + Math.floor(heading * 10000)/10000
					+ "#" + Math.floor(velocityX * 10000)/10000
					+ "#" + Math.floor(velocityY * 10000)/10000
					+ "#" + Math.floor(player.getAcceleration() * 10000)/10000
					+ "#" + Math.floor(player.getSteeringAngle() * 10000)/10000
					+ "#";
		}
		return outgoing + "End";
	}

	private void debugPrint(VehicleSimView vehicle) {
		//print out vin, spawn time, delistTime, bid, path
		double delta = sim.getSimulationTime()-playerSpawnTime;
		System.out.println(this.simID + 
				": Vin: " + playerVehicle.getVIN() + 
				", Spawn Time:" + playerSpawnTime + 
				", Finish Time:" + sim.getSimulationTime() + 
				", Delta Time:" + delta + 
				", Bid:" + playerBid +
				", Path Used:" + this.getPath()
				);
		
	}

	private Point2D getProximityOffset(int playerVehicle, double offsetDist) {
		//retrieve proxy vehicle
		ProxyVehicleSimView subject = null;
		try{
			subject = playerVehicles.get(playerVehicle);
		} catch (Exception e){
			return new Point2D.Double(this.sim.getMap().getDimensions().getBounds2D().getCenterX(),this.sim.getMap().getDimensions().getBounds2D().getCenterY()) ;
		}
		Point2D subjectPoint= subject.getCenterPoint();
		subjectPoint.setLocation(subjectPoint.getX() + Math.sin(subject.gaugeHeading())*offsetDist, subjectPoint.getY() + Math.cos(subject.gaugeHeading())*offsetDist);
		return subjectPoint;
	}
	
	private Point2D getProximityOffset(VehicleSimView vehicle, double offsetDist) {
		//retrieve proxy vehicle
		VehicleSimView subject = null;
		try{
			subject = vehicle;
		} catch (Exception e){
			return new Point2D.Double(this.sim.getMap().getDimensions().getBounds2D().getCenterX(),this.sim.getMap().getDimensions().getBounds2D().getCenterY()) ;
		}
		Point2D subjectPoint= subject.getCenterPoint();
		subjectPoint.setLocation(subjectPoint.getX() + Math.sin(subject.gaugeHeading())*offsetDist, subjectPoint.getY() + Math.cos(subject.gaugeHeading())*offsetDist);
		return subjectPoint;
	}

	//takes a list of vehicles, compares to a list of predecessor vehicles, outputs a list where any new vehicles replace
	//old vehicles that are no longer required
	private void orderMapping(ArrayList<VehicleSimView> vehicles) {
		
		//first: check if the values in mapping exist in the current vehicle list, if not, delete mapping
		for(int i = 0; i < vehicleMapping.size(); i++){
			//check if key exists
			if(vehicleMapping.containsKey(i)){
				//check that value attributed to key is in current vehicle list, if not, delete mapping
				if(!vehicles.contains(vehicleMapping.get(i))){
					vehicleMapping.remove(i);
				}
			}
		}
		//second: add new mappings for new vehicles in vehicle list
		for(VehicleSimView vehicle : vehicles){
			//if vehicle is player vehicle, try next vehicle
			if(getPlayerVehicle() != null && vehicle.getVIN() == getPlayerVehicle().getVIN()) {
				continue;
			}
			//if vehicle isn't in the mapping, add 
			if(!vehicleMapping.containsValue(vehicle)){
				for(int i = 0; i < vehicleMapping.size() || i < 52; i++){
					if(!vehicleMapping.containsKey(i)){
						//special assignment code for vans
						//exclude position 4,5,6 and 7 for vans
						if(vehicle.getSpec().getName().equals("VAN")){
							if(i >= 6 && i <= 10){
								vehicleMapping.put(i, vehicle);
								break;
							} else {
								continue;
							}
						} else {
							if(i < 6 || i > 10){
								vehicleMapping.put(i, vehicle);
								break;
							} else {
								i = 11;
								continue;
							}
						}
						/*this block contains the original code replaced by the "VAN" if block above
						vehicleMapping.put(i, vehicle);
						break;*/
					}
				}
			}
		}
	}
	//returns a list of vehicles positioned a given distance away from a given set of points
	//list is limited to the closest 52 vehicles not counting the proxy vehicle
	private ArrayList<VehicleSimView> getClosestVehiclesToPoint(ArrayList<VehicleSimView> vehicles, double proximityCutoff, double centerX, double centerY){
		HashMap<VehicleSimView, Double> vehicleToDistance = new HashMap<VehicleSimView,Double>();
		ArrayList<Double> distances = new ArrayList<Double>();
		ArrayList<VehicleSimView> outVehicles = new ArrayList<VehicleSimView>(); 
		double distance = 0;
		//if no vehicles, return the empty list
		if(vehicles.isEmpty()){
			return vehicles;
		}
		//create a new mapping
		for(VehicleSimView vehicle : vehicles){
			//check vehicle isn't the proxy vehicle
			if(!playerVehicles.isEmpty()){
				if(vehicle.getVIN() == playerVehicles.get(0).getVIN()){
					continue;
				}
			}
			//get distance to specified point and add to distance/vehicle mapping
			distance = vehicle.gaugePosition().distance(centerX, centerY);
			distances.add(distance);
			vehicleToDistance.put(vehicle, distance);
		}
		//sort the arraylist of distances in ascending order and get the 51st element to use as the proximity cutoff
		Collections.sort(distances);
		distance = distances.get(Math.min(distances.size() - 1, 51));
		if(proximityCutoff <= 0){
			proximityCutoff = distance;
		} else {
			proximityCutoff = Math.min(proximityCutoff, distance);
		}
		//return vehicles below the proximity cutoff value
		Iterator<Entry<VehicleSimView, Double>> it = vehicleToDistance.entrySet().iterator();
		while(it.hasNext()){
			HashMap.Entry<VehicleSimView, Double> pair = (HashMap.Entry<VehicleSimView, Double>) it.next();
			if(pair.getValue() <= proximityCutoff){
				outVehicles.add(pair.getKey());
			}
		}
		//if(outVehicles.size() > 48 || true){
			//System.out.println("the number of elements in outVehicles is: " + outVehicles.size());
		//}
		return outVehicles;
	}
	/**
	 * generates a player vehicle
	 * @param timeStep
	 * @return reference to the proxy vehicle
	 */
	public ProxyVehicle generateProxyVehicle(double timeStep){
		//spawn vehicle
		for(SpawnPoint spawnPoint : sim.getMap().getSpawnPoints()) {
		      List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
		      if (!spawnSpecs.isEmpty()) {
		        if (canSpawnVehicle(spawnPoint)) {
		          if(true || this.vinToVehicles.size() < (52 - spawnSpecs.size())){
		            for(SpawnSpec spawnSpec : spawnSpecs) {
		              ProxyVehicle vehicle = makeVehicle(spawnPoint, spawnSpec);
		              VinRegistry.registerVehicle(vehicle); // Get vehicle a VIN number
		              vinToVehicles.put(vehicle.getVIN(), vehicle);
		              return vehicle;
		              //break; // only handle the first spawn vehicle
		                   // TODO: need to fix this
		            }
		          }
		        } // else ignore the spawnSpecs and do nothing
		      }
		    }
		return null;
	}
	private boolean canSpawnVehicle(SpawnPoint spawnPoint) {
	    Rectangle2D noVehicleZone = spawnPoint.getNoVehicleZone();
	    for(VehicleSimView vehicle : vinToVehicles.values()) {
	      if (vehicle.getShape().intersects(noVehicleZone)) {
	        return false;
	      }
	    }
	    return true;
	  }
	private ProxyVehicle makeVehicle(SpawnPoint spawnPoint,
            SpawnSpec spawnSpec) {
		VehicleSpec spec = spawnSpec.getVehicleSpec();
		Lane lane = spawnPoint.getLane();
		// Now just take the minimum of the max velocity of the vehicle, and
		// the speed limit in the lane
		double initVelocity = Math.min(spec.getMaxVelocity(), lane.getSpeedLimit());
		// Obtain a Vehicle
		ProxyVehicle vehicle =
				new ProxyVehicle(spawnPoint.getPosition(),
						spawnPoint.getHeading(),
						spawnPoint.getSteeringAngle(),
						initVelocity, // velocity
						initVelocity,  // target velocity
						spawnPoint.getAcceleration(),
						spawnSpec.getSpawnTime());
		// Set the driver
		ProxyDriver driver = new ProxyDriver(vehicle, sim.getMap());
		driver.setCurrentLane(lane);
		driver.setSpawnPoint(spawnPoint);
		driver.setDestination(spawnSpec.getDestinationRoad());
		vehicle.setDriver(driver);
		this.playerVehicles.add(vehicle);
		return vehicle;
	}
	
	public void proxyTest(double timeStep) {
		for(ProxyVehicle player : playerVehicles){
			//player.setMovement(pos, heading, velocity, steeringAngle, acceleration, targetVelocity);
			Real2ProxyPVUpdate msg;
			double x = 160;//player.gaugePosition().getX();
			double y = 40;//player.gaugePosition().getY();
			double velocity = 0;
			double targetVelocity = 0;
			double acceleration = 0;
			double heading = Math.PI * 0.5;
			double steeringAngle = 0;
			//player.setMovement(new Point2D.Double(15.5,15.5), heading, velocity, steeringAngle, acceleration, targetVelocity);
			msg = new Real2ProxyPVUpdate(player.getVIN(), x + velocity*timeStep*sim.getSimulationTime(), y, heading, steeringAngle, velocity, targetVelocity, acceleration, sim.getSimulationTime());
			player.processReal2ProxyMsg(msg);
			player.getDriver().setCurrentLane(getClosestLane(player));
			player.move(timeStep);
			System.err.println("Player Vehicle Position: " + player.getPosition().getX() + "/"+ player.getPosition().getY());
		}
	}
	/**
	 * returns the closest lane to a given BasicAutoVehicle object
	 * @param player
	 * @return
	 */
	public Lane getClosestLane(BasicAutoVehicle player){
		
		Point2D[] positions = player.getCornerPoints();
		Lane laneReturn = player.getDriver().getCurrentLane();
		Lane tempLane = laneReturn;
		for(int i = 0; sim.getMap().getLaneRegistry().isIdExist(i); i++){
			tempLane = sim.getMap().getLaneRegistry().get(i);
			for(int j = 0; j < 4; j++){
				if(tempLane.contains(positions[j])){
					//return tempLane;
				}
				if(player.getShape().intersects(tempLane.getShape().getBounds2D())){
					return tempLane;
				}
				//note, need to add ability to return multiple lanes
			}
		}
		return laneReturn;
	}
	/**
	 * Sets the lanes occupied by the given basic vehicle for the purposes of collision detection outside of an intersection's area of control.
	 * Primarily used for manual mode player vehicles
	 * @param player
	 */
	public void setOccupiedLanes(BasicVehicle player){
		Lane tempLane = null;
		boolean firstRun = true;
		
		//search through all lanes
		for(int i = 0; sim.getMap().getLaneRegistry().isIdExist(i); i++){
			tempLane = sim.getMap().getLaneRegistry().get(i);
			if(player.getShape().intersects(tempLane.getShape().getBounds2D())){
				//if first lane registered, create new lane set, else just add
				if(firstRun){
					player.getDriver().setCurrentLane(tempLane);
					firstRun = false;
				} else {
					player.getDriver().addCurrentlyOccupiedLane(tempLane);
				}
			}
		}
	}
	public ODPair getPath(){
		return this.path;
	}
	
	private double rollOverBearing(double bearing){
		bearing = Math.floor(bearing*1000000)/1000000;
		while(bearing < 0){
			bearing = bearing + 2 * Math.PI;
		}
		while(bearing > 2*Math.PI){
			bearing = bearing - 2 * Math.PI;
		}
		return bearing;
	}
	/*
	 * 
	 * 
	 * 
	 * to test
	 */
	private VehicleSimView generatePlayerVehicle(Simulator sim, ODPair path, double bid) {
		boolean sucessful = false;
		ArrayList<SpawnPoint> candidateSpawns = new ArrayList<SpawnPoint>();
		ArrayList<Road> candidateDestinations = new ArrayList<Road>();
		SpawnPoint initSpawnPoint = null;
		Road destRoad;
		SpawnSpec spawnSpec;
		
		VehicleSimView vehicle = null;
		
		double startBearing = 0;
		double endBearing = 0;
		//set bearings, hard coded
		if(path.equals(ODPair.NORTH_EAST)||path.equals(ODPair.NORTH_WEST)||path.equals(ODPair.NORTH_SOUTH)){
			startBearing = Math.PI;
		} else if(path.equals(ODPair.SOUTH_EAST)||path.equals(ODPair.SOUTH_WEST)||path.equals(ODPair.SOUTH_NORTH)){
			startBearing = 0;
		} else if(path.equals(ODPair.EAST_WEST)||path.equals(ODPair.EAST_SOUTH)||path.equals(ODPair.EAST_NORTH)){
			startBearing = Math.PI / 2;
		} else if(path.equals(ODPair.WEST_EAST)||path.equals(ODPair.WEST_SOUTH)||path.equals(ODPair.WEST_NORTH)){
			startBearing = (3 * Math.PI ) / 2;
		}
		
		if(path.equals(ODPair.SOUTH_NORTH)||path.equals(ODPair.EAST_NORTH)||path.equals(ODPair.WEST_NORTH)){
			endBearing = 0;
		} else if(path.equals(ODPair.NORTH_SOUTH)||path.equals(ODPair.EAST_SOUTH)||path.equals(ODPair.WEST_SOUTH)){
			endBearing = Math.PI;
		} else if(path.equals(ODPair.EAST_WEST)||path.equals(ODPair.SOUTH_WEST)||path.equals(ODPair.NORTH_WEST)){
			endBearing = Math.PI / 2;
		} else if(path.equals(ODPair.NORTH_EAST)||path.equals(ODPair.SOUTH_EAST)||path.equals(ODPair.WEST_EAST)){
			endBearing = (3 * Math.PI ) / 2;
		}
		
		for(SpawnPoint spawnPoint : sim.getMap().getSpawnPoints()){
			if(spawnPoint.getHeading() == startBearing){
				candidateSpawns.add(spawnPoint);
			}
		}
		for(Road road :sim.getMap().getDestinationRoads()){
			if(road.getIndexLane().getInitialHeading() == endBearing){
				candidateDestinations.add(road);
			}
		}
		destRoad = candidateDestinations.get(0);
		//return null if no results
		if(candidateSpawns.isEmpty() || candidateDestinations.isEmpty()){
			return null;
		}
		//decide if lane will cross traffic paths
		for(SpawnPoint spawnPoint: candidateSpawns){
			if(willNotCrossLanes(spawnPoint.getLane(),destRoad)){
				initSpawnPoint = spawnPoint;
				break;
			}
		}
		//check if spawnPoint is valid
		if(initSpawnPoint == null){
			return null;
		}
		//create spawn spec
		spawnSpec = new SpawnSpec(sim.getSimulationTime(), VehicleSpecDatabase.getVehicleSpecByName("SEDAN"), destRoad);
		this.playerSpawnTime = sim.getSimulationTime();
		if(this.sim.canSpawnVehicle(initSpawnPoint)){
			vehicle = this.sim.makeVehicle(initSpawnPoint, spawnSpec);
			if(vehicle != null){
				vehicle.getDriver().getWallet().setBid(bid, false);
				VinRegistry.registerVehicle(vehicle); // Get vehicle a VIN number
				vinToVehicles.put(vehicle.getVIN(), vehicle);
			} else {
				return null;
			}
			Debug.setVehicleColor(vehicle.getVIN(), new Color(204,0,204));
			if(destRoad.getDual().getLanes().contains(initSpawnPoint.getLane())){
				return null;
			}
			System.out.println("vehicle vin = " + vehicle.getVIN());
			//Successfully generated the player vehicle
			setPlayerVehicle(vehicle);
		} else {
			this.initialised = false;
		}
		return vehicle;
	}
		
	private boolean willNotCrossLanes(Lane currentLane, Road dest) {
		//allow straight paths
		  if(pathIsStraight(currentLane.getInitialHeading(), dest.getIndexLane().getInitialHeading())){
			  return true;
		  }
		  //sim is left hand drive
		  //if there is a left lane, exclude right hand turns
		  //else exclude left hand turns
		  if(currentLane.hasLeftNeighbor()){
			  if(isLeftHandTurn(currentLane.getInitialHeading(), dest.getIndexLane().getInitialHeading())){
				  return false;
			  } else {
				  return true;
			  }
		  } else {
			  if(isLeftHandTurn(currentLane.getInitialHeading(), dest.getIndexLane().getInitialHeading())){
				  return true;
			  }else{
				  return false;
			  }
		  }
	}
	//for shapes of misc size use java.awt.polygon
	/**
	   * @author Alexander Humphry
	   * 
	   * returns true if a left hand turn is required to travel from the initial heading to the final heading
	   * @param initHeading
	   * @param finalHeading
	   * @return
	   */
	private boolean isLeftHandTurn(double initHeading, double finalHeading) {
		if(initHeading < Math.PI){
			  //if heading is in first 180 degrees
			  if(initHeading < finalHeading && finalHeading < initHeading + Math.PI){
				  return false;
			  }
			  return true;
		  } else {
			  if(initHeading - Math.PI < finalHeading && finalHeading < initHeading){
				  return true;
			  }
			  return false;
		  }
	}
	/**
	   * @author Alexander Humphry
	   * 
	   * returns true if the path requires a turn of no more than PI * 0.1 radians
	   * @param initHeading
	   * @param finalHeading
	   * @return
	   */
	private boolean pathIsStraight(double initHeading, double finalHeading) {
		if(Math.abs(initHeading - finalHeading) < Math.PI * 0.1){
			  return true;
		}
		if(Math.abs(initHeading + 2*Math.PI - finalHeading) < Math.PI * 0.1 || 
				  Math.abs(initHeading - 2*Math.PI - finalHeading) < Math.PI * 0.1){
			  return true;
		}
		return false;
	}
}
