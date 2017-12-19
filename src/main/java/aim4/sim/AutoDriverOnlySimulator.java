/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package aim4.sim;

import java.awt.Color;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Random;

import aim4.UDP.SimulatorSerializer;
import aim4.config.Debug;
import aim4.config.DebugPoint;
import aim4.conflictPoint.ConflictPointGeneratorSimple;
import aim4.driver.AutoDriver;
import aim4.driver.DriverSimView;
import aim4.driver.ProxyDriver;
import aim4.driver.Wallet;
import aim4.im.IntersectionManager;
import aim4.im.v2i.V2IManager;
import aim4.map.DataCollectionLine;
import aim4.map.BasicMap;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.lane.Lane;
import aim4.msg.i2v.I2VMessage;
import aim4.msg.v2i.V2IMessage;
import aim4.vehicle.AutoVehicleSimView;
import aim4.vehicle.BasicAutoVehicle;
import aim4.vehicle.HumanDrivenVehicleSimView;
import aim4.vehicle.ProxyVehicleSimView;
import aim4.vehicle.VehicleSpec;
import aim4.vehicle.VinRegistry;
import aim4.vehicle.VehicleSimView;


/**
 * The autonomous drivers only simulator.
 */
public class AutoDriverOnlySimulator implements Simulator {

  /////////////////////////////////
  // NESTED CLASSES
  /////////////////////////////////

  /**
   * The result of a simulation step.
   */
  public static class AutoDriverOnlySimStepResult implements SimStepResult {

    /** The VIN of the completed vehicles in this time step */
    List<Integer> completedVINs;

    /**
     * Create a result of a simulation step
     *
     * @param completedVINs  the VINs of completed vehicles.
     */
    public AutoDriverOnlySimStepResult(List<Integer> completedVINs) {
      this.completedVINs = completedVINs;
    }

    /**
     * Get the list of VINs of completed vehicles.
     *
     * @return the list of VINs of completed vehicles.
     */
    public List<Integer> getCompletedVINs() {
      return completedVINs;
    }
  }

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////

  /** The map */
  private BasicMap basicMap;
  /** All active vehicles, in form of a map from VINs to vehicle objects. */
  private Map<Integer,VehicleSimView> vinToVehicles;
  /** The current time */
  private double currentTime;
  /** The number of completed vehicles */
  private int numOfCompletedVehicles;
  /** The total number of bits transmitted by the completed vehicles */
  private int totalBitsTransmittedByCompletedVehicles;
  /** The total number of bits received by the completed vehicles */
  private int totalBitsReceivedByCompletedVehicles;
  /** Random Number Generator*/
  private Random rand;
  /** the communications module - SimCreator*/
  private ArrayList<SimulatorSerializer> simCreatorConnections;
  
  private SimulatorSerializer simSerializer;
  
  private boolean hasRun;
  
  private int releaseTestVehicles = 50;//in seconds
  /////////////////////////////////
  // CLASS CONSTRUCTORS
  /////////////////////////////////

  /**
   * Create an instance of the simulator.
   *
   * @param basicMap             the map of the simulation
   */
  public AutoDriverOnlySimulator(BasicMap basicMap) {
    this.basicMap = basicMap;
    this.vinToVehicles = new HashMap<Integer,VehicleSimView>();

    currentTime = 0.0;
    numOfCompletedVehicles = 0;
    totalBitsTransmittedByCompletedVehicles = 0;
    totalBitsReceivedByCompletedVehicles = 0;
    
    rand = new Random();
    
    //send UDP Alex Humphry
    //simSerialiser = new SimulatorSerialiser(this, 2500, "192.168.62.131");//linux machine
    
    //simSerializer = new SimulatorSerializer(this, 2500, "192.168.0.1");//direct sim connect
    
    //simSerializer = new SimulatorSerializer(this, 2501, "192.168.0.2");//indirect sim connect
    ////simSerializer = new SimulatorSerializer(this, 2501, "127.0.0.1");//local loopback sim connect
    //simSerialiser = new SimulatorSerialiser(this, 2500, "192.168.1.135");
    //pass reference of vinToVehicles list to serialiser for proxy vehicle addition
    //simSerialiser.setVinToVehicles(timeStep,vinToVehicles);
    
    
    simSerializer = new SimulatorSerializer(this,2502, "192.168.1.143");
    
    simCreatorConnections = new ArrayList<SimulatorSerializer>();
    
    //add simulator connections
    //simCreatorConnections.add(new SimulatorSerializer(this,2502, "192.168.0.2",SimulatorSerializer.ODPair.NORTH_SOUTH)); //sim1 - maintenance
    /*simCreatorConnections.add(new SimulatorSerializer(this,2500, "192.168.0.3",SimulatorSerializer.ODPair.EAST_WEST)); //sim2
    simCreatorConnections.add(new SimulatorSerializer(this,2504, "192.168.0.4",SimulatorSerializer.ODPair.SOUTH_NORTH)); //sim3
    simCreatorConnections.add(new SimulatorSerializer(this,2506, "192.168.0.5",SimulatorSerializer.ODPair.WEST_NORTH)); //sim4
    simCreatorConnections.add(new SimulatorSerializer(this,2508, "192.168.0.6",SimulatorSerializer.ODPair.NORTH_EAST)); //sim5*/
    double average = 0.28;
    ArrayList<Double> bidSet = new ArrayList<Double>();
    bidSet.add(average * 1);//test2 Sim2
    bidSet.add(average * 1);//test2 Sim3
    bidSet.add(average * 1);//test2 Sim4
    bidSet.add(average * 1);//test2 Sim5
    
    bidSet.add(average * 1);//test3 Sim2
    bidSet.add(average * 1);//test3 Sim3
    bidSet.add(average * 1);//test3 Sim4
    bidSet.add(average * 1);//test3 Sim5
    
    testProgramSelect(2, bidSet);
    hasRun = false;
    
    //conflict point test
	ConflictPointGeneratorSimple test = new ConflictPointGeneratorSimple(this.basicMap);
	test.generateConflictPoints(this.basicMap.getIntersectionManagers().get(0).getIntersection(), true);
    
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // the main loop

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized AutoDriverOnlySimStepResult step(double timeStep) {
	/*for(SimulatorSerializer simConnect : simCreatorConnections){
		if(!simConnect.isInitialised() && this.getSimulationTime() > 50.0){
			simConnect.setVinToVehicles(timeStep, vinToVehicles, simConnect.getPath());
		}
	}*/
	if(!this.hasRun && !vinToVehicles.isEmpty()){
		//old code
		this.hasRun = simSerializer.setVinToVehicles(timeStep,vinToVehicles);
		this.hasRun = true;
	}
	
	for(SimulatorSerializer simConnect : simCreatorConnections){
		if(!simConnect.isInitialisedNoVehicles() && !vinToVehicles.isEmpty()) {
			simConnect.setVinToVehicles(timeStep, vinToVehicles);
			simConnect.setInitialisedNoVehicles(true);
		} else if(!simConnect.isInitialised() && this.getSimulationTime() > releaseTestVehicles) {
			simConnect.setVinToVehicles(timeStep, vinToVehicles, simConnect.getPath(), simConnect.getBid());
		}
	}
	//old code
	/*if(!simCreatorConnections.get(0).isInitialisedNoVehicles() && !vinToVehicles.isEmpty()) {
		simCreatorConnections.get(0).setVinToVehicles(timeStep, vinToVehicles);
		simCreatorConnections.get(0).setInitialisedNoVehicles(true);
	}*/
	/*if(!simSerializer.isInitialisedNoVehicles() && !vinToVehicles.isEmpty()) {
		simSerializer.setVinToVehicles(timeStep, vinToVehicles);
		simSerializer.setInitialisedNoVehicles(true);
	}*/

	
	
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("--------------------------------------\n");
      System.err.printf("------SIM:spawnVehicles---------------\n");
    }
    spawnVehicles(timeStep);
    //simSerialiser.recieve();
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:provideSensorInput---------------\n");
    }
    provideSensorInput();
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:letDriversAct---------------\n");
    }
    letDriversAct();
    //simSerializer.proxyTest(timeStep);
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:letIntersectionManagersAct--------------\n");
    }
    letIntersectionManagersAct(timeStep);
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:communication---------------\n");
    }
    communication();
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:moveVehicles---------------\n");
    }
    moveVehicles(timeStep);
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:cleanUpCompletedVehicles---------------\n");
    }
    List<Integer> completedVINs = cleanUpCompletedVehicles();
    currentTime += timeStep;
    // debug
    checkClocks();
    
    //Alex Humphry
    //send data after time step via UDP
    ////simSerializer.send(timeStep);
    //simCreatorConnections.get(0).send(timeStep);
    for(SimulatorSerializer simConnect : simCreatorConnections){
			simConnect.send(timeStep);
	}
    //not pointed in correct direction
    //System.out.println(simSerialiser.recieve().getText());
    
    return new AutoDriverOnlySimStepResult(completedVINs);
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // information retrieval

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized BasicMap getMap() {
    return basicMap;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getSimulationTime() {
    return currentTime;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized int getNumCompletedVehicles() {
    return numOfCompletedVehicles;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsTransmittedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsTransmittedByCompletedVehicles)
             / numOfCompletedVehicles;
    } else {
      return 0.0;
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsReceivedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsReceivedByCompletedVehicles)
             / numOfCompletedVehicles;
    } else {
      return 0.0;
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized Set<VehicleSimView> getActiveVehicles() {
    return new HashSet<VehicleSimView>(vinToVehicles.values());
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public VehicleSimView getActiveVehicle(int vin) {
    return vinToVehicles.get(vin);
  }


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized void addProxyVehicle(ProxyVehicleSimView vehicle) {
    Point2D pos = vehicle.getPosition();
    Lane minLane = null;
    double minDistance = -1.0;

    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        double d = lane.nearestDistance(pos);
        if (minLane == null || d < minDistance) {
          minLane = lane;
          minDistance = d;
        }
      }
    }
    assert minLane != null;

    ProxyDriver driver = vehicle.getDriver();
    if (driver != null) {
      driver.setCurrentLane(minLane);
      driver.setSpawnPoint(null);
      driver.setDestination(null);
    }

    vinToVehicles.put(vehicle.getVIN(), vehicle);
  }


  /////////////////////////////////
  // PRIVATE METHODS
  /////////////////////////////////

  /////////////////////////////////
  // STEP 1
  /////////////////////////////////

  /**
   * Spawn vehicles.
   *
   * @param timeStep  the time step
   */
  private void spawnVehicles(double timeStep) {
	  int vanNumber = 0;
	  SpawnPoint tempSpawnPoint = null;
    for(SpawnPoint spawnPoint : basicMap.getSpawnPoints()) {
      List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
      if (!spawnSpecs.isEmpty()) {
        if (canSpawnVehicle(spawnPoint)) {          
	        for(SpawnSpec spawnSpec : spawnSpecs) {
	        	tempSpawnPoint = congestionBalance(spawnPoint, spawnSpec);
	        	if(canSpawnVehicle(tempSpawnPoint)){
		        	VehicleSimView vehicle = makeVehicle(tempSpawnPoint, spawnSpec);
		        	VinRegistry.registerVehicle(vehicle); // Get vehicle a VIN number
		        	vinToVehicles.put(vehicle.getVIN(), vehicle);
		        	
	        	}
	        	break; // only handle the first spawn vehicle
		               // TODO: need to fix this
	        }
        } // else ignore the spawnSpecs and do nothing
      }
    }
  }
  /*
   * backup of spawnVehicles
   * 
   * private void spawnVehicles(double timeStep) {
    for(SpawnPoint spawnPoint : basicMap.getSpawnPoints()) {
      List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
      if (!spawnSpecs.isEmpty()) {
        if (canSpawnVehicle(spawnPoint)) {
          //alex aim code next if statement
          if(this.vinToVehicles.size() < (52 - spawnSpecs.size())){
            for(SpawnSpec spawnSpec : spawnSpecs) {
              VehicleSimView vehicle = makeVehicle(spawnPoint, spawnSpec);
              VinRegistry.registerVehicle(vehicle); // Get vehicle a VIN number
              vinToVehicles.put(vehicle.getVIN(), vehicle);
              break; // only handle the first spawn vehicle
                   // TODO: need to fix this
            }
          }
        } // else ignore the spawnSpecs and do nothing
      }
    }
  }
   */




/**
   * Whether a spawn point can spawn any vehicle
   *
   * @param spawnPoint  the spawn point
   * @return Whether the spawn point can spawn any vehicle
   */
  public boolean canSpawnVehicle(SpawnPoint spawnPoint) {
    // TODO: can be made much faster.
    Rectangle2D noVehicleZone = spawnPoint.getNoVehicleZone();
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      if (vehicle.getShape().intersects(noVehicleZone)) {
        return false;
      }
      //disable spawns where destination involves turning left from right lane or right from left lane
      //if vehicle is turning
      try {
	      /*if(vehicle.getDriver().getCurrentLane().getInitialHeading() != vehicle.getDriver().getDestination().getIndexLane().getInitialHeading()){
	    	  //if vehicle is in left Lane
	    	  if(!vehicle.getDriver().getCurrentLane().hasLeftNeighbor()){
	    		  //if destination heading is within 180 degrees turning left
	    		  if(){
	    			  resolveHeadingChange(vehicle.getDriver().getCurrentLane().getInitialHeading(), vehicle.getDriver().getDestination().getIndexLane().getInitialHeading());
	    		  }//put in if statement
	    	  }
	    	  Lane init = vehicle.getDriver().getCurrentLane();
	    	  Lane dest = vehicle.getDriver().getDestination().getLanes().get(0);
	
	    	  return true;
	      }*/
      } catch(Exception e){
    	  //do something in case of exception
      }
    }
    return true;
  }

  /**
   * Create a vehicle at a spawn point.
   *
   * @param spawnPoint  the spawn point
   * @param spawnSpec   the spawn specification
   * @return the vehicle
   */
  public VehicleSimView makeVehicle(SpawnPoint spawnPoint,
                                     SpawnSpec spawnSpec) {
    VehicleSpec spec = spawnSpec.getVehicleSpec();
    Lane lane = spawnPoint.getLane();
    // Now just take the minimum of the max velocity of the vehicle, and
    // the speed limit in the lane
    double initVelocity = Math.min(spec.getMaxVelocity(), lane.getSpeedLimit());
    // Obtain a Vehicle
    AutoVehicleSimView vehicle =
      new BasicAutoVehicle(spec,
                           spawnPoint.getPosition(),
                           spawnPoint.getHeading(),
                           spawnPoint.getSteeringAngle(),
                           initVelocity, // velocity
                           initVelocity,  // target velocity
                           spawnPoint.getAcceleration(),
                           spawnSpec.getSpawnTime());
    // Set the driver
    AutoDriver driver = new AutoDriver(vehicle, basicMap);
    driver.setCurrentLane(lane);
    driver.setSpawnPoint(spawnPoint);
    driver.setDestination(spawnSpec.getDestinationRoad());
    driver.setWallet(new Wallet());
    vehicle.setDriver(driver);

    return vehicle;
  }


  /////////////////////////////////
  // STEP 2
  /////////////////////////////////

  /**
   * Compute the lists of vehicles of all lanes.
   *
   * @return a mapping from lanes to lists of vehicles sorted by their
   *         distance on their lanes
   */
  private Map<Lane,SortedMap<Double,VehicleSimView>> computeVehicleLists() {
    // Set up the structure that will hold all the Vehicles as they are
    // currently ordered in the Lanes
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists =
      new HashMap<Lane,SortedMap<Double,VehicleSimView>>();
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        vehicleLists.put(lane, new TreeMap<Double,VehicleSimView>());
      }
    }
    // Now add each of the Vehicles, but make sure to exclude those that are
    // already inside (partially or entirely) the intersection
    for(VehicleSimView vehicle : vinToVehicles.values()) {
    	/*if(simSerializer.getProxyVehicles().contains(vehicle)){
    		Point2D test = vehicle.gaugePosition();
    		test.getX();
    		test.getY();
    	}*/
      // Find out what lanes it is in.
      Set<Lane> lanes = vehicle.getDriver().getCurrentlyOccupiedLanes();
      for(Lane lane : lanes) {
        // Find out what IntersectionManager is coming up for this vehicle
        IntersectionManager im =
          lane.getLaneIM().nextIntersectionManager(vehicle.getPosition());
        // Only include this Vehicle if it is not in the intersection.
        if(lane.getLaneIM().distanceToNextIntersection(vehicle.getPosition())>0
            || im == null || !im.intersects(vehicle.getShape().getBounds2D())) {
          // Now find how far along the lane it is.
          double dst = lane.distanceAlongLane(vehicle.getPosition());
          // Now add it to the map.
          vehicleLists.get(lane).put(dst, vehicle);
        }
      }
    }
    // Now consolidate the lists based on lanes
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        // We may have already removed this Lane from the map
        if(vehicleLists.containsKey(lane)) {
          Lane currLane = lane;
          // Now run through the lanes
          while(currLane.hasNextLane()) {
            currLane = currLane.getNextLane();
            // Put everything from the next lane into the original lane
            // and remove the mapping for the next lane
            vehicleLists.get(lane).putAll(vehicleLists.remove(currLane));
          }
        }
      }
    }

    return vehicleLists;
  }
  /**
   * determines whether congestion on a particular road unnecessarily slows down straight traffic
   * If so, outputs the spawn point of a neighboring lane in the road experiencing the least congestion
   * @param spawnPoint
   * @param spawnSpecs
   * @return
   */
  private SpawnPoint congestionBalance(SpawnPoint spawnPoint,
		SpawnSpec spawnSpec) {
	  if(!pathIsStraight(spawnPoint.getHeading(),spawnSpec.getDestinationRoad().getIndexLane().getInitialHeading())){
		  return spawnPoint;
	  }
	  
	  int deltaL = 0;
	  int deltaR = 0;
	  
	  int candHurdle = 4;
	  
	  boolean right = false;
	  boolean left = false;
	  
	  //get candidate Lanes
	  Lane currLane = spawnPoint.getLane();
	  Lane laneLeft = spawnPoint.getLane().getLeftNeighbor();
	  Lane laneRight = spawnPoint.getLane().getRightNeighbor();
	  
	  //store lane fill
	  if(laneLeft != null){
		  left = true;
	  }
	  if(laneRight != null){
		  right = true;
	  }
	  
	  //Initialize vehicle arrays
	  ArrayList<Integer> vinsCurrLane = new ArrayList<Integer>();
	  ArrayList<Integer> vinsLeftLane = new ArrayList<Integer>();
	  ArrayList<Integer> vinsRightLane = new ArrayList<Integer>();
	  
	  //if lanes not null, determine traffic load
	  for(VehicleSimView vehicle : vinToVehicles.values()){
		  if(vehicle.getDriver().getCurrentLane() == currLane){
			  vinsCurrLane.add(vehicle.getVIN());
		  } else if(left && vehicle.getDriver().getCurrentLane() == laneLeft){
			  vinsLeftLane.add(vehicle.getVIN());
		  } else if(right && vehicle.getDriver().getCurrentLane() == laneRight){
			  vinsRightLane.add(vehicle.getVIN());
		  }
	  }
	  //count lane deltas
	  //higher numbers indicate congestion in center lane relative to each candidate lane
	  if(laneLeft != null){
		  deltaL = vinsCurrLane.size() - vinsLeftLane.size();
	  }
	  if(laneRight != null){
		  deltaR = vinsCurrLane.size() - vinsRightLane.size();
	  }
	  
	  //reset left and right boolean values
	  right = false;
	  left = false;
	  
	  //determine if delta is high enough
	  if(deltaL > candHurdle){
		  left = true;
	  }
	  if(deltaR > candHurdle){
		  right = true;
	  }
	  
	  //Prioritize less congested option
	  if(left && right){
		  if(deltaL > deltaR){
			  right = false;
		  } else {
			  left = false;
		  }
	  }
	  
	  //

	  if(left){
		  for(SpawnPoint sp : Debug.currentMap.getSpawnPoints()){
			  if(sp.getLane() == laneLeft){
				  return sp;
			  }
		  }
	  }
	  if(right){
		  for(SpawnPoint sp : Debug.currentMap.getSpawnPoints()){
			  if(sp.getLane() == laneRight){
				  return sp;
			  }
		  }
	  }
	  
	  
	return spawnPoint;
}
  
  private boolean pathIsStraight(double initHeading, double finalHeading){
	  if(Math.abs(initHeading - finalHeading) < Math.PI * 0.1){
		  return true;
	  }
	  if(Math.abs(initHeading + 2*Math.PI - finalHeading) < Math.PI * 0.1 || 
			  Math.abs(initHeading - 2*Math.PI - finalHeading) < Math.PI * 0.1){
		  return true;
	  }
	  return false;
  }

  /**
   * Compute the next vehicles of all vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   * @return a mapping from vehicles to next vehicles
   */
  private Map<VehicleSimView, VehicleSimView> computeNextVehicle(
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists) {
    // At this point we should only have mappings for start Lanes, and they
    // should include all the Lanes they run into.  Now we need to turn this
    // into a hash map that maps Vehicles to the next vehicle in the Lane
    // or any Lane the Lane runs into
    Map<VehicleSimView, VehicleSimView> nextVehicle =
      new HashMap<VehicleSimView,VehicleSimView>();
    // For each of the ordered lists of vehicles
    for(SortedMap<Double,VehicleSimView> vehicleList : vehicleLists.values()) {
      VehicleSimView lastVehicle = null;
      // Go through the Vehicles in order of their position in the Lane
      for(VehicleSimView currVehicle : vehicleList.values()) {
        if(lastVehicle != null) {
          // Create the mapping from the previous Vehicle to the current one
          nextVehicle.put(lastVehicle,currVehicle);
        }
        lastVehicle = currVehicle;
      }
    }

    return nextVehicle;
  }

  /**
   * Provide each vehicle with sensor information to allow it to make
   * decisions.  This works first by making an ordered list for each Lane of
   * all the vehicles in that Lane, in order from the start of the Lane to
   * the end of the Lane.  We must make sure to leave out all vehicles that
   * are in the intersection.  We must also concatenate the lists for lanes
   * that feed into one another.  Then, for each vehicle, depending on the
   * state of its sensors, we provide it with the appropriate sensor input.
   */
  private void provideSensorInput() {
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists =
      computeVehicleLists();
    Map<VehicleSimView, VehicleSimView> nextVehicle =
      computeNextVehicle(vehicleLists);

    provideIntervalInfo(nextVehicle);
    provideVehicleTrackingInfo(vehicleLists);
    provideTrafficSignal();
  }

  /**
   * Provide sensing information to the intervalometers of all vehicles.
   *
   * @param nextVehicle  a mapping from vehicles to next vehicles
   */
  private void provideIntervalInfo(
    Map<VehicleSimView, VehicleSimView> nextVehicle) {

    // Now that we have this list set up, let's provide input to all the
    // Vehicles.
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;
        
        switch(autoVehicle.getLRFMode()) {
        case DISABLED:
          // Find the interval to the next vehicle
          double interval;
          // If there is a next vehicle, then calculate it
          if(nextVehicle.containsKey(autoVehicle)) {
            // It's the distance from the front of this Vehicle to the point
            // at the rear of the Vehicle in front of it
            interval = calcInterval(autoVehicle, nextVehicle.get(autoVehicle));
          } else { // Otherwise, just set it to the maximum possible value
            interval = Double.MAX_VALUE;
          }
          // Now actually record it in the vehicle
          autoVehicle.getIntervalometer().record(interval);
          autoVehicle.setLRFSensing(false); // Vehicle is not using
                                            // the LRF sensor
          break;
        case LIMITED:
          // FIXME
          autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
          break;
        case ENABLED:
          // FIXME
          autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
          break;
        default:
          throw new RuntimeException("Unknown LRF Mode: " +
                                     autoVehicle.getLRFMode().toString());
        }
      }
    }
  }

  /**
   * Provide tracking information to vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   */
  private void provideVehicleTrackingInfo(
    Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists) {
    // Vehicle Tracking
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;

        if (autoVehicle.isVehicleTracking()) {
          DriverSimView driver = autoVehicle.getDriver();
          Lane targetLane = autoVehicle.getTargetLaneForVehicleTracking();
          Point2D pos = autoVehicle.getPosition();
          double dst = targetLane.distanceAlongLane(pos);

          // initialize the distances to infinity
          double frontDst = Double.MAX_VALUE;
          double rearDst = Double.MAX_VALUE;
          VehicleSimView frontVehicle = null ;
          VehicleSimView rearVehicle = null ;

          // only consider the vehicles on the target lane
          SortedMap<Double,VehicleSimView> vehiclesOnTargetLane =
            vehicleLists.get(targetLane);

          // compute the distances and the corresponding vehicles
          try {
            double d = vehiclesOnTargetLane.tailMap(dst).firstKey();
            frontVehicle = vehiclesOnTargetLane.get(d);
            frontDst = (d-dst)-frontVehicle.getSpec().getLength();
          } catch(NoSuchElementException e) {
            frontDst = Double.MAX_VALUE;
            frontVehicle = null;
          }
          try {
            double d = vehiclesOnTargetLane.headMap(dst).lastKey();
            rearVehicle = vehiclesOnTargetLane.get(d);
            rearDst = dst-d;
          } catch(NoSuchElementException e) {
            rearDst = Double.MAX_VALUE;
            rearVehicle = null;
          }

          // assign the sensor readings

          autoVehicle.getFrontVehicleDistanceSensor().record(frontDst);
          autoVehicle.getRearVehicleDistanceSensor().record(rearDst);

          // assign the vehicles' velocities

          if(frontVehicle!=null) {
            autoVehicle.getFrontVehicleSpeedSensor().record(
                frontVehicle.getVelocity());
          } else {
            autoVehicle.getFrontVehicleSpeedSensor().record(Double.MAX_VALUE);
          }
          if(rearVehicle!=null) {
            autoVehicle.getRearVehicleSpeedSensor().record(
                rearVehicle.getVelocity());
          } else {
            autoVehicle.getRearVehicleSpeedSensor().record(Double.MAX_VALUE);
          }

          // show the section on the viewer
          if (Debug.isTargetVIN(driver.getVehicle().getVIN())) {
            Point2D p1 = targetLane.getPointAtNormalizedDistance(
                Math.max((dst-rearDst)/targetLane.getLength(),0.0));
            Point2D p2 = targetLane.getPointAtNormalizedDistance(
                Math.min((frontDst+dst)/targetLane.getLength(),1.0));
            Debug.addLongTermDebugPoint(
              new DebugPoint(p2, p1, "cl", Color.RED.brighter()));
          }
        }
      }
    }

  }

  /**
   * Provide traffic signals.
   */
  private void provideTrafficSignal() {
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      if (vehicle instanceof HumanDrivenVehicleSimView) {
        HumanDrivenVehicleSimView manualVehicle =
          (HumanDrivenVehicleSimView)vehicle;
        provideTrafficLightSignal(manualVehicle);
      }
    }
  }

  /**
   * Calculate the distance between vehicle and the next vehicle on a lane.
   *
   * @param vehicle      the vehicle
   * @param nextVehicle  the next vehicle
   * @return the distance between vehicle and the next vehicle on a lane
   */
  private double calcInterval(VehicleSimView vehicle,
                              VehicleSimView nextVehicle) {
    // From Chiu: Kurt, if you think this function is not okay, probably
    // we should talk to see what to do.
    Point2D pos = vehicle.getPosition();
    if(nextVehicle.getShape().contains(pos)) {
      return 0.0;
    } else {
      // TODO: make it more efficient
      double interval = Double.MAX_VALUE ;
      for(Line2D edge : nextVehicle.getEdges()) {
        double dst = edge.ptSegDist(pos);
        if(dst < interval){
          interval = dst;
        }
      }
      return interval;
    }
  }
  // Kurt's code:
  // interval = vehicle.getPosition().
  //   distance(nextVehicle.get(vehicle).getPointAtRear());


  /**
   * Provide traffic light signals to a vehicle.
   *
   * @param vehicle  the vehicle
   */
  private void provideTrafficLightSignal(HumanDrivenVehicleSimView vehicle) {
    // TODO: implement it later
//    DriverSimView driver = vehicle.getDriver();
//    Lane currentLane = driver.getCurrentLane();
//    Point2D pos = vehicle.getPosition();
//    IntersectionManager im = currentLane.getLaneIM().
//                             nextIntersectionManager(pos);
//    if (im != null) {
//      if (im instanceof LightOnlyManager) {
//        LightOnlyManager lightOnlyIM = (LightOnlyManager)im;
//        if (!im.getIntersection().getArea().contains(pos)) {
//          LightState s = lightOnlyIM.getLightState(currentLane);
//          vehicle.setLightState(s);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(s);
//          }
//        } else {
//          vehicle.setLightState(null);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(null);
//          }
//        }
//      }
//    } else {
//      vehicle.setLightState(null);
//      if (driver instanceof HumanDriver) {
//        ((HumanDriver)driver).setLightState(null);
//      }
//    }
  }

  /////////////////////////////////
  // STEP 3
  /////////////////////////////////

  /**
   * Allow each driver to act.
   */
  private void letDriversAct() {
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      vehicle.getDriver().act();
    }
  }

  /////////////////////////////////
  // STEP 4
  /////////////////////////////////

  /**
   * Allow each intersection manager to act.
   *
   * @param timeStep  the time step
   */
  private void letIntersectionManagersAct(double timeStep) {
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
    	/*if(im instanceof V2IManager ){//&& currentTime >= 3.2 && currentTime <= 3.4){
    		((V2IManager) im).resetReservationGrid(10);
    	}*/
      im.act(timeStep);
    }
  }

  /////////////////////////////////
  // STEP 5
  /////////////////////////////////

  /**
   * Deliver the V2I and I2V messages.
   */
  private void communication() {
    deliverV2IMessages();
    deliverI2VMessages();
//    deliverV2VMessages();
  }

  /**
   * Deliver the V2I messages.
   */
  private void deliverV2IMessages() {
    // Go through each vehicle and deliver each of its messages
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      // Start with V2I messages
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
        Queue<V2IMessage> v2iOutbox = sender.getV2IOutbox();
        while(!v2iOutbox.isEmpty()) {
          V2IMessage msg = v2iOutbox.poll();
          V2IManager receiver =
            (V2IManager)basicMap.getImRegistry().get(msg.getImId());
          // Calculate the distance the message must travel
          double txDistance =
            sender.getPosition().distance(
                receiver.getIntersection().getCentroid());
          // Find out if the message will make it that far
          if(transmit(txDistance, sender.getTransmissionPower())) {
            // Actually deliver the message
            receiver.receive(msg);
            // Add the delivery to the debugging information
          }
          // Either way, we increment the number of transmitted messages
        }
      }
    }
  }

  /**
   * Deliver the I2V messages.
   */
  private void deliverI2VMessages() {
    // Now deliver all the I2V messages
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      V2IManager senderIM = (V2IManager)im;
      for(Iterator<I2VMessage> i2vIter = senderIM.outboxIterator();
          i2vIter.hasNext();) {
        I2VMessage msg = i2vIter.next();
        AutoVehicleSimView vehicle =
          (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
            msg.getVin());
        // Calculate the distance the message must travel
        double txDistance =
          senderIM.getIntersection().getCentroid().distance(
            vehicle.getPosition());
        // Find out if the message will make it that far
        if(transmit(txDistance, senderIM.getTransmissionPower())) {
          // Actually deliver the message
          vehicle.receive(msg);
        }
      }
      // Done delivering the IntersectionManager's messages, so clear the
      // outbox.
      senderIM.clearOutbox();
    }
  }

//  private void deliverV2VMessages() {
//
//    // Create a place to store broadcast messages until they can be sent so
//    // that we don't have to run through the list of Vehicles for each one
//    List<V2VMessage> broadcastMessages = new ArrayList<V2VMessage>();
//
//    // Go through each vehicle and deliver each of its messages
//    for(VehicleSimView vehicle : vinToVehicles.values()) {
//      // Then do V2V messages.
//      if (vehicle instanceof AutoVehicleSimView) {
//        AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
//        for(V2VMessage msg : sender.getV2VOutbox()) {
//          if(msg.isBroadcast()) {
//            // If it's a broadcast message, we save it and worry about it later
//            broadcastMessages.add(msg);
//          } else {
//            // Otherwise, we just deliver it! Woo!
//            // TODO: need to check whether the vehicle is AutoVehicleSpec
//            AutoVehicleSimView receiver =
//              (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
//                msg.getDestinationID());
//            // Calculate the distance the message must travel
//            double txDistance =
//              sender.getPosition().distance(receiver.getPosition());
//            // Find out if the message will make it that far
//            if(transmit(txDistance, sender.getTransmissionPower())) {
//              // Actually deliver the message
//              receiver.receive(msg);
//              // Add the delivery to the debugging information
//            }
//          }
//        }
//        // Done delivering the V2V messages (except broadcast which we will
//        // handle in a moment), so clear the outbox
//        sender.getV2VOutbox().clear();
//      }
//    }
//    // Now go through the vehicles and deliver the broadcast messages
//    for(V2VMessage msg : broadcastMessages) {
//      // Send a copy to the IM for debugging/statistics purposes
//      IntersectionManager im =
//        basicMap.getImRegistry().get(msg.getIntersectionManagerID());
////      if(im != null) {
////        switch(im.getIntersectionType()) {
////        case V2V:
////          ((V2VManager) im).logBroadcast(msg);
////        }
////      }
//      // Determine who sent this message
//      // TODO: need to check whether the vehicle is AutoVehicleSpec
//      AutoVehicleSimView sender =
//        (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
//          msg.getSourceID());
//      // Deliver to each vehicle
//      for(VehicleSimView vehicle : vinToVehicles.values()) {
//        if (vehicle instanceof AutoVehicleSimView) {
//          AutoVehicleSimView receiver = (AutoVehicleSimView)vehicle;
//          // Except the one that sent it
//          if(sender != receiver) {
//            // Find out how far away they are
//            double txDistance =
//              sender.getPosition().distance(receiver.getPosition());
//            // See if this Vehicle is close enough to receive the message
//            if(transmit(txDistance, sender.getTransmissionPower())) {
//              // Actually deliver the message
//              receiver.receive(msg);
//            }
//          }
//        } // else ignore other non-autonomous vehicle
//      }
//    }
//  }


  /**
   * Whether the transmission of a message is successful
   *
   * @param distance  the distance of the transmission
   * @param power     the power of the transmission
   * @return whether the transmission of a messsage is successful
   */
  private boolean transmit(double distance, double power) {
    // Simple for now
    return distance <= power;
  }


  /////////////////////////////////
  // STEP 6
  /////////////////////////////////

  /**
   * Move all the vehicles.
   *
   * @param timeStep  the time step
   */
  private void moveVehicles(double timeStep) {
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      Point2D p1 = vehicle.getPosition();
      vehicle.move(timeStep);
      Point2D p2 = vehicle.getPosition();
      for(DataCollectionLine line : basicMap.getDataCollectionLines()) {
        line.intersect(vehicle, currentTime, p1, p2);
      }
      if (Debug.isPrintVehicleStateOfVIN(vehicle.getVIN())) {
        vehicle.printState();
      }
    }
  }


  /////////////////////////////////
  // STEP 7
  /////////////////////////////////

  /**
   * Remove all completed vehicles.
   *
   * @return the VINs of the completed vehicles
   */
  private List<Integer> cleanUpCompletedVehicles() {
    List<Integer> completedVINs = new LinkedList<Integer>();

    Rectangle2D mapBoundary = basicMap.getDimensions();

    List<Integer> removedVINs = new ArrayList<Integer>(vinToVehicles.size());
    for(int vin : vinToVehicles.keySet()) {
      VehicleSimView v = vinToVehicles.get(vin);
      // If the vehicle is no longer in the layout
      // TODO: this should be replaced with destination zone.
      if(!v.getShape().intersects(mapBoundary)) {
        // Process all the things we need to from this vehicle
        if (v instanceof AutoVehicleSimView) {
          AutoVehicleSimView v2 = (AutoVehicleSimView)v;
          totalBitsTransmittedByCompletedVehicles += v2.getBitsTransmitted();
          totalBitsReceivedByCompletedVehicles += v2.getBitsReceived();
        }
        removedVINs.add(vin);
      }
    }
    // Remove the marked vehicles
    for(int vin : removedVINs) {
      vinToVehicles.remove(vin);
      completedVINs.add(vin);
      numOfCompletedVehicles++;
    }

    return completedVINs;
  }

  /////////////////////////////////
  // DEBUG
  /////////////////////////////////

  /**
   * Check whether the clocks are in sync.
   */
  private void checkClocks() {
    // Check the clocks for all autonomous vehicles.
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      vehicle.checkCurrentTime(currentTime);
    }
    // Check the clocks for all the intersection managers.
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      im.checkCurrentTime(currentTime);
    }
  }
  
  /**
   * determine if a heading is right of another
   */
  /*public boolean resolveHeadingChange(double initHeading, double finalHeading){
	  if(initHeading > Math.PI){
		  if(initHeading < finalHeading){
			  return true;
		  } else {
			  return false;
		  }
	  }
	  return false;
  }*/
  private double randomRange(double min, double max){
	  Double minStore = min*100;
	  Double maxStore = max*100;
	  int minInt = minStore.intValue();
	  int maxInt = maxStore.intValue();
	  int resultToConvert = rand.nextInt((maxInt - minInt) + 1) + minInt;
	  return resultToConvert/100.0;
  }
  /**
   * a function which initializes the simulator connections with given vehicle routes
   * @return a list of initialized simulatorSerializer objects
   */
  private void testProgramSelect(int testScheduleID){
	  if(testScheduleID == 1){
		  //Straight, straight, right, left
		  simCreatorConnections.add(new SimulatorSerializer(this,2500, "192.168.0.3",SimulatorSerializer.ODPair.EAST_WEST, randomRange(0.05, 0.5),"sim2")); //sim2
		  simCreatorConnections.add(new SimulatorSerializer(this,2504, "192.168.0.4",SimulatorSerializer.ODPair.SOUTH_NORTH, randomRange(0.05, 0.5),"sim3")); //sim3
		  simCreatorConnections.add(new SimulatorSerializer(this,2506, "192.168.0.5",SimulatorSerializer.ODPair.WEST_NORTH, randomRange(0.05, 0.5),"sim4")); //sim4
		  simCreatorConnections.add(new SimulatorSerializer(this,2508, "192.168.0.6",SimulatorSerializer.ODPair.NORTH_EAST, randomRange(0.05, 0.5),"sim5")); //sim5
	  } else if(testScheduleID == 2){
		  //Straight, right, left, straight
		  simCreatorConnections.add(new SimulatorSerializer(this,2500, "192.168.0.3",SimulatorSerializer.ODPair.WEST_EAST, randomRange(0.05, 0.5),"sim2")); //sim2
		  simCreatorConnections.add(new SimulatorSerializer(this,2504, "192.168.0.4",SimulatorSerializer.ODPair.SOUTH_EAST, randomRange(0.05, 0.5),"sim3")); //sim3
		  simCreatorConnections.add(new SimulatorSerializer(this,2506, "192.168.0.5",SimulatorSerializer.ODPair.WEST_NORTH, randomRange(0.05, 0.5),"sim4")); //sim4
		  simCreatorConnections.add(new SimulatorSerializer(this,2508, "192.168.0.6",SimulatorSerializer.ODPair.NORTH_SOUTH, randomRange(0.05, 0.5),"sim5")); //sim5
	  } else {
		  //right, left, straight, straight
		  simCreatorConnections.add(new SimulatorSerializer(this,2500, "192.168.0.3",SimulatorSerializer.ODPair.EAST_NORTH, randomRange(0.05, 0.5),"sim2")); //sim2
		  simCreatorConnections.add(new SimulatorSerializer(this,2504, "192.168.0.4",SimulatorSerializer.ODPair.SOUTH_WEST, randomRange(0.05, 0.5),"sim3")); //sim3
		  simCreatorConnections.add(new SimulatorSerializer(this,2506, "192.168.0.5",SimulatorSerializer.ODPair.WEST_EAST, randomRange(0.05, 0.5),"sim4")); //sim4
		  simCreatorConnections.add(new SimulatorSerializer(this,2508, "192.168.0.6",SimulatorSerializer.ODPair.NORTH_SOUTH, randomRange(0.05, 0.5),"sim5")); //sim5  
	  }
	    
  }
  private void testProgramSelect(int testScheduleID, List<Double> bids){
	  if(bids == null){
		  bids = new ArrayList<Double>();
	  }
	  if(bids.isEmpty() || bids.size() < 8){
		  for(int i = bids.size(); i < 8; i++){
			  bids.add(randomRange(0.05, 0.5));
		  }
	  }
	  if(testScheduleID == 1){
		  //Straight, straight, right, left
		  simCreatorConnections.add(new SimulatorSerializer(this,2500, "192.168.0.3",SimulatorSerializer.ODPair.EAST_WEST, randomRange(0.05, 0.5),"sim2")); //sim2
		  simCreatorConnections.add(new SimulatorSerializer(this,2504, "192.168.0.4",SimulatorSerializer.ODPair.SOUTH_NORTH, randomRange(0.05, 0.5),"sim3")); //sim3
		  simCreatorConnections.add(new SimulatorSerializer(this,2506, "192.168.0.5",SimulatorSerializer.ODPair.WEST_NORTH, randomRange(0.05, 0.5),"sim4")); //sim4
		  //simCreatorConnections.add(new SimulatorSerializer(this,2508, "192.168.0.6",SimulatorSerializer.ODPair.NORTH_EAST, randomRange(0.05, 0.5),"sim5")); //sim5
	  } else if(testScheduleID == 2){
		  //Straight, right, left, straight
		  simCreatorConnections.add(new SimulatorSerializer(this,2500, "192.168.0.3",SimulatorSerializer.ODPair.WEST_EAST, bids.get(0),"sim2")); //sim2
		  simCreatorConnections.add(new SimulatorSerializer(this,2504, "192.168.1.143",SimulatorSerializer.ODPair.SOUTH_EAST, bids.get(1),"sim3")); //sim3
		  simCreatorConnections.add(new SimulatorSerializer(this,2506, "192.168.0.5",SimulatorSerializer.ODPair.WEST_NORTH, bids.get(2),"sim4")); //sim4
		  //simCreatorConnections.add(new SimulatorSerializer(this,2508, "192.168.0.6",SimulatorSerializer.ODPair.NORTH_SOUTH, bids.get(3),"sim5")); //sim5
	  } else {
		  //right, left, straight, straight
		  simCreatorConnections.add(new SimulatorSerializer(this,2500, "192.168.0.3",SimulatorSerializer.ODPair.EAST_NORTH, bids.get(4),"sim2")); //sim2
		  simCreatorConnections.add(new SimulatorSerializer(this,2504, "192.168.0.4",SimulatorSerializer.ODPair.SOUTH_WEST, bids.get(5),"sim3")); //sim3
		  simCreatorConnections.add(new SimulatorSerializer(this,2506, "192.168.0.5",SimulatorSerializer.ODPair.WEST_EAST, bids.get(6),"sim4")); //sim4
		  //simCreatorConnections.add(new SimulatorSerializer(this,2508, "192.168.0.6",SimulatorSerializer.ODPair.NORTH_SOUTH, bids.get(7),"sim5")); //sim5  
	  }
	    
  }
  
}
