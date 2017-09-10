package aim4.im.v2i.batch;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableSet;
import java.util.PriorityQueue;

import aim4.config.Debug;
import aim4.im.TrackModel;
import aim4.im.v2i.RequestHandler.BatchModeRequestHandler.IndexedProposal;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.msg.v2i.Request.Proposal;
import aim4.vehicle.VehicleSimView;

public class WalletBasedReordering implements ReorderingStrategy {

	 /**
	   * The time period between the processing times.
	   */
	  public static final double DEFAULT_PROCESSING_INTERVAL = 2.0;  // seconds

	  /**
	   * The estimated time this reordering strategy takes to compute the
	   * reordering plus the time the intersection manager takes to send
	   * the confirm and reject messages.
	   */
	  private static final double COMP_COMM_DELAY = 0.05; // seconds

	  /**
	   * The amount of lookahead (the period of time between the end time of the
	   * last processed batch and the target batch).
	   */
	  private static final double LOOKAHEAD_TIME = 3.0;  // seconds

	  /**
	   * The amount of time of a batch.
	   */
	  private static final double BATCH_INTERVAL = DEFAULT_PROCESSING_INTERVAL;


	  /////////////////////////////////
	  // PRIVATE FIELDS
	  /////////////////////////////////

	  /**
	   * The next processing time for the next batch.
	   */
	  private double nextProcessingTime;

	  /**
	   * The next proposal deadline for the next batch.
	   */
	  private double nextProposalDeadline;


	  /**
	   * The time period between the processing times.
	   */
	  private double processingInterval = DEFAULT_PROCESSING_INTERVAL;
	  
	  private BidLaneTrack bidLaneTracker;


	  /////////////////////////////////
	  // CONSTRUCTORS
	  /////////////////////////////////

	  /**
	   * Create a road based reordering strategy
	   *
	   * @param processingInterval  the processing interval
	   */
	  public WalletBasedReordering(double processingInterval) {
	    this.processingInterval = processingInterval;
	    bidLaneTracker = new BidLaneTrack();
	  }

	  /////////////////////////////////
	  // PUBLIC METHODS
	  /////////////////////////////////

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public void setInitialTime(double initTime) {
	    nextProcessingTime = initTime + processingInterval;
	    nextProposalDeadline = nextProcessingTime + COMP_COMM_DELAY;
	  }


	  /////////////////////////////////
	  // PUBLIC METHODS
	  /////////////////////////////////

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public List<IndexedProposal> getBatch(double currentTime,
	                                        NavigableSet<IndexedProposal> queue,
	                                        TrackModel trackModel) {
		bidLaneTracker.refreshTraffic(queue, currentTime);
	    List<IndexedProposal> proposals1 = selectProposals(currentTime, queue);
	    List<IndexedProposal> proposals2 = reorderProposals(proposals1);
	    List<IndexedProposal> proposals3 = bidLaneTracker.reorderProposals(proposals2, currentTime);

	    nextProcessingTime = currentTime + processingInterval;
	    nextProposalDeadline = nextProcessingTime + COMP_COMM_DELAY;
	    return proposals3;
	  }

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public double getNextProcessingTime() {
	    return nextProcessingTime;
	  }

	  /**
	   * {@inheritDoc}
	   */
	  @Override
	  public double getNextProposalDeadline() {
	    return nextProposalDeadline;
	  }

	  /////////////////////////////////
	  // PRIVATE METHODS
	  /////////////////////////////////

	  /**
	   * Select the set of proposals in a batch.
	   *
	   * @param currentTime  the current time
	   * @param queue        the message queue
	   * @return the set of proposals in a batch
	   */
	  private List<IndexedProposal> selectProposals(
	                                  double currentTime,
	                                  NavigableSet<IndexedProposal> queue) {
	    List<IndexedProposal> result = new LinkedList<IndexedProposal>();

	    double startTime = currentTime + LOOKAHEAD_TIME;
	    double endTime = startTime + BATCH_INTERVAL;

	    for(IndexedProposal iProposal : queue) {
	      Proposal proposal = iProposal.getProposal();
	      double arrivalTime = proposal.getArrivalTime();
	      if (arrivalTime < endTime) {    // exclude the arrivalTime == maxTime
	        result.add(iProposal);
	      } else {
	        // the remaining proposals in the queue have a larger arrival time
	    	// all proposals are required to total bids for the lane
	    	result.add(iProposal);
	        break;
	      }
	    }

	    return result;
	  }

	  /**
	   * Reorder a list of indexed proposals.
	   *
	   * @param iProposals a list of indexed proposals
	   * @return a reordered list of indexed proposals
	   */
	  private List<IndexedProposal> reorderProposals(
	                                             List<IndexedProposal> iProposals) {
	    // a partition of the proposals according to the road of the arrival lane.
	    Map<Road,List<IndexedProposal>> partition =
	      new HashMap<Road,List<IndexedProposal>>();
	    
	    //bid storage
	    LinkedHashMap<Integer,Double> laneBids = new LinkedHashMap<Integer,Double>();
	    double oldBid = 0;
	    
	    for(IndexedProposal iProposal : iProposals) {
	      int laneId = iProposal.getProposal().getDepartureLaneID();
	      double bid = iProposal.getProposal().getBid();
	      Road road = Debug.currentMap.getRoad(laneId);
	      if (partition.containsKey(road)) {
	        partition.get(road).add(iProposal);
	      } else {
	        List<IndexedProposal> l = new LinkedList<IndexedProposal>();
	        l.add(iProposal);
	        partition.put(road, l);
	      }
	      //add bid to current lane, if bid dosen't exist, add new mapping
	      if(laneBids.isEmpty()){
	    	  laneBids.put(laneId, bid);
	    	  
	      } else if(laneBids.containsKey(laneId)){
	    	  oldBid = laneBids.get(laneId);
	    	  laneBids.replace(laneId, oldBid, oldBid + bid);
	    	  
	      } else {
	    	  laneBids.put(laneId, bid);
	    	  
	      }
	      
	    }
	    //sort list by largest cumulative bid
	    ArrayList<Entry<Integer,Double>> sortList = new ArrayList<Entry<Integer, Double>>(laneBids.entrySet());
	    Collections.sort(sortList, valueComparator);
	    // combine the proposals according to the partition
	    List<IndexedProposal> result = new LinkedList<IndexedProposal>();

	    for(List<IndexedProposal> iProposals2 : partition.values()) {
	      result.addAll(iProposals2);
	    }

	    return result;
	  }
	  Comparator<Entry<Integer, Double>> valueComparator = new Comparator<Entry<Integer,Double>>() {
          
          @Override
          public int compare(Entry<Integer, Double> e1, Entry<Integer, Double> e2) {
              return (int) (e1.getValue() - e2.getValue());
          }
      };

	}
/**
 * 
 * @author Alexander
 * keeps track of the whole set of vehicles ordered by bid and time spent in queue
 */
class BidLaneTrack{
	
	PriorityQueue<LaneOrder> laneList;
	
	
	public BidLaneTrack(){
		Comparator<LaneOrder> laneComparator = new Comparator<LaneOrder>(){

			@Override
			//gives the lane with the highest bid, the lowest score
			public int compare(LaneOrder lane1, LaneOrder lane2) {
				
				return (int)((lane2.getLaneBid() - lane1.getLaneBid())*100);
			}
			
		};
		laneList = new PriorityQueue<LaneOrder>(8,laneComparator);
		
		for(SpawnPoint sp : Debug.currentMap.getSpawnPoints()){
			laneList.add(new LaneOrder(sp.getLane().getId()));
		}
	}
	
	/**
	 * updates bids for each lane in this object
	 * @param currentTime
	 */
	public void updateAllLaneBids(double currentTime){
		for(LaneOrder lane : laneList){
			lane.getLaneBidU(currentTime);
		}
	}
	/**
	 * Resets all existing vehicles to active, adds any new vehicles (set to active) and deletes all non active vehicles
	 * @param traffic
	 */
	public void refreshTraffic(NavigableSet<IndexedProposal> traffic, double currentTime){
		for(IndexedProposal proposal : traffic){
			for(LaneOrder lane : laneList){
				if(lane.getID() == proposal.getProposal().getArrivalLaneID()){
					lane.addVehicle(proposal, currentTime);
					break;
				}
			}
		}
		for(LaneOrder lane : laneList){
			lane.removeInactiveVehicles(currentTime);
		}
	}
	/**
	 * set all vehicles associated with this object to inactive
	 * unless set to active, refresh traffic will remove the vehicle
	 */
	public void setAllToInactive(){
		for(LaneOrder lane : laneList){
			lane.setInactive();
		}
	}
	/**
	 * Takes the current set of selected proposals and orders them based on which lane has the most bidding power.
	 * @param proposals2
	 * @return
	 */
	public ArrayList<IndexedProposal> reorderProposals(List<IndexedProposal> proposals2, double currentTime){
		ArrayList<IndexedProposal> tempStorage = new ArrayList<IndexedProposal>(proposals2);
		ArrayList<IndexedProposal> result = new ArrayList<IndexedProposal>();
		
		updateAllLaneBids(currentTime);
		
		for(LaneOrder lane : laneList){
			for(IndexedProposal proposal : tempStorage){
				if(proposal.getProposal().getArrivalLaneID() == lane.getID()){
					result.add(proposal);
				}
			}
		}
		return result;
	}
}

/**
 * 
 * @author Alexander
 * groups vehicles by lane and keeps effective bids up to date
 */
class LaneOrder {
	private LinkedList<QueueVehicle> laneVehicles;
	private double laneBid;
	private int laneId;
	
	LaneOrder(int laneId){
		laneVehicles = new LinkedList<QueueVehicle>();
		laneBid = 0;
		this.laneId = laneId;
	}

	public int getID() {
		return laneId;
	}

	public ArrayList<QueueVehicle> getLaneVehicles(){
		return new ArrayList<QueueVehicle>(this.laneVehicles);
	}
	
	public double getLaneBid(){
		return laneBid;
	}
	
	/**
	 * updates the current bid for the lane and returns the result
	 * @param currentTime
	 * @return current Lane Bids
	 */
	public double getLaneBidU(double currentTime){
		double bidSubTotal = 0;
		for(QueueVehicle vehicle : laneVehicles){
			bidSubTotal += vehicle.effectiveBid(currentTime);
		}
		this.laneBid = bidSubTotal;
		return bidSubTotal;
	}
	

	public void addVehicle(IndexedProposal proposal, double currentTime){
		for(QueueVehicle vehicle :laneVehicles){
			if(vehicle.getVin() == proposal.getRequest().getVin()){
				vehicle.setIsActive(true);
				return;
			}
		}
		QueueVehicle queueVehicle = new QueueVehicle(proposal,currentTime);
		laneBid += queueVehicle.getBid();
		laneVehicles.addFirst(queueVehicle);
		
	}
	public void removeInactiveVehicles(double currentTime){
		
		ArrayList<QueueVehicle> tempList = new ArrayList<QueueVehicle>();
		for(QueueVehicle vehicle : tempList){
			if(!vehicle.isActive()){
				laneVehicles.remove(vehicle);
				getLaneBidU(currentTime);
			}
		}
	}
	
	public void setInactive(){
		for(QueueVehicle vehicle : laneVehicles){
			vehicle.setIsActive(false);
		}
	}
}
/**
 * 
 * @author Alexander
 * stores the vehicle's bid and time since entry of queue
 */
class QueueVehicle {
	
	private int vin;
	private IndexedProposal proposal;
	private double bid;
	private double timeEntered;
	private boolean isActive;
	
	QueueVehicle(int vin, IndexedProposal proposal, double timeEntered){
		this.vin = vin;
		this.proposal = proposal;
		bid = proposal.getProposal().getBid();
		this.timeEntered = timeEntered;
		this.isActive = true;
	}
	
	QueueVehicle(IndexedProposal proposal, double timeEntered){
		this.vin = proposal.getRequest().getVin();
		this.proposal = proposal;
		bid = proposal.getProposal().getBid();
		this.timeEntered = timeEntered;
		this.isActive = true;
	}
	
	public int getVin(){
		return vin;
	}
	public double getBid(){
		return bid;
	}
	public double getTimeEntered(){
		return timeEntered;
	}
	public double getTimeCost(double currentTime){
		return (Math.abs(currentTime - timeEntered) / 100);
	}
	public double effectiveBid(double currentTime){
		return getTimeEntered() + getTimeCost(currentTime);
	}
	public IndexedProposal getIProposal(){
		return proposal;
	}
	public boolean setIProposal(IndexedProposal proposal){
		if(this.vin == proposal.getRequest().getVin()){
			this.proposal = proposal;
			return true;
		} else {
			return false;
		}
	}
	public boolean isActive(){
		return isActive;
	}
	public void setIsActive(boolean isActive){
		this.isActive = isActive;
	}
}

