package aim4.im.v2i.batch;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.NavigableSet;

import aim4.config.Debug;
import aim4.im.v2i.RequestHandler.BatchModeRequestHandler.IndexedProposal;
import aim4.map.SpawnPoint;

//add protected boolean to stop reject message cars from being purged from the list

/**
 * 
 * @author Alexander
 * keeps track of the whole set of vehicles ordered by bid and time spent in queue
 */
public class BidLaneTrack{
	
	private ArrayList<LaneOrder> laneList;
	private Comparator<LaneOrder> laneComparator;
	
	public BidLaneTrack(){
		laneComparator = new Comparator<LaneOrder>(){

			@Override
			//gives the lane with the highest bid, the lowest score
			public int compare(LaneOrder lane1, LaneOrder lane2) {
				
				return (int)((lane2.getLaneBid() - lane1.getLaneBid())*100);
			}
			
		};
		laneList = new ArrayList<LaneOrder>(8);
		
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
		//update priorityQueue
		Collections.sort(laneList, laneComparator);
		return;
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
		
		ArrayList<QueueVehicle> tempList = new ArrayList<QueueVehicle>(laneVehicles);
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

